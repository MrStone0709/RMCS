#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

namespace {

/// 读取可选参数：yaml 里配置了就采用配置值，否则声明并返回默认值。
template <typename T>
auto get_param_or(rclcpp::Node& node, const std::string& name, T default_value) -> T {
    if (!node.has_parameter(name))
        node.declare_parameter<T>(name, default_value);
    return node.get_parameter(name).get_value<T>();
}

} // namespace

/// @brief 龙门发射架硬件层：C 板 CAN1 上的 3 台 M2006+C610（ID 1/2/3）+ DR16 + 2 路底部限位。
///
/// 职责边界（本组件**不含 PID**，三路速度环由 yaml 里的 PidController 组件完成）：
///
/// 1. 板级 IO
///    - CAN1 每周期下发 **一帧** `0x200`，同时装三台电机的电流命令 → 命令时间一致性最好；
///    - 反馈 `0x201/0x202/0x203` → `/gimbal/{yaw,pitch_left,pitch_right}/...`（DjiMotor 提供）；
///    - DBUS → device::Dr16 → device::RemoteControl（`/remote/*`）。
/// 2. 摇杆映射（深度 ∝ 速度）
///    - 左摇杆**物理竖直轴**（前推/后拉）→ 俯仰速度给定；
///    - 左摇杆**物理水平轴**（左推/右推）→ 偏航速度给定；
///    - 所有速度给定都用**电机输出轴 rad/s**，与 PidController 的 measurement 同量纲，
///      不经过任何几何换算（几何参数只用于角度显示、歪斜量与自动限位）。
///    - 两台 pitch 电机在**同一 tick 用同一个摇杆值**各自算出给定 → 天然"同步前进"。
/// 3. 自动找最高点（上电自检）
///    - 进程启动后的**启动窗口**内（homing_start_window_ticks）具备条件就俯仰持续上升，
///      检测到**持续堵转**（推得动但不动）即认定到达最高点，停转，并把此时的俯仰角记为
///      **上限软限位**；此后摇杆不能再把俯仰推过该角度。
///    - 窗口过后不再自动开始，必须用 `/gimbal/home` 显式触发；这对"长时间待机（遥控关着，
///      状态停在 IDLE）→ 回来一开遥控 → 机构突然满速上升"这种事故很重要。
///    - 上升期间出现硬件故障（反馈超时/过温）立即中止判失败；被拨杆/遥控中断超时也判失败。
///    - 未找到最高点（未标定/失败）时，摇杆向上的速度再乘 unhomed_up_speed_scale 限速。
///    - 下限软限位本版本取消（yaml 里已无该项）；下行保护依赖底部限位开关（若已接）。
/// 4. 安全保护（触发时把该轴速度给定置 NaN）
///    - 任一电机 CAN 反馈超时 / 过温 → 全部失能（fault）；
///    - DBUS 失联 / 拨杆双 DOWN → 全部失能（disabled，不算 fault）；
///    - 底部限位开关 → **单侧**给定归零（另一侧继续），因此"两侧一起压到底"可以重新对齐。
///
/// NaN 为什么会变成"安全停机"：PidController 的 err = setpoint - measurement 得到 NaN，
/// PidCalculator 对非有限误差直接返回 NaN，DjiMotor::generate_command 里对 NaN 显式输出
/// 0 电流，等效于失能（dji_motor.hpp 的 NaN 分支）。
///
/// 接口一览（yaml 里按这些名字接线）：
///   ROS 服务/话题：/gimbal/calibrate（std_msgs/Int32：0 全部/1 yaw/2 左/3 右）零标定
///                /gimbal/home（std_msgs/Int32：非 0 = 手动触发重新找最高点，0 = 取消）
///   输出（状态）：/gimbal/yaw/{angle,velocity,torque,max_torque}
///                /gimbal/pitch_left/{angle,velocity,torque,max_torque}
///                /gimbal/pitch_right/{angle,velocity,torque,max_torque}
///                以上 angle/velocity 均为**电机输出轴**的 rad / rad·s^-1（DjiMotor 约定，已含 36:1）。
///   输出（机构量）：/gimbal/yaw/angle_rad, /gimbal/pitch/angle_rad
///                /gimbal/pitch/{height_left_m,height_right_m,skew_m,skew_rad}
///                /gimbal/pitch/upper_limit_rad（自动测得的最高点，未测出时为 yaml 兜底值）
///   输出（给定）：/gimbal/yaw/control_velocity
///                /gimbal/pitch_left/control_velocity, /gimbal/pitch_right/control_velocity
///                /gimbal/pitch/control_velocity（公共值，仅观测）
///                以上速度给定均为**电机输出轴** rad/s，直接喂给 PidController 的 setpoint。
///   输出（诊断）：/gimbal/{control_mode,fault,max_temperature,homing_state}
///                /gimbal/stick/{pitch_axis,yaw_axis}（摇杆解析结果，排查摇杆问题用）
///                /gimbal/pitch_left/limit_switch, /gimbal/pitch_right/limit_switch
///   输入（控制）：/gimbal/yaw/control_torque, /gimbal/pitch_left/control_torque,
///                /gimbal/pitch_right/control_torque      （由 PidController 写入）
class LauncherGantry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    LauncherGantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , command_(
              create_partner_component<LauncherGantryCommand>(
                  get_component_name() + "_command", *this))
        , yaw_motor_(*this, *command_, "/gimbal/yaw")
        , pitch_left_motor_(*this, *command_, "/gimbal/pitch_left")
        , pitch_right_motor_(*this, *command_, "/gimbal/pitch_right") {

        // ---- 1. 机构几何：只用于角度显示、歪斜量与自动限位（速度给定不经过它）----
        screw_lead_m_per_turn_ = get_param_or(*this, "screw_lead_mm_per_turn", 2.0) / 1000.0;
        pitch_rad_per_meter_ = get_param_or(*this, "pitch_rad_per_meter", 2.5);
        // 丝杆转一圈 → 横梁升高 screw_lead，对应俯仰角变化 lead × rad_per_meter
        pitch_rad_per_motor_turn_ = screw_lead_m_per_turn_ * pitch_rad_per_meter_;
        // 偏航传动比：只影响 /gimbal/yaw/angle_rad 的物理含义与（默认关闭的）偏航软限位
        yaw_rad_per_motor_turn_ =
            get_param_or(*this, "yaw_rad_per_motor_turn", 2.0 * std::numbers::pi);
        if (!(pitch_rad_per_motor_turn_ > 1e-9) || !(yaw_rad_per_motor_turn_ > 1e-9))
            throw std::runtime_error(
                "[launcher] screw_lead_mm_per_turn / pitch_rad_per_meter / yaw_rad_per_motor_turn "
                "必须为正");

        pitch_zero_rad_ = get_param_or(*this, "pitch_zero_rad", 0.0);
        yaw_zero_rad_ = get_param_or(*this, "yaw_zero_rad", 0.0);

        // ---- 2. 软限位 ----
        // 俯仰上限：homing 开启且成功时用自动测得的最高点，否则用 yaml 的 pitch_max_rad 兜底
        pitch_max_rad_ = get_param_or(*this, "pitch_max_rad", 0.6);
        pitch_upper_limit_rad_ = pitch_max_rad_;
        // 俯仰下限：本版本取消（"最低点软限位暂时取消"），下行保护依赖底部限位开关
        // 偏航软限位默认关闭：偏航传动比未知时它极易误触发（表现为"摇杆推不动偏航电机"）
        yaw_limit_enabled_ = get_param_or(*this, "yaw_limit_enabled", false);
        yaw_min_rad_ = get_param_or(*this, "yaw_min_rad", -0.5);
        yaw_max_rad_ = get_param_or(*this, "yaw_max_rad", 0.5);
        if (yaw_limit_enabled_ && !(yaw_min_rad_ < yaw_max_rad_))
            throw std::runtime_error("[launcher] 偏航软限位顺序错误（要求 min < max）");

        // ---- 3. 三台电机：kM2006 + C610，全部挂在 CAN1，ID 1/2/3 ----
        // @note 这里不要改 reduction_ratio：DjiMotor 的力矩换算系数与它相关，
        //       M2006 的默认值 36 同时给出正确的输出轴角与 1.8 N·m 力矩上限。
        configure_motor(yaw_motor_, "yaw_motor");
        configure_motor(pitch_left_motor_, "pitch_left_motor");
        configure_motor(pitch_right_motor_, "pitch_right_motor");

        // ---- 4. 摇杆映射参数（速度单位一律是电机输出轴 rad/s）----
        stick_deadzone_ = get_param_or(*this, "stick_deadzone", 0.02);
        pitch_axis_sign_ = get_param_or(*this, "pitch_axis_sign", 1.0);
        yaw_axis_sign_ = get_param_or(*this, "yaw_axis_sign", 1.0);
        max_pitch_velocity_motor_ = get_param_or(*this, "max_pitch_velocity_motor", 50.0);
        max_yaw_velocity_motor_ = get_param_or(*this, "max_yaw_velocity_motor", 50.0);
        slow_speed_scale_ = get_param_or(*this, "slow_speed_scale", 0.35);

        // ---- 5. 自动找最高点 ----
        homing_enabled_ = get_param_or(*this, "homing_enabled", true);
        homing_rise_velocity_ = get_param_or(*this, "homing_rise_velocity", 40.0);
        homing_stall_velocity_ = get_param_or(*this, "homing_stall_velocity", 2.0);
        homing_stall_torque_ = get_param_or(*this, "homing_stall_torque", 0.4);
        homing_stall_confirm_ticks_ = get_param_or(*this, "homing_stall_confirm_ticks", 100);
        homing_timeout_ticks_ = get_param_or(*this, "homing_timeout_ticks", 20000);
        homing_require_remote_ = get_param_or(*this, "homing_require_remote", true);
        homing_backoff_ticks_ = get_param_or(*this, "homing_backoff_ticks", 0);
        homing_backoff_velocity_ = get_param_or(*this, "homing_backoff_velocity", 20.0);
        // 自动开始的时限：进程启动后这么多周期内允许自动上升；超过后必须用 /gimbal/home 显式触发。
        // 这条是防"长时间待机（遥控关着）→ 回来一开遥控 → 机构突然满速上升"。
        homing_start_window_ticks_ = get_param_or(*this, "homing_start_window_ticks", 60000);
        // 上升过程中被中断（拨杆失能/遥控离线）超过这么多周期就判定失败并停转，不再自动续上
        homing_interrupt_abort_ticks_ = get_param_or(*this, "homing_interrupt_abort_ticks", 2000);
        // 还没找到最高点（未标定/失败）时，摇杆向上的速度按这个比例限制，避免盲目冲顶
        unhomed_up_speed_scale_ = get_param_or(*this, "unhomed_up_speed_scale", 0.25);
        if (!homing_enabled_) {
            homing_state_ = HomingState::kDisabled;
            RCLCPP_WARN(
                logger_, "[launcher] 自动找最高点已关闭，俯仰上限取 yaml 的 %.4f rad",
                pitch_max_rad_);
        }

        // ---- 6. 安全参数 ----
        signal_timeout_ns_ = static_cast<std::int64_t>(
            get_param_or(*this, "signal_timeout_ms", 100.0) * 1'000'000.0);
        max_temperature_ = get_param_or(*this, "max_temperature", 70.0);

        // ---- 7. 限位开关：C 板 GPIO 通道 0..6（PWM1..PWM7），-1 表示未接/不启用 ----
        pitch_left_limit_gpio_ = get_param_or(*this, "pitch_left_limit_gpio", -1);
        pitch_right_limit_gpio_ = get_param_or(*this, "pitch_right_limit_gpio", -1);
        check_gpio_index(pitch_left_limit_gpio_, "pitch_left_limit_gpio");
        check_gpio_index(pitch_right_limit_gpio_, "pitch_right_limit_gpio");
        limit_active_high_ = get_param_or(*this, "limit_active_high", true);
        limit_pull_up_ = get_param_or(*this, "limit_pull_up", true);
        limit_read_period_ms_ = get_param_or(*this, "limit_read_period_ms", 20);

        // ---- 8. 状态与诊断输出 ----
        register_output("/gimbal/yaw/angle_rad", yaw_angle_rad_, 0.0);
        register_output("/gimbal/pitch/angle_rad", pitch_angle_rad_, 0.0);
        register_output("/gimbal/pitch/height_left_m", height_left_m_, 0.0);
        register_output("/gimbal/pitch/height_right_m", height_right_m_, 0.0);
        register_output("/gimbal/pitch/skew_m", skew_m_, 0.0);
        register_output("/gimbal/pitch/skew_rad", skew_rad_, 0.0);
        register_output("/gimbal/pitch/upper_limit_rad", pitch_upper_limit_output_, pitch_max_rad_);

        register_output("/gimbal/pitch_left/limit_switch", pitch_left_limit_, 0.0);
        register_output("/gimbal/pitch_right/limit_switch", pitch_right_limit_, 0.0);
        register_output("/gimbal/max_temperature", max_temperature_output_, 0.0);
        register_output("/gimbal/control_mode", control_mode_, 0.0);
        register_output("/gimbal/fault", fault_, 1.0);
        register_output("/gimbal/homing_state", homing_state_output_, 0.0);
        // 发送异常计数：librmcs 在 USB 发送缓冲不可用时是**静默丢帧**（记一条 error 就返回成功），
        // "CAN transmission failed" 这种抛出才是这里能计到的；它能间接反映链路健康度。
        register_output("/gimbal/tx_failures", tx_failures_output_, 0.0);

        // 摇杆解析结果：排查"摇杆推了但电机不动"时先看这两个量
        register_output("/gimbal/stick/pitch_axis", pitch_stick_output_, 0.0);
        register_output("/gimbal/stick/yaw_axis", yaw_stick_output_, 0.0);

        // ---- 9. 速度给定输出（单位：电机输出轴 rad/s，与 DjiMotor 的 velocity 同量纲）----
        register_output("/gimbal/yaw/control_velocity", yaw_control_velocity_, nan_);
        register_output("/gimbal/pitch/control_velocity", pitch_control_velocity_, nan_);
        register_output("/gimbal/pitch_left/control_velocity", pitch_left_control_velocity_, nan_);
        register_output(
            "/gimbal/pitch_right/control_velocity", pitch_right_control_velocity_, nan_);

        // ---- 10. 零标定入口：把机械放到参考位后调用，把打印出来的值写进 yaml ----
        // @note 角度是"多圈增量"，上电时为 0；因此**上电前机构必须停在标定参考位**
        //       （RMCS 各机器人的既有约定），否则角度基准会整体偏移。
        // @note QoS 用 depth=1（而不是 QoS{0}）：后者会触发 rclcpp 的
        //       "A zero depth with KEEP_LAST doesn't make sense" 警告。
        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{1}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

        // ---- 10b. 手动触发/取消自动找最高点 ----
        // @note 进程启动 homing_start_window_ticks 之后，自动上升不再自行开始，必须用这个入口
        //       显式触发（data != 0 触发，data == 0 取消）。这样"长时间待机后回到机器旁边"
        //       不会因为遥控刚上线就让机构突然满速上升。
        gimbal_home_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/home", rclcpp::QoS{1}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_home_subscription_callback(std::move(msg));
            });

        // ---- 11. 遥控 ----
        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        // ---- 12. 板卡：CBoard 在构造期间就可能触发回调，必须放在构造函数最后 ----
        auto options = librmcs::board::AdvancedOptions{};
        options.dangerously_skip_version_checks = get_param_or(*this, "skip_version_check", false);
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_param_or(*this, "board_serial", std::string{}), options);

        // ---- 13. 限位开关周期读取（板卡已就绪后才能下发配置）----
        {
            auto builder = board_->start_transmit();
            const auto pull =
                limit_pull_up_ ? librmcs::data::GpioPull::kUp : librmcs::data::GpioPull::kNone;
            const auto period = static_cast<std::uint16_t>(limit_read_period_ms_);
            if (pitch_left_limit_gpio_ >= 0)
                builder.gpio_digital_read(
                    Spec::kGpios[static_cast<std::size_t>(pitch_left_limit_gpio_)],
                    {.period_ms = period, .pull = pull});
            if (pitch_right_limit_gpio_ >= 0)
                builder.gpio_digital_read(
                    Spec::kGpios[static_cast<std::size_t>(pitch_right_limit_gpio_)],
                    {.period_ms = period, .pull = pull});
        }

        RCLCPP_INFO(
            logger_,
            "[launcher] board ready: CAN1, yaw=id%u pitch_left=id%u pitch_right=id%u | "
            "pitch %.4f rad/turn(电机轴) | 限位 GPIO: L=%d R=%d | 自动找最高点=%s",
            static_cast<unsigned>(yaw_motor_.id()),
            static_cast<unsigned>(pitch_left_motor_.id()),
            static_cast<unsigned>(pitch_right_motor_.id()), pitch_rad_per_motor_turn_,
            pitch_left_limit_gpio_, pitch_right_limit_gpio_, homing_enabled_ ? "开" : "关");
    }

    LauncherGantry(const LauncherGantry&) = delete;
    LauncherGantry& operator=(const LauncherGantry&) = delete;
    LauncherGantry(LauncherGantry&&) = delete;
    LauncherGantry& operator=(LauncherGantry&&) = delete;
    ~LauncherGantry() override = default;

    /// @brief 采样 + 自动找最高点 + 摇杆映射 + 安全判断，由 executor 按 1 kHz 周期调用。
    /// @note 本组件在拓扑序中先于三个 PidController（它们要读本组件发布的速度给定与速度反馈），
    ///       而下发 CAN 的伴生组件 LauncherGantryCommand 又依赖 PidController 的力矩输出，
    ///       因此整条链路是 采样 → 给定 → PID → 下发，不需要额外同步。
    void update() override {
        dr16_.update_status();
        remote_control_->update();

        yaw_motor_.update_status();
        pitch_left_motor_.update_status();
        pitch_right_motor_.update_status();

        const auto now = now_ns();
        const bool dbus_fresh = is_fresh(last_dbus_ns_.load(std::memory_order_relaxed), now);
        const bool feedback_fresh = is_fresh(last_yaw_can_ns_.load(std::memory_order_relaxed), now)
                                 && is_fresh(last_pitch_left_can_ns_.load(
                                                 std::memory_order_relaxed), now)
                                 && is_fresh(last_pitch_right_can_ns_.load(
                                                 std::memory_order_relaxed), now);

        const double temperature = std::max(
            {yaw_motor_.temperature(), pitch_left_motor_.temperature(),
             pitch_right_motor_.temperature()});
        const bool over_temperature = feedback_fresh && temperature >= max_temperature_;
        const bool hardware_ok = feedback_fresh && !over_temperature;

        // ---- 机构量换算（角度显示、歪斜量、自动限位）----
        const double yaw_angle = motor_angle_to_launcher(
            yaw_motor_.angle(), yaw_rad_per_motor_turn_, yaw_zero_rad_);
        const double pitch_left_angle = motor_angle_to_launcher(
            pitch_left_motor_.angle(), pitch_rad_per_motor_turn_, pitch_zero_rad_);
        const double pitch_right_angle = motor_angle_to_launcher(
            pitch_right_motor_.angle(), pitch_rad_per_motor_turn_, pitch_zero_rad_);
        const double pitch_angle = 0.5 * (pitch_left_angle + pitch_right_angle);

        const double height_left =
            pitch_left_motor_.angle() / (2.0 * std::numbers::pi) * screw_lead_m_per_turn_;
        const double height_right =
            pitch_right_motor_.angle() / (2.0 * std::numbers::pi) * screw_lead_m_per_turn_;
        const double skew = height_left - height_right;

        *yaw_angle_rad_ = finite_or(yaw_angle, 0.0);
        *pitch_angle_rad_ = finite_or(pitch_angle, 0.0);
        *height_left_m_ = finite_or(height_left, 0.0);
        *height_right_m_ = finite_or(height_right, 0.0);
        *skew_m_ = finite_or(skew, 0.0);
        *skew_rad_ = finite_or(skew, 0.0) * pitch_rad_per_meter_;
        *max_temperature_output_ = finite_or(temperature, 0.0);
        *tx_failures_output_ = static_cast<double>(tx_failures_);

        // ---- 底部限位开关 ----
        const bool left_limit_active = limit_is_active(
            pitch_left_limit_gpio_, pitch_left_limit_raw_.load(std::memory_order_relaxed));
        const bool right_limit_active = limit_is_active(
            pitch_right_limit_gpio_, pitch_right_limit_raw_.load(std::memory_order_relaxed));
        *pitch_left_limit_ = left_limit_active ? 1.0 : 0.0;
        *pitch_right_limit_ = right_limit_active ? 1.0 : 0.0;

        // ---- 模式仲裁 ----
        const auto mode = decode_mode(hardware_ok, dbus_fresh);
        if (mode != last_mode_) {
            RCLCPP_WARN(
                logger_, "[launcher] control mode: %s -> %s", mode_name(last_mode_),
                mode_name(mode));
            last_mode_ = mode;
        }
        *control_mode_ = static_cast<double>(static_cast<int>(mode));
        *fault_ = (mode == Mode::kFault) ? 1.0 : 0.0;

        // ---- 摇杆解析（无论使能与否都发布，方便排查摇杆问题）----
        // @note device/dr16.hpp 里名字与物理轴是反的：
        //       joystick_left().x() <- ch3 = 物理竖直轴（前推/后拉）
        //       joystick_left().y() <- ch2 = 物理水平轴（左推/右推，内部取了负号）
        //       所以"竖直→俯仰、水平→偏航"要用 .x()/.y()，不要按名字配对。
        const double pitch_stick = stick(dr16_.joystick_left().x() * pitch_axis_sign_);
        const double yaw_stick = stick(dr16_.joystick_left().y() * yaw_axis_sign_);
        *pitch_stick_output_ = pitch_stick;
        *yaw_stick_output_ = yaw_stick;

        // ---- 自动找最高点：可能需要接管俯仰给定（此时忽略俯仰摇杆）----
        // @note 只有"本周期确实允许运动"时才推进状态机，否则暂停（含计时）：
        //       否则在 Disabled/Fault 期间空转会把 homing_timeout_ticks 耗光，造成假超时。
        //       homing_require_remote=false 时只要求硬件正常，供台架无遥控调试。
        const bool motion_allowed = (mode == Mode::kFull || mode == Mode::kSlow);
        const bool homing_allowed =
            homing_require_remote_ ? motion_allowed : hardware_ok;
        double pitch_velocity_motor = 0.0;
        const bool homing_takes_over = update_homing(
            homing_allowed, mode == Mode::kFault, pitch_angle, pitch_left_motor_.velocity(),
            pitch_right_motor_.velocity(), pitch_left_motor_.torque(),
            pitch_right_motor_.torque(), pitch_velocity_motor);
        *homing_state_output_ = static_cast<double>(static_cast<int>(homing_state_));
        *pitch_upper_limit_output_ = pitch_upper_limit_rad_;

        if (mode == Mode::kFault || mode == Mode::kDisabled) {
            // NaN 会一路传到 DjiMotor 的 NaN 分支，输出 0 电流（等效失能、且不会污染 PID 积分）
            *yaw_control_velocity_ = nan_;
            *pitch_control_velocity_ = nan_;
            *pitch_left_control_velocity_ = nan_;
            *pitch_right_control_velocity_ = nan_;
            return;
        }

        const double speed_scale = (mode == Mode::kSlow) ? slow_speed_scale_ : 1.0;

        double yaw_velocity_motor = max_yaw_velocity_motor_ * speed_scale * yaw_stick;
        if (!homing_takes_over)
            pitch_velocity_motor = max_pitch_velocity_motor_ * speed_scale * pitch_stick;

        // ---- 俯仰上限软限位：自动找最高点成功后用自动值，否则用 yaml 兜底值 ----
        // @note 找最高点过程中不施加该限位（正在往上找），由 update_homing 接管。
        if (!homing_takes_over && pitch_velocity_motor > 0.0
            && pitch_angle >= pitch_upper_limit_rad_) {
            RCLCPP_WARN_THROTTLE(
                logger_, *get_clock(), 1000,
                "[launcher] 俯仰已达上限 %.4f rad（当前 %.4f），不再上升", pitch_upper_limit_rad_,
                pitch_angle);
            pitch_velocity_motor = 0.0;
        }
        // ---- 还没找到最高点时的额外保护：向上的速度按 unhomed_up_speed_scale 限速 ----
        // 目的是把"任何原因导致俯仰被持续往上推"的后果限制成慢速爬升，而不是满速冲顶。
        if (!homing_takes_over && !pitch_upper_limit_valid_ && pitch_velocity_motor > 0.0) {
            RCLCPP_WARN_THROTTLE(
                logger_, *get_clock(), 2000,
                "[launcher] 尚未找到最高点（homing_state=%s），俯仰上升限速到 %.0f%%",
                homing_state_name(homing_state_), unhomed_up_speed_scale_ * 100.0);
            pitch_velocity_motor *= unhomed_up_speed_scale_;
        }
        // ---- 偏航软限位（默认关闭；开启后撞限位只禁止继续朝外）----
        if (yaw_limit_enabled_) {
            if ((yaw_angle >= yaw_max_rad_ && yaw_velocity_motor > 0.0)
                || (yaw_angle <= yaw_min_rad_ && yaw_velocity_motor < 0.0)) {
                RCLCPP_WARN_THROTTLE(
                    logger_, *get_clock(), 1000,
                    "[launcher] 偏航软限位：给定置零（当前 %.3f rad）", yaw_angle);
                yaw_velocity_motor = 0.0;
            }
        }

        // ---- 公共给定（观测用；yaml 里的两个 pitch PID 用的是下面按侧修正过的名字）----
        *pitch_control_velocity_ = pitch_velocity_motor;
        *yaw_control_velocity_ = yaw_velocity_motor;

        // ---- 底部限位：**单侧**归零。一侧压到底先停，另一侧继续走，
        //      因此"两侧一起往下压到底"就是把累积歪斜重新对齐的操作。----
        double pitch_left_velocity_motor = pitch_velocity_motor;
        double pitch_right_velocity_motor = pitch_velocity_motor;
        if (left_limit_active && pitch_left_velocity_motor < 0.0)
            pitch_left_velocity_motor = 0.0;
        if (right_limit_active && pitch_right_velocity_motor < 0.0)
            pitch_right_velocity_motor = 0.0;
        *pitch_left_control_velocity_ = pitch_left_velocity_motor;
        *pitch_right_control_velocity_ = pitch_right_velocity_motor;

        if ((left_limit_active || right_limit_active) && pitch_velocity_motor < 0.0) {
            RCLCPP_WARN_THROTTLE(
                logger_, *get_clock(), 2000,
                "[launcher] 底部限位触发(L=%d R=%d)：该侧禁止继续下降，歪斜 %.3f mm",
                static_cast<int>(left_limit_active), static_cast<int>(right_limit_active),
                skew * 1000.0);
        }
    }

    /// @brief 由伴生组件在"三路 PID 算完之后"调用：一帧 0x200 下发三路电流。
    ///
    /// @warning 两条关于"命令发不出去"的事实，必须知道：
    /// 1. librmcs 的 `write_can` 在 USB 发送缓冲不可用时是**静默丢帧**：
    ///    `handler.cpp` 的 `process_result()` 对 `kBadAlloc` 只记一条
    ///    "Transmit buffer unavailable (acquire failed)" 日志，然后 **返回 true**，
    ///    因此 `can_transmit()` 不会抛异常，我们也无法从这里感知到丢帧。
    ///    → 结果是 ESC 收不到新指令，而 C610 会**保持上一条电流指令**；
    ///      如果上一条正好是"持续向上"，就会表现为俯仰自行满速上升。
    /// 2. 所以本组件只能做两件事：try/catch 兜住真正会抛的失败（保护 executor 不被打挂），
    ///    以及把"命令必须尽快变成 0"的窗口尽量缩短（见 update_homing 的启动窗口/故障即停）。
    void command_update() {
        try {
            // CAN ID 1..4 共用 0x200（C610/M2006），槽位是 (id - 1) % 4
            device::CanPacket8 packet{uint64_t{0}};
            packet.data[(yaw_motor_.id() - 1) % 4] = command_of(yaw_motor_);
            packet.data[(pitch_left_motor_.id() - 1) % 4] = command_of(pitch_left_motor_);
            packet.data[(pitch_right_motor_.id() - 1) % 4] = command_of(pitch_right_motor_);

            board_->start_transmit().can_transmit(
                Spec::kCans.kCan1, //
                {.can_id = 0x200, .can_data = packet.as_bytes()});
        } catch (const std::exception& exception) {
            // 真正的抛出（如 CAN 数据非法）绝不能逃出本函数：executor 的更新线程会
            // 直接 FATAL 关机，进程被 launch respawn 后又会在新进程里重新找最高点。
            ++tx_failures_;
            RCLCPP_ERROR_THROTTLE(
                logger_, *get_clock(), 1000,
                "[launcher] 下发 CAN 帧失败（累计 %lld 次）：%s", static_cast<long long>(tx_failures_),
                exception.what());
        }
    }

private:
    /// 控制模式。数值会出现在 /gimbal/control_mode 上，便于 Foxglove 观察。
    enum class Mode : int {
        kFault = 0,    ///< 电机反馈超时 / 过温 → 全部失能
        kDisabled = 1, ///< 遥控失联或拨杆双 DOWN → 全部失能
        kFull = 2,     ///< 右拨杆 UP：满速
        kSlow = 3,     ///< 右拨杆 MIDDLE：按 slow_speed_scale 限速（精细模式）
    };

    /// 自动找最高点的状态。数值会出现在 /gimbal/homing_state 上。
    enum class HomingState : int {
        kIdle = 0,     ///< 等待条件满足后开始上升
        kRising = 1,   ///< 正在持续上升
        kBackoff = 2,  ///< 已找到最高点，正在回退（homing_backoff_ticks > 0 时才有）
        kDone = 3,     ///< 已找到最高点，上限软限位已生效
        kFailed = 4,   ///< 超时未找到（俯仰停转，上限退回 yaml 兜底值）
        kDisabled = 5, ///< 配置里关闭了自动找最高点
    };

    static auto mode_name(Mode mode) -> const char* {
        switch (mode) {
        case Mode::kFault: return "FAULT";
        case Mode::kDisabled: return "DISABLED";
        case Mode::kFull: return "FULL";
        case Mode::kSlow: return "SLOW";
        }
        return "?";
    }

    static auto homing_state_name(HomingState state) -> const char* {
        switch (state) {
        case HomingState::kIdle: return "IDLE";
        case HomingState::kRising: return "RISING";
        case HomingState::kBackoff: return "BACKOFF";
        case HomingState::kDone: return "DONE";
        case HomingState::kFailed: return "FAILED";
        case HomingState::kDisabled: return "DISABLED";
        }
        return "?";
    }

    /// @brief 三台电机统一配置：kM2006 + 多圈角（多圈角是角度换算/自动限位的前提）。
    void configure_motor(device::DjiMotor& motor, const std::string& prefix) {
        const auto id = get_param_or(*this, prefix + ".motor_id", 1);
        if (id < 1 || id > 4)
            throw std::runtime_error(
                "[launcher] " + prefix + ".motor_id 必须在 1..4（0x200 帧只覆盖 ID 1~4）");

        auto config =
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, static_cast<std::uint8_t>(id)}
                .set_encoder_zero_point(get_param_or(*this, prefix + ".encoder_zero_point", 0))
                .enable_multi_turn_angle();
        if (get_param_or(*this, prefix + ".reversed", false))
            config.set_reversed();
        motor.configure(config);
    }

    void check_gpio_index(int index, const std::string& name) const {
        if (index < -1 || index > 6)
            throw std::runtime_error(
                "[launcher] " + name + " 必须在 0..6（C 板 PWM1..PWM7）或 -1（不启用）");
    }

    /// 电机输出轴角 → 机构角。
    static auto motor_angle_to_launcher(
        double motor_angle_rad, double rad_per_turn, double zero_rad) -> double {
        return zero_rad + motor_angle_rad / (2.0 * std::numbers::pi) * rad_per_turn;
    }

    /// 摇杆深度 → [-1, 1]，带死区并重新归一化（照搬 gm6020 板的做法）。
    auto stick(double axis) const -> double {
        if (!std::isfinite(axis) || std::abs(axis) <= stick_deadzone_)
            return 0.0;
        return std::copysign((std::abs(axis) - stick_deadzone_) / (1.0 - stick_deadzone_), axis);
    }

    auto decode_mode(bool hardware_ok, bool dbus_fresh) const -> Mode {
        using rmcs_msgs::Switch;

        if (!hardware_ok)
            return Mode::kFault;
        if (!dbus_fresh)
            return Mode::kDisabled;

        const auto switch_left = dr16_.switch_left();
        const auto switch_right = dr16_.switch_right();
        if (switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            return Mode::kDisabled;
        if (switch_left == Switch::DOWN && switch_right == Switch::DOWN)
            return Mode::kDisabled;

        switch (switch_right) {
        case Switch::UP: return Mode::kFull;
        case Switch::MIDDLE: return Mode::kSlow;
        default: return Mode::kDisabled;
        }
    }

    /// @brief 自动找最高点状态机：上电后持续上升 → 持续堵转即认定到顶。
    ///
    /// 堵转判据（两台 pitch 电机都要满足）：|速度| < homing_stall_velocity 且 |力矩| >
    /// homing_stall_torque，连续 homing_stall_confirm_ticks 个周期；与
    /// controller/chassis/climber/stick_group.hpp 的 blocked 判据同一思路。
    ///
    /// 三条额外的安全规则（针对"长时间待机后机构突然满速上升"的事故）：
    /// 1. **启动窗口**：进程启动 homing_start_window_ticks 之内才允许自动开始上升；
    ///    超过窗口必须用 `/gimbal/home` 显式触发。避免"待机时遥控关着（状态一直停在
    ///    IDLE）→ 回来一开遥控 → 机构立刻自己冲上去"。
    /// 2. **故障即失败**：上升期间出现硬件故障（CAN 反馈超时/过温）直接判失败并停转，
    ///    不再"等条件恢复后接着往上冲"；短暂失能（拨杆/遥控）中断超过
    ///    homing_interrupt_abort_ticks 也同样判失败。
    /// 3. 失败/未完成时上限退回 yaml 兜底值，并且调用方会对"向上"指令额外限速。
    ///
    /// @param permitted 本周期是否允许运动（模式为 FULL/SLOW，或台架模式下硬件正常）
    /// @param faulted 本周期是否处于硬件故障
    /// @param pitch_command 输出：本周期俯仰速度给定（电机轴 rad/s）
    /// @return true 表示本周期俯仰由 homing 接管（调用方忽略俯仰摇杆与上限软限位）
    auto update_homing(
        bool permitted, bool faulted, double pitch_angle, double left_velocity,
        double right_velocity, double left_torque, double right_torque, double& pitch_command)
        -> bool {

        if (!homing_enabled_ || homing_state_ == HomingState::kDone
            || homing_state_ == HomingState::kFailed)
            return false;

        ++homing_lifetime_ticks_;
        const bool within_start_window = homing_lifetime_ticks_ <= homing_start_window_ticks_;

        if (homing_state_ == HomingState::kIdle) {
            if (!(within_start_window || homing_requested_.exchange(false))) {
                if (!start_window_expired_logged_) {
                    start_window_expired_logged_ = true;
                    RCLCPP_WARN(
                        logger_,
                        "[launcher] 启动窗口（%d 周期）内没有条件自动找最高点，已停止自动上升。"
                        "需要时发一次 /gimbal/home（std_msgs/Int32 非 0）手动触发。",
                        homing_start_window_ticks_);
                }
                return true; // 接管但不动，等显式触发
            }
            if (!permitted)
                return true; // 条件不满足：接管但不动（避免一满足条件就被摇杆抢走）
            homing_state_ = HomingState::kRising;
            stall_ticks_ = 0;
            homing_ticks_ = 0;
            interrupt_ticks_ = 0;
            RCLCPP_INFO(
                logger_,
                "[launcher] 自动找最高点开始：以 %.1f rad/s(电机轴) 持续上升，堵转 %.1f rad/s / "
                "%.2f N·m 持续 %d 周期即认定到顶",
                homing_rise_velocity_, homing_stall_velocity_, homing_stall_torque_,
                homing_stall_confirm_ticks_);
        }

        if (faulted) {
            // 硬件故障：直接失败。这样即便随后通信恢复，也不会接着往上冲。
            abort_homing("出现硬件故障（CAN 反馈超时/过温）");
            pitch_command = 0.0;
            return true;
        }

        if (!permitted) {
            // 短暂失能（拨杆/遥控离线）：先暂停；超过时限则判失败
            pitch_command = 0.0;
            if (++interrupt_ticks_ >= homing_interrupt_abort_ticks_) {
                abort_homing("上升过程中被中断超时（拨杆失能/遥控离线）");
            }
            return true;
        }
        interrupt_ticks_ = 0;

        switch (homing_state_) {
        case HomingState::kRising: {
            ++homing_ticks_;
            pitch_command = homing_rise_velocity_;

            const bool left_stalled = std::abs(left_velocity) < homing_stall_velocity_
                                   && std::abs(left_torque) > homing_stall_torque_;
            const bool right_stalled = std::abs(right_velocity) < homing_stall_velocity_
                                    && std::abs(right_torque) > homing_stall_torque_;
            stall_ticks_ = (left_stalled && right_stalled) ? stall_ticks_ + 1 : 0;

            if (stall_ticks_ >= homing_stall_confirm_ticks_) {
                pitch_upper_limit_rad_ = pitch_angle;
                pitch_upper_limit_valid_ = true;
                if (homing_backoff_ticks_ > 0) {
                    homing_state_ = HomingState::kBackoff;
                    backoff_ticks_ = 0;
                } else {
                    homing_state_ = HomingState::kDone;
                    pitch_command = 0.0;
                }
                RCLCPP_INFO(
                    logger_,
                    "[launcher] 已找到最高点：俯仰角 %.4f rad（横梁高度 %.1f mm，对应两侧电机轴 "
                    "%.4f rad），已设为俯仰上限软限位",
                    pitch_upper_limit_rad_,
                    pitch_upper_limit_rad_ / std::max(pitch_rad_per_meter_, 1e-9) * 1000.0,
                    pitch_angle / std::max(pitch_rad_per_motor_turn_, 1e-9) * 2.0
                        * std::numbers::pi);
            } else if (homing_ticks_ >= homing_timeout_ticks_) {
                homing_state_ = HomingState::kFailed;
                pitch_command = 0.0;
                RCLCPP_ERROR(
                    logger_,
                    "[launcher] 自动找最高点超时（%d 周期）：俯仰停转，上限退回 yaml 兜底值 "
                    "%.4f rad。请检查丝杆是否卡死/电机是否上电/堵转阈值是否过严",
                    homing_timeout_ticks_, pitch_max_rad_);
            }
            break;
        }
        case HomingState::kBackoff: {
            ++backoff_ticks_;
            pitch_command = -std::abs(homing_backoff_velocity_);
            if (backoff_ticks_ >= homing_backoff_ticks_) {
                homing_state_ = HomingState::kDone;
                pitch_command = 0.0;
                RCLCPP_INFO(logger_, "[launcher] 回退结束，上限软限位 %.4f rad 生效", pitch_upper_limit_rad_);
            }
            break;
        }
        default: break;
        }

        return true;
    }

    /// @brief 中止自动找最高点：停转、把上限退回 yaml 兜底值，并明确停下（不自动续上）。
    ///
    /// @note 一旦中止就不会再自动开始（状态 kFailed），需要重新触发只能发 /gimbal/home。
    ///       这是为了"通信/信号异常恢复后不要再接着往上冲"。
    void abort_homing(const char* reason) {
        homing_state_ = HomingState::kFailed;
        pitch_upper_limit_valid_ = false;
        pitch_upper_limit_rad_ = pitch_max_rad_;
        RCLCPP_ERROR(
            logger_,
            "[launcher] 自动找最高点中止（%s）：俯仰停转，上限退回 yaml 兜底值 %.4f rad。"
            "需要重新找顶请发一次 /gimbal/home",
            reason, pitch_max_rad_);
    }

    /// GPIO 通道未启用（-1）时视为未触发。
    auto limit_is_active(int gpio_index, bool raw_high) const -> bool {
        if (gpio_index < 0)
            return false;
        return raw_high == limit_active_high_;
    }

    /// 把力矩命令转换成 CAN 槽位数据；故障时直接给 0 电流（NaN 之外的第二道保险）。
    auto command_of(device::DjiMotor& motor) const -> uint16_t {
        if (*fault_ >= 0.5)
            return device::CanPacket8::Quarter{0}.data;
        return motor.generate_command().data;
    }

    static auto finite_or(double value, double fallback) -> double {
        return std::isfinite(value) ? value : fallback;
    }

    // ---- 看门狗时间戳：回调在 librmcs 传输线程上写、executor 线程上读，必须原子 ----
    static auto now_ns() -> std::int64_t {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    /// @note 0 表示"本进程还没收到过"，steady_clock 原点在启动时刻，两者不会混淆。
    auto is_fresh(std::int64_t last_ns, std::int64_t now) const -> bool {
        return last_ns != 0 && (now - last_ns) < signal_timeout_ns_;
    }

    // ---- 板级回调：运行在 librmcs 传输线程上，只能做无锁 store ----

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
            [[unlikely]]
            return;
        if (can != Spec::kCans.kCan1)
            return;

        const auto now = now_ns();
        if (yaw_motor_.match_then_store_status(data.can_id, data.can_data))
            last_yaw_can_ns_.store(now, std::memory_order_relaxed);
        else if (pitch_left_motor_.match_then_store_status(data.can_id, data.can_data))
            last_pitch_left_can_ns_.store(now, std::memory_order_relaxed);
        else if (pitch_right_motor_.match_then_store_status(data.can_id, data.can_data))
            last_pitch_right_can_ns_.store(now, std::memory_order_relaxed);
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart != Spec::kUarts.kDbus)
            return;

        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        last_dbus_ns_.store(now_ns(), std::memory_order_relaxed);
    }

    void gpio_digital_read_result_callback(const Spec::Gpio& gpio, const View::GpioDigital& data)
        override {
        if (pitch_left_limit_gpio_ >= 0
            && gpio == Spec::kGpios[static_cast<std::size_t>(pitch_left_limit_gpio_)])
            pitch_left_limit_raw_.store(data.high, std::memory_order_relaxed);
        else if (
            pitch_right_limit_gpio_ >= 0
            && gpio == Spec::kGpios[static_cast<std::size_t>(pitch_right_limit_gpio_)])
            pitch_right_limit_raw_.store(data.high, std::memory_order_relaxed);
    }

    // ---- 零标定：msg->data = 0 全部 / 1 yaw / 2 pitch_left / 3 pitch_right ----
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr msg) {
        const auto calibrate = [this](device::DjiMotor& motor, const char* name) {
            const int zero_point = motor.calibrate_zero_point();
            RCLCPP_INFO(
                logger_, "[launcher] %s: new encoder_zero_point = %d（写入 gantry.yaml 后重启）",
                name, zero_point);
        };

        switch (msg->data) {
        case 0:
            calibrate(yaw_motor_, "yaw_motor");
            calibrate(pitch_left_motor_, "pitch_left_motor");
            calibrate(pitch_right_motor_, "pitch_right_motor");
            break;
        case 1: calibrate(yaw_motor_, "yaw_motor"); break;
        case 2: calibrate(pitch_left_motor_, "pitch_left_motor"); break;
        case 3: calibrate(pitch_right_motor_, "pitch_right_motor"); break;
        default:
            RCLCPP_WARN(
                logger_, "[launcher] /gimbal/calibrate 参数应为 0(全部)/1(yaw)/2(左)/3(右)");
            break;
        }
    }

    // ---- 手动触发/取消自动找最高点：data != 0 触发，data == 0 取消 ----
    void gimbal_home_subscription_callback(std_msgs::msg::Int32::UniquePtr msg) {
        if (msg->data == 0) {
            if (homing_state_ == HomingState::kRising || homing_state_ == HomingState::kBackoff)
                abort_homing("操作者取消（/gimbal/home 0）");
            else
                RCLCPP_INFO(logger_, "[launcher] /gimbal/home 0：当前没有在找最高点，忽略");
            return;
        }

        if (!homing_enabled_) {
            RCLCPP_WARN(logger_, "[launcher] homing_enabled=false，/gimbal/home 被忽略");
            return;
        }
        // 重新从 IDLE 开始（会覆盖已有的上限），并绕过启动窗口限制
        homing_state_ = HomingState::kIdle;
        homing_requested_.store(true, std::memory_order_relaxed);
        start_window_expired_logged_ = false;
        RCLCPP_WARN(
            logger_, "[launcher] 收到 /gimbal/home：将重新以 %.1f rad/s(电机轴) 上升找最高点",
            homing_rise_velocity_);
    }

    /// @brief 伴生组件：只负责"在三路 PID 算完之后"下发 CAN 帧。
    ///
    /// 拆成独立组件的原因与 gm6020 板相同：executor 按接口依赖做拓扑排序，
    /// 三台电机的 "/.../control_torque" 输入注册在本组件上，
    /// 使得本组件必然排在三路 PidController 之后、且在同一个 1 kHz 周期内完成下发。
    class LauncherGantryCommand : public rmcs_executor::Component {
    public:
        explicit LauncherGantryCommand(LauncherGantry& gantry)
            : gantry_(gantry) {}

        void update() override { gantry_.command_update(); }

    private:
        LauncherGantry& gantry_;
    };

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;
    std::shared_ptr<LauncherGantryCommand> command_;

    device::DjiMotor yaw_motor_;
    device::DjiMotor pitch_left_motor_;
    device::DjiMotor pitch_right_motor_;

    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;
    std::unique_ptr<librmcs::board::CBoard> board_;

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> gimbal_calibrate_subscription_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> gimbal_home_subscription_;

    double screw_lead_m_per_turn_ = 0.002;
    double pitch_rad_per_meter_ = 2.5;
    double pitch_rad_per_motor_turn_ = 0.005;
    double yaw_rad_per_motor_turn_ = 2.0 * std::numbers::pi;
    double pitch_zero_rad_ = 0.0;
    double yaw_zero_rad_ = 0.0;

    double pitch_max_rad_ = 0.6;          // yaml 兜底上限（homing 关闭或失败时使用）
    double pitch_upper_limit_rad_ = 0.6;  // 实际生效的上限（homing 成功后为自动测得值）
    bool pitch_upper_limit_valid_ = false; // 上限是否来自"找到的最高点"
    bool yaw_limit_enabled_ = false;
    double yaw_min_rad_ = -0.5, yaw_max_rad_ = 0.5;

    double stick_deadzone_ = 0.02;
    double pitch_axis_sign_ = 1.0, yaw_axis_sign_ = 1.0;
    double max_pitch_velocity_motor_ = 50.0, max_yaw_velocity_motor_ = 50.0;
    double slow_speed_scale_ = 0.35;

    bool homing_enabled_ = true;
    double homing_rise_velocity_ = 40.0;
    double homing_stall_velocity_ = 2.0;
    double homing_stall_torque_ = 0.4;
    int homing_stall_confirm_ticks_ = 100;
    int homing_timeout_ticks_ = 20000;
    bool homing_require_remote_ = true;
    int homing_backoff_ticks_ = 0;
    double homing_backoff_velocity_ = 20.0;
    int homing_start_window_ticks_ = 60000;
    int homing_interrupt_abort_ticks_ = 2000;
    double unhomed_up_speed_scale_ = 0.25;

    int pitch_left_limit_gpio_ = -1, pitch_right_limit_gpio_ = -1;
    bool limit_active_high_ = true;
    bool limit_pull_up_ = true;
    int limit_read_period_ms_ = 20;

    std::int64_t signal_timeout_ns_ = 100'000'000;
    double max_temperature_ = 70.0;

    OutputInterface<double> yaw_angle_rad_, pitch_angle_rad_;
    OutputInterface<double> height_left_m_, height_right_m_, skew_m_, skew_rad_;
    OutputInterface<double> pitch_upper_limit_output_;
    OutputInterface<double> pitch_left_limit_, pitch_right_limit_;
    OutputInterface<double> max_temperature_output_;
    OutputInterface<double> control_mode_, fault_, homing_state_output_;
    OutputInterface<double> pitch_stick_output_, yaw_stick_output_;
    OutputInterface<double> tx_failures_output_;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_control_velocity_;
    OutputInterface<double> pitch_left_control_velocity_, pitch_right_control_velocity_;

    std::atomic<std::int64_t> last_dbus_ns_{0};
    std::atomic<std::int64_t> last_yaw_can_ns_{0};
    std::atomic<std::int64_t> last_pitch_left_can_ns_{0};
    std::atomic<std::int64_t> last_pitch_right_can_ns_{0};
    std::atomic<bool> pitch_left_limit_raw_{false};
    std::atomic<bool> pitch_right_limit_raw_{false};
    std::atomic<bool> homing_requested_{false};

    Mode last_mode_ = Mode::kFault;
    HomingState homing_state_ = HomingState::kIdle;
    int stall_ticks_ = 0;
    int homing_ticks_ = 0;
    int backoff_ticks_ = 0;
    int homing_lifetime_ticks_ = 0;
    int interrupt_ticks_ = 0;
    bool start_window_expired_logged_ = false;
    std::int64_t tx_failures_ = 0;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::LauncherGantry, rmcs_executor::Component)
