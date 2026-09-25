#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
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

#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"
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

/// @brief 单电机控制板：CBoard 板卡 + CAN1 上的 GM6020 + DR16 遥控器。
///
/// 实现拆成两个组件：
/// - Gm6020ControlBoard（本类）：板级 IO。刷新电机反馈、对反馈做低通滤波、维护失联看门狗，
///   并把控制帧发到 CAN1。
/// - Gm6020Command（伴生组件，见下）：控制律。把遥控器右摇杆的竖直轴（前推为正）映射成
///   速度给定或角度给定，用「单环速度」或「双环角度」PID 算出力矩。
///
/// 之所以拆成两个组件：RemoteControl 已经把 "/remote/*" 注册成本组件的输出，而同一个组件内
/// 不允许注册同名输入（见 rmcs_executor::Component::ensure_registration_name_is_available）。
/// 拆开之后，executor 会按接口依赖做拓扑排序，自动保证「先采样 -> 再控制 -> 后下发」，
/// 与 OmniInfantry + InfantryCommand 的做法一致。
///
/// 控制方式（右拨杆三档，左拨杆 DOWN 为总禁用）：
/// - 右拨杆 UP     ：双环角度。角度 PID 输出速度给定，速度 PID 输出力矩。
/// - 右拨杆 MIDDLE ：单环速度。摇杆直接给速度给定。
/// - 右拨杆 DOWN   ：失能。
class Gm6020ControlBoard
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    Gm6020ControlBoard()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , command_(
              create_partner_component<Gm6020Command>(get_component_name() + "_command", *this))
        , motor_(*this, *command_, "/motor")
        , velocity_filter_(1.0) {

        // ---- 电机：GM6020，直驱（减速比 1），开多圈角度供角度环使用 ----
        const auto motor_id = static_cast<std::uint8_t>(get_param_or(*this, "motor_id", 1));
        if (motor_id < 1 || motor_id > 7)
            throw std::runtime_error("[gm6020] motor_id must be in [1, 7]");

        auto config =
            device::DjiMotor::Config{device::DjiMotor::Type::kGM6020, motor_id}
                .set_encoder_zero_point(get_param_or(*this, "encoder_zero_point", 0))
                .enable_multi_turn_angle();
        if (get_param_or(*this, "motor_reversed", false))
            config.set_reversed();
        motor_.configure(config);

        // ---- 状态输出。DjiMotor 已注册 /motor/{angle,velocity,torque,max_torque} ----
        // 诊断量统一用 double：broadcaster::ValueBroadcaster 只能转发 double 输出，
        // 用 bool 的话在 Foxglove 里看不到这些保护状态。
        register_output("/motor/velocity_filtered", velocity_filtered_output_, 0.0);
        register_output("/motor/temperature", temperature_output_, 0.0);
        register_output("/motor/remote_valid", remote_valid_output_, 0.0);
        register_output("/motor/feedback_valid", feedback_valid_output_, 0.0);
        register_output("/motor/over_temperature", over_temperature_output_, 0.0);
        register_input("/predefined/update_rate", update_rate_);

        // ---- 遥控：DBUS 帧解析由 device::Dr16 完成，RemoteControl 统一对外接口 ----
        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        signal_timeout_ns_ = static_cast<std::int64_t>(
            get_param_or(*this, "signal_timeout_ms", 100.0) * 1'000'000.0);
        max_temperature_ = get_param_or(*this, "max_temperature", 70.0);
        filter_cutoff_hz_ = get_param_or(*this, "velocity_filter_cutoff_hz", 80.0);

        // ---- 板卡：CBoard 在构造期间就可能开始收发回调，所以放在构造函数最后 ----
        auto options = librmcs::board::AdvancedOptions{};
        options.dangerously_skip_version_checks = get_param_or(*this, "skip_version_check", false);
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_param_or(*this, "board_serial", std::string{}), options);

        RCLCPP_INFO(
            logger_, "[gm6020] board ready: CAN1, motor_id=%u, reversed=%d",
            static_cast<unsigned>(motor_.id()), static_cast<int>(config.reversed));
    }

    Gm6020ControlBoard(const Gm6020ControlBoard&) = delete;
    Gm6020ControlBoard& operator=(const Gm6020ControlBoard&) = delete;
    Gm6020ControlBoard(Gm6020ControlBoard&&) = delete;
    Gm6020ControlBoard& operator=(Gm6020ControlBoard&&) = delete;
    ~Gm6020ControlBoard() override = default;

    /// @brief 反馈刷新：遥控 -> 电机反馈 -> 滤波 -> 看门狗。由 executor 周期调用。
    void update() override {
        dr16_.update_status();
        remote_control_->update();
        motor_.update_status();

        if (!configured_)
            configure_once();

        const auto now = now_ns();
        const bool remote_fresh = is_fresh(last_dbus_ns_.load(std::memory_order_relaxed), now);
        const bool feedback_fresh = is_fresh(last_can_ns_.load(std::memory_order_relaxed), now);

        if (feedback_fresh) {
            *velocity_filtered_output_ = velocity_filter_.update(motor_.velocity());
            *temperature_output_ = motor_.temperature();
            *over_temperature_output_ = motor_.temperature() >= max_temperature_ ? 1.0 : 0.0;
        } else {
            // 还没收到过电机反馈时，DjiMotor 的解析结果来自未初始化的原子缓冲，
            // 必须丢弃并复位滤波器，否则脏数据会被 IIR 带到后续若干个周期。
            velocity_filter_.reset();
            *velocity_filtered_output_ = 0.0;
            *temperature_output_ = 0.0;
            *over_temperature_output_ = 0.0;
        }

        *remote_valid_output_ = remote_fresh ? 1.0 : 0.0;
        *feedback_valid_output_ = feedback_fresh ? 1.0 : 0.0;
    }

private:
    /// @note 必须在 update() 里做，不能在 before_updating() 里做：
    /// executor 是在所有 before_updating() 之后才调用 set_update_rate()，
    /// 那时 "/predefined/update_rate" 还是 0，会把滤波器 alpha 算成 NaN。
    void configure_once() {
        configured_ = true;

        const double sampling_hz = *update_rate_;
        if (!std::isfinite(sampling_hz) || sampling_hz <= 0.0) {
            RCLCPP_WARN(
                logger_, "[gm6020] invalid update_rate %.3f, velocity filter bypassed",
                sampling_hz);
            velocity_filter_.set_alpha(1.0);
            return;
        }

        velocity_filter_.set_cutoff(filter_cutoff_hz_, sampling_hz);
        RCLCPP_INFO(
            logger_, "[gm6020] velocity low-pass filter: fc=%.1f Hz, fs=%.1f Hz", filter_cutoff_hz_,
            sampling_hz);
    }

    // 看门狗时间戳在 librmcs 的传输线程上写入、在 executor 线程上读取，必须原子。
    static auto now_ns() -> std::int64_t {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    /// @note 0 表示「本进程还没收到过」，steady_clock 的原点是启动时刻，两者不会混淆。
    auto is_fresh(std::int64_t last_ns, std::int64_t now) const -> bool {
        return last_ns != 0 && (now - last_ns) < signal_timeout_ns_;
    }

    // ---- 板级回调运行在 librmcs 的传输线程上，只能做无锁的 store ----

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        if (can != Spec::kCans.kCan1)
            return;
        if (!motor_.match_then_store_status(data.can_id, data.can_data))
            return;

        last_can_ns_.store(now_ns(), std::memory_order_relaxed);
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart != Spec::kUarts.kDbus)
            return;

        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        last_dbus_ns_.store(now_ns(), std::memory_order_relaxed);
    }

    /// @brief 把力矩指令打包成 CAN1 上的 GM6020 控制帧并下发。
    /// @note GM6020 的帧内槽位是 (id - 1) % 4，id 1~4 用 0x1FE，id 5~7 用 0x2FE。
    void transmit_torque(double torque) {
        device::CanPacket8 packet{uint64_t{0}};
        packet.data[(motor_.id() - 1) % 4] = motor_.generate_command(torque).data;

        board_->start_transmit().can_transmit(
            Spec::kCans.kCan1, //
            {.can_id = motor_.send_id(), .can_data = packet.as_bytes()});
    }

    /// @brief 控制律：摇杆映射 + 单环速度 / 双环角度 PID + CAN1 下发。
    class Gm6020Command
        : public rmcs_executor::Component
        , public rclcpp::Node {
    public:
        explicit Gm6020Command(Gm6020ControlBoard& board)
            : rclcpp::Node{
                  get_component_name(),
                  rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
            , board_(board)
            , velocity_pid_(controller::pid::make_pid_calculator(*this, "velocity_", 0.6, 0.0, 0.0))
            , angle_pid_(controller::pid::make_pid_calculator(*this, "angle_", 10.0, 0.0, 0.0))
            , deadzone_(get_param_or(*this, "deadzone", 0.05))
            , max_velocity_(get_param_or(*this, "max_velocity", 20.0))
            , max_angle_rate_(get_param_or(*this, "max_angle_rate", 6.0))
            , angle_min_(get_param_or(*this, "angle_min", -3.0))
            , angle_max_(get_param_or(*this, "angle_max", 3.0)) {

            // DjiMotor 已把 "/motor/control_torque" 注册为本组件的输入（标准外部接管点），
            // 内部 PID 走 generate_command(torque) 的值重载，两者互不干扰。

            register_input("/remote/joystick/right", joystick_right_);
            register_input("/remote/switch/right", switch_right_);
            register_input("/remote/switch/left", switch_left_);

            register_input("/motor/angle", motor_angle_);
            register_input("/motor/velocity_filtered", motor_velocity_filtered_);
            register_input("/motor/max_torque", motor_max_torque_);
            register_input("/motor/remote_valid", remote_valid_);
            register_input("/motor/feedback_valid", feedback_valid_);
            register_input("/motor/over_temperature", over_temperature_);
            register_input("/predefined/update_rate", update_rate_);

            register_output("/motor/control_mode", control_mode_output_, 0.0);
            register_output("/motor/angle_setpoint", angle_setpoint_output_, nan_);
            register_output("/motor/velocity_setpoint", velocity_setpoint_output_, nan_);
            register_output("/motor/applied_torque", applied_torque_output_, 0.0);
        }

        void before_updating() override {
            RCLCPP_INFO(
                get_logger(),
                "[gm6020] velocity pid kp=%.4f ki=%.4f kd=%.4f | angle pid kp=%.4f ki=%.4f "
                "kd=%.4f | max_velocity=%.2f rad/s, max_angle_rate=%.2f rad/s",
                velocity_pid_.kp, velocity_pid_.ki, velocity_pid_.kd, angle_pid_.kp, angle_pid_.ki,
                angle_pid_.kd, max_velocity_, max_angle_rate_);
        }

        void update() override {
            const auto mode = decode_mode();

            // 模式切换做无扰切换：清掉积分，并把角度给定重置为当前角度，避免猛冲
            if (mode != last_mode_) {
                velocity_pid_.reset();
                angle_pid_.reset();
                if (mode == Mode::kAngle)
                    angle_setpoint_ = *motor_angle_;
                last_mode_ = mode;
            }

            double velocity_setpoint = nan_;
            double torque = 0.0;

            switch (mode) {
            case Mode::kVelocity:
                // 单环：速度 PID，摇杆直接给速度给定
                velocity_setpoint = max_velocity_ * stick();
                torque = velocity_pid_.update(velocity_setpoint - *motor_velocity_filtered_);
                break;
            case Mode::kAngle:
                // 双环：角度 PID 输出速度给定，速度 PID 输出力矩。
                // 角度给定用增量式：摇杆回中即保持当前角度
                angle_setpoint_ += max_angle_rate_ * stick() * dt();
                angle_setpoint_ = std::clamp(angle_setpoint_, angle_min_, angle_max_);
                velocity_setpoint = angle_pid_.update(angle_setpoint_ - *motor_angle_);
                torque = velocity_pid_.update(velocity_setpoint - *motor_velocity_filtered_);
                break;
            case Mode::kDisabled: break;
            }

            if (!std::isfinite(torque))
                torque = 0.0;
            torque = std::clamp(torque, -*motor_max_torque_, *motor_max_torque_);

            *control_mode_output_ = static_cast<double>(static_cast<int>(mode));
            *angle_setpoint_output_ = (mode == Mode::kAngle) ? angle_setpoint_ : nan_;
            *velocity_setpoint_output_ = velocity_setpoint;
            *applied_torque_output_ = torque;

            board_.transmit_torque(torque);
        }

    private:
        enum class Mode : int {
            kDisabled = 0,
            kVelocity = 1,
            kAngle = 2,
        };

        auto decode_mode() const -> Mode {
            using rmcs_msgs::Switch;

            // 电机反馈超时或过温时无条件失能
            if (*feedback_valid_ < 0.5 || *over_temperature_ >= 0.5)
                return Mode::kDisabled;

            // 遥控失联时失能。
            // 注意：RemoteControl 在未注册 VT13 时会关闭 DR16 的超时失效
            // （见 device/remote_control.hpp 的 update_timeout_interlock），
            // 所以这里必须依赖 Gm6020ControlBoard 自己的 DBUS 看门狗。
            if (*remote_valid_ < 0.5)
                return Mode::kDisabled;

            if (*switch_left_ == Switch::DOWN)
                return Mode::kDisabled;

            switch (*switch_right_) {
            case Switch::UP: return Mode::kAngle;
            case Switch::MIDDLE: return Mode::kVelocity;
            default: return Mode::kDisabled;
            }
        }

        /// @brief 右摇杆的竖直轴（前推为正），带死区并重新归一化到 [-1, 1]。
        ///
        /// @note 这里必须取 .x()，不能取 .y()。
        /// DBUS 的通道顺序是 ch0=右摇杆水平、ch1=右摇杆竖直，而 device/dr16.hpp 把 ch1 放进了
        /// .x()、把 ch0 放进了 .y()，所以 Eigen 向量的 .x() 才是前后轴。这一点由
        /// chassis_controller.cpp 的 `*joystick_right_ + {w - s, a - d}` 佐证：W 键（前进）
        /// 贡献 +x，说明 .x() 与前向同号。取错轴的现象就是「拨杆能切模式、摇杆完全没反应」。
        auto stick() const -> double {
            const double axis = joystick_right_->x();
            if (!std::isfinite(axis) || std::abs(axis) <= deadzone_)
                return 0.0;
            return std::copysign((std::abs(axis) - deadzone_) / (1.0 - deadzone_), axis);
        }

        /// @brief 控制周期。角度给定按增量式累加，需要它把 rad/s 换算成 rad/tick。
        auto dt() const -> double {
            const double rate = *update_rate_;
            return (std::isfinite(rate) && rate > 0.0) ? 1.0 / rate : 0.001;
        }

        static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

        Gm6020ControlBoard& board_;

        controller::pid::PidCalculator velocity_pid_, angle_pid_;

        const double deadzone_;
        const double max_velocity_;
        const double max_angle_rate_;
        const double angle_min_, angle_max_;

        rmcs_executor::Component::InputInterface<Eigen::Vector2d> joystick_right_;
        rmcs_executor::Component::InputInterface<rmcs_msgs::Switch> switch_right_, switch_left_;

        rmcs_executor::Component::InputInterface<double> motor_angle_;
        rmcs_executor::Component::InputInterface<double> motor_velocity_filtered_;
        rmcs_executor::Component::InputInterface<double> motor_max_torque_;
        rmcs_executor::Component::InputInterface<double> remote_valid_, feedback_valid_,
            over_temperature_;
        rmcs_executor::Component::InputInterface<double> update_rate_;

        rmcs_executor::Component::OutputInterface<double> control_mode_output_;
        rmcs_executor::Component::OutputInterface<double> angle_setpoint_output_,
            velocity_setpoint_output_, applied_torque_output_;

        Mode last_mode_ = Mode::kDisabled;
        double angle_setpoint_ = 0.0;
    };

    rclcpp::Logger logger_;

    std::shared_ptr<Gm6020Command> command_;

    device::DjiMotor motor_;
    filter::LowPassFilter<1> velocity_filter_;

    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;
    std::unique_ptr<librmcs::board::CBoard> board_;

    rmcs_executor::Component::InputInterface<double> update_rate_;

    rmcs_executor::Component::OutputInterface<double> velocity_filtered_output_;
    rmcs_executor::Component::OutputInterface<double> temperature_output_;
    rmcs_executor::Component::OutputInterface<double> remote_valid_output_;
    rmcs_executor::Component::OutputInterface<double> feedback_valid_output_;
    rmcs_executor::Component::OutputInterface<double> over_temperature_output_;

    std::atomic<std::int64_t> last_dbus_ns_{0};
    std::atomic<std::int64_t> last_can_ns_{0};
    std::int64_t signal_timeout_ns_ = 100'000'000;
    bool configured_ = false;
    double max_temperature_ = 70.0;
    double filter_cutoff_hz_ = 80.0;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Gm6020ControlBoard, rmcs_executor::Component)
