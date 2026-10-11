#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace rmcs_core::controller::identification {

/// 云台单轴「角度阶跃响应」激励与录制组件。
///
/// 设计目的
/// --------
/// 原始工程的 DeformableInfantryGimbalController 只输出**增量式**指令
/// (SetControlShift)，摇杆打满产生的是角度**斜坡**而非阶跃，无法用它做阶跃辨识。
/// 而 LkMotor 的命令优先级是:
///     control_angle_shift > control_angle > control_velocity > control_torque
/// 所以只要向 /gimbal/<axis>/control_angle 写入一个**非 nan** 值，电机就会切到
/// **角度模式**，云台控制器写的力矩被自动忽略，无需修改其控制逻辑。
///
/// 因此本组件:
///   1. 独占 /gimbal/<axis>/control_angle，输出一个绝对目标角阶跃；
///   2. 同时把该轴的全部可观测量写进 CSV，便于离线分析。
///
/// 依赖配置
/// --------
/// 必须先把云台控制器对应轴的让出开关置 false，否则两个组件会注册同名输出而
/// 启动崩溃 (Duplicate Normal output):
///     gimbal_controller:
///       ros__parameters:
///         yaw_control_enabled: false      # 让出 yaw
///         pitch_control_enabled: false    # 让出 pitch
///
/// 取数接口一览
/// ------------
///   本组件输出         : <prefix>/control_angle            (目标角，rad)
///   硬件层实测 (LkMotor): <prefix>/angle                    (实际角，rad)
///                        <prefix>/velocity                 (实际速度，rad/s)
///                        <prefix>/torque                   (实际力矩，N*m)
///                        <prefix>/temperature              (温度，摄氏度)
///                        <prefix>/max_torque               (额定力矩，N*m)
///   IMU                : <prefix>/velocity_imu             (陀螺仪角速度，rad/s)
///   云台控制器输出      : <prefix>/control_torque           (其算出的力矩，角度模式下被忽略)
///                        <prefix>/control_angle_error      (其算出的角误差)
///
/// 注意
/// ----
/// 角度模式下电机内部自带位置环，因此本实验测到的是**电机内置位置环**的响应，
/// 不是 gimbal_controller 位置环的响应。分析时必须区分。
class StepResponseController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    StepResponseController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {}

    auto before_pairing(const OutputInfoMap& output_map) -> void override {
        (void)output_map; // 本组件不依赖其它组件输出，只看参数

        // ---- 目标轴 ----------------------------------------------------------
        std::string axis;
        get_parameter_or("axis", axis, std::string{"yaw"});
        if (axis != "yaw" && axis != "pitch")
            throw std::runtime_error("axis must be either \"yaw\" or \"pitch\"");
        prefix_ = "/gimbal/" + axis;

        // ---- 阶跃参数 --------------------------------------------------------
        get_parameter_or("angle_initial", angle_initial_param_, kNaN);
        get_parameter_or("angle_final", angle_final_, 0.0);
        get_parameter_or("hold_before_s", hold_before_, 1.0);
        get_parameter_or("hold_after_s", hold_after_, 2.0);
        get_parameter_or("arming_delay_s", arming_delay_, 2.0);

        // 行程保护。默认给一个宽松值，防止实验期间跑飞。
        // 这是【位置】保护，不是力矩限幅 —— 与云台控制器自带的角度限位同性质。
        get_parameter_or("angle_limit", angle_limit_, 1.5);

        // 预留：角度模式的速度上限（输出轴 rad/s）。
        //
        // LkMotor 的角度命令 (0xA3/0xA4) 确实支持该字段，但【当前无法从组件侧下发】:
        //   硬件把 /gimbal/<axis>/control_velocity 绑定为角度命令的第二参数
        //   (deformable-infantry-omni-c.cpp 中 gimbal_pitch_motor_.generate_angle_command()
        //    只传一个参数，取默认 kNan)，而本组件不能注册 control_velocity
        //   —— 它已由硬件注册为输入，重复注册会启动崩溃。
        //
        // 因此这里只做参数校验并记录，实际值暂不下发（限位由电机内部角度环保证）。
        // 若将来需要，可在硬件层把 control_velocity 作为角度命令的 velocity_limit 透传。
        get_parameter_or("velocity_limit", velocity_limit_, kNaN);
        if (!std::isnan(velocity_limit_) && !(velocity_limit_ > 0.0))
            throw std::runtime_error("velocity_limit must be > 0 (or omit it / set nan)");
        if (!std::isnan(velocity_limit_))
            RCLCPP_WARN(
                get_logger(),
                "velocity_limit=%.3f was given but is NOT applied: the hardware angle-command "
                "path does not forward it. It is validated only.",
                velocity_limit_);

        get_parameter_or("csv_path", csv_path_, std::string{"/tmp/step_response.csv"});

        if (!std::isfinite(angle_final_))
            throw std::runtime_error("angle_final must be finite");
        if (!(hold_before_ >= 0.0) || !(hold_after_ >= 0.0))
            throw std::runtime_error("hold_before_s / hold_after_s must be >= 0");
        if (!(angle_limit_ > 0.0))
            throw std::runtime_error("angle_limit must be > 0");

        // ---- 接口注册（必须在 before_pairing 里）----------------------------
        // 输出：目标角。写非 nan 即让电机进入角度模式。
        register_output(prefix_ + "/control_angle", control_angle_, kNaN);

        // 实测与参考信号，全部设为非必需，避免个别信号缺失导致启动失败。
        register_input(prefix_ + "/angle", measured_angle_, false);
        register_input(prefix_ + "/velocity", measured_velocity_, false);
        register_input(prefix_ + "/torque", measured_torque_, false);
        register_input(prefix_ + "/temperature", measured_temperature_, false);
        register_input(prefix_ + "/max_torque", measured_max_torque_, false);
        register_input(prefix_ + "/velocity_imu", measured_velocity_imu_, false);

        // 云台控制器该轴的输出（角度模式下电机不采信，仅作对比记录）。
        register_input(prefix_ + "/control_torque", ctrl_control_torque_, false);
        register_input(prefix_ + "/control_angle_error", ctrl_angle_error_, false);

        // 触发用遥控器开关
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);

        // 时间基准
        register_input("/predefined/update_rate", update_rate_, false);

        // ---- CSV -------------------------------------------------------------
        csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open())
            throw std::runtime_error("failed to open csv: " + csv_path_);
        // 全精度，避免分析时精度损失
        csv_file_ << std::setprecision(17);
        csv_file_ << "update_count,elapsed_s,target_angle,error,"
                  << prefix_ << "/angle,"
                  << prefix_ << "/velocity,"
                  << prefix_ << "/velocity_imu,"
                  << prefix_ << "/torque,"
                  << prefix_ << "/temperature,"
                  << prefix_ << "/max_torque,"
                  << prefix_ << "/control_torque,"
                  << prefix_ << "/control_angle_error\n";

        RCLCPP_INFO(
            get_logger(), "step response on %s: %.4f -> %.4f rad, hold %.2f s before / %.2f s after, csv=%s",
            prefix_.c_str(), angle_initial_, angle_final_, hold_before_, hold_after_,
            csv_path_.c_str());
    }

    auto before_updating() -> void override {
        // 每次启动/复位重新开始，避免上一轮状态残留
        *control_angle_ = kNaN;
        phase_ = Phase::kIdle;
        angle_initial_ = angle_initial_param_; // nan 时每次触发重新取当期实测角
        step_issued_ = false;
        sample_index_ = 0;
        last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    }

    auto update() -> void override {
        const auto dt = update_dt();

        // ---- 时间基准必须在最前面建立 ---------------------------------------
        // 曾经踩过的坑：如果把计时基准放在遥控器处理之后，第一拍读到的
        // elapsed 是默认值 0，会把唯一的那个上升沿消耗在 arming delay 里。
        if (run_start_time_ < 0.0)
            run_start_time_ = timestamp_now();

        const auto switch_left =
            switch_left_.ready() ? *switch_left_ : rmcs_msgs::Switch::UNKNOWN;
        const auto switch_right =
            switch_right_.ready() ? *switch_right_ : rmcs_msgs::Switch::UNKNOWN;

        handle_remote(switch_left, switch_right);

        switch (phase_) {
        case Phase::kIdle: *control_angle_ = kNaN; break;
        case Phase::kRunning: *control_angle_ = run_and_log(dt); break;
        case Phase::kAborted:
            *control_angle_ = kNaN;
            break;
        }

        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr int kAngleViolationSamples = 10;
    static constexpr auto kDefaultDt = 1e-3;

    enum class Phase { kIdle, kRunning, kAborted };

    static auto timestamp_now() -> double {
        return static_cast<double>(
                   std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::steady_clock::now().time_since_epoch())
                       .count())
             * 1e-9;
    }

    auto update_dt() const -> double {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return kDefaultDt;
    }

    static auto wrap_to_pi(double angle) -> double {
        return std::remainder(angle, 2 * std::numbers::pi);
    }

    /// rmcs_msgs::Switch 是普通 enum class，没有 to_string，这里自己转。
    static auto switch_name(rmcs_msgs::Switch value) -> const char* {
        switch (value) {
        case rmcs_msgs::Switch::UNKNOWN: return "UNKNOWN";
        case rmcs_msgs::Switch::UP: return "UP";
        case rmcs_msgs::Switch::DOWN: return "DOWN";
        case rmcs_msgs::Switch::MIDDLE: return "MIDDLE";
        }
        return "?";
    }

    /// 遥控器状态机。
    ///   左中 + 右中  -> 待机
    ///   左中 + 右上  -> 起振（上升沿触发一次阶跃）
    ///   左下 + 右下  -> 急停
    auto handle_remote(rmcs_msgs::Switch left, rmcs_msgs::Switch right) -> void {
        using rmcs_msgs::Switch;

        if (left != last_switch_left_ || right != last_switch_right_)
            RCLCPP_INFO(
                get_logger(), "remote switches: left=%s right=%s (phase=%s)",
                switch_name(left), switch_name(right), phase_name());

        // 急停
        if (left == Switch::DOWN && right == Switch::DOWN) {
            if (phase_ != Phase::kAborted && phase_ != Phase::kIdle)
                RCLCPP_WARN(get_logger(), "E-STOP: switches left+right DOWN");
            phase_ = Phase::kAborted;
            return;
        }

        // 起振条件：上升沿 左中+右上
        const bool armed = last_switch_left_ == Switch::MIDDLE
                        && last_switch_right_ == Switch::MIDDLE;
        const bool fire = left == Switch::MIDDLE && right == Switch::UP;
        if (armed && fire && phase_ != Phase::kRunning) {
            const auto elapsed_since_boot = timestamp_now() - run_start_time_;
            if (elapsed_since_boot < arming_delay_) {
                RCLCPP_WARN(
                    get_logger(), "ignoring start request (t=%.2f s): still within the %.2f s arming delay",
                    elapsed_since_boot, arming_delay_);
                return;
            }
            start_run();
        }
    }

    auto start_run() -> void {
        // 起点：nan 时取触发瞬间的实测角，保证阶跃一定从当前姿态出发
        if (std::isnan(angle_initial_)) {
            if (measured_angle_.ready() && std::isfinite(*measured_angle_)) {
                angle_initial_ = wrap_to_pi(*measured_angle_);
                RCLCPP_INFO(
                    get_logger(), "initial angle latched from measurement: %.6f rad",
                    angle_initial_);
            } else {
                RCLCPP_WARN(
                    get_logger(), "angle_initial is auto but %s/angle not ready; start rejected",
                    prefix_.c_str());
                return;
            }
        }

        center_angle_ = angle_initial_;
        violation_samples_ = 0;
        step_issued_ = false;
        t_ = 0.0;
        phase_ = Phase::kRunning;
        RCLCPP_INFO(
            get_logger(),
            "STEP START: %.6f -> %.6f rad (delta %.6f rad = %.2f deg), hold %.2f s -> step -> %.2f s",
            angle_initial_, angle_final_, angle_final_ - angle_initial_,
            (angle_final_ - angle_initial_) * 180.0 / std::numbers::pi, hold_before_,
            hold_after_);
    }

    /// 一个控制拍：推进一步、产生目标角、落盘一行。
    auto run_and_log(double dt) -> double {
        t_ += dt;

        const double target = (t_ < hold_before_) ? angle_initial_ : angle_final_;
        if (!step_issued_ && t_ >= hold_before_) {
            step_issued_ = true;
            RCLCPP_INFO(
                get_logger(), "STEP ISSUED at t=%.4f s: target %.6f rad", t_, target);
        }

        // ---- 行程保护（位置保护，非力矩限幅）--------------------------------
        if (measured_angle_.ready() && std::isfinite(*measured_angle_)) {
            // 实测角是 [0,2pi) 单圈值，先去掉 2pi 整数倍再比较
            const auto reference = target;
            const auto wrapped = reference + wrap_to_pi(*measured_angle_ - reference);
            if (std::abs(wrapped - center_angle_) > angle_limit_) {
                if (++violation_samples_ >= kAngleViolationSamples) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "ANGLE LIMIT VIOLATED: measured %.4f rad deviates from center %.4f "
                        "by more than %.4f rad -> aborting",
                        wrapped, center_angle_, angle_limit_);
                    phase_ = Phase::kAborted;
                    return kNaN;
                }
            } else {
                violation_samples_ = 0;
            }
        }

        write_row(t_, target, dt);

        if (t_ >= hold_before_ + hold_after_) {
            RCLCPP_INFO(get_logger(), "STEP RUN FINISHED at t=%.4f s (idle)", t_);
            phase_ = Phase::kIdle;
            csv_file_.flush();
        }
        return target;
    }

    auto write_row(double elapsed, double target, double /*dt*/) -> void {
        const auto measured = measured_angle_.ready() ? *measured_angle_ : kNaN;

        csv_file_ << sample_index_++ << "," << elapsed << "," << target << ","
                  << wrap_to_pi(target - measured) << "," << measured << ","
                  << value_or_nan(measured_velocity_) << ","
                  << value_or_nan(measured_velocity_imu_) << ","
                  << value_or_nan(measured_torque_) << ","
                  << value_or_nan(measured_temperature_) << ","
                  << value_or_nan(measured_max_torque_) << ","
                  << value_or_nan(ctrl_control_torque_) << ","
                  << value_or_nan(ctrl_angle_error_) << "\n";

        if (++flush_tick_ >= 100) {
            flush_tick_ = 0;
            csv_file_.flush();
        }
    }

    template <typename Interface>
    static auto value_or_nan(const Interface& interface) -> double {
        if (!interface.ready())
            return kNaN;
        return *interface;
    }

    auto phase_name() const -> const char* {
        switch (phase_) {
        case Phase::kIdle: return "idle";
        case Phase::kRunning: return "running";
        case Phase::kAborted: return "aborted";
        }
        return "?";
    }

    // ---- 参数 ---------------------------------------------------------------
    std::string prefix_{"/gimbal/yaw"};
    std::string csv_path_{"/tmp/step_response.csv"};
    /// 参数给出的起点角。nan 表示"每次触发时取当期实测角"。
    double angle_initial_param_ = kNaN;
    /// 本次运行的起点角（可能是从实测角锁存来的）。
    double angle_initial_ = kNaN;
    double angle_final_ = 0.0;
    double hold_before_ = 1.0;
    double hold_after_ = 2.0;
    double arming_delay_ = 2.0;
    double angle_limit_ = 1.5;
    double velocity_limit_ = kNaN;

    // ---- 运行时状态 ---------------------------------------------------------
    Phase phase_ = Phase::kIdle;
    double run_start_time_ = -1.0;
    double t_ = 0.0;
    double center_angle_ = 0.0;
    bool step_issued_ = false;
    int violation_samples_ = 0;
    int sample_index_ = 0;
    int flush_tick_ = 0;

    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    // ---- 接口 ---------------------------------------------------------------
    rmcs_executor::Component::OutputInterface<double> control_angle_;

    rmcs_executor::Component::InputInterface<double> measured_angle_;
    rmcs_executor::Component::InputInterface<double> measured_velocity_;
    rmcs_executor::Component::InputInterface<double> measured_torque_;
    rmcs_executor::Component::InputInterface<double> measured_temperature_;
    rmcs_executor::Component::InputInterface<double> measured_max_torque_;
    rmcs_executor::Component::InputInterface<double> measured_velocity_imu_;
    rmcs_executor::Component::InputInterface<double> ctrl_control_torque_;
    rmcs_executor::Component::InputInterface<double> ctrl_angle_error_;

    rmcs_executor::Component::InputInterface<rmcs_msgs::Switch> switch_left_;
    rmcs_executor::Component::InputInterface<rmcs_msgs::Switch> switch_right_;
    rmcs_executor::Component::InputInterface<double> update_rate_;

    std::ofstream csv_file_;
};

} // namespace rmcs_core::controller::identification

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::StepResponseController,
    rmcs_executor::Component)
