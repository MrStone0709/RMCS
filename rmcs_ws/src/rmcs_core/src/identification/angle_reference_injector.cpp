#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace rmcs_core::controller::identification {

/// yaw **角度环**「目标角偏移」激励与录制组件。
///
/// 与 VelocityReferenceInjector 的分工
/// -----------------------------------
///   VelocityReferenceInjector : 替换**速度参考** -> 激励速度环（角度环被 reset 旁路）
///   本组件                    : 给**角误差叠加偏移** -> 激励角度环（速度环保持在线）
///
/// 本组件写 /gimbal/yaw/angle_command，gimbal_controller 在
/// `yaw_angle_command_input: true` 时把它加到求解器给出的角误差上：
///     error' = (目标角 + Δ) - 当前角
/// 即等效于把目标角阶跃 Δ，路径无不连续，激励的正是生产角度环。
///
/// 配置
/// ----
///     gimbal_controller:
///       ros__parameters:
///         yaw_control_enabled: true        # 必须保持在线
///         yaw_angle_command_input: true    # 打开角度偏移注入
///         yaw_velocity_command_input: false# 两个注入不要同时开
///
/// 触发方式（与其它辨识组件一致）
/// ------------------------------
///   左中 + 右中 -> 待机 ; 左中 + 右上 -> 触发 ; 左下 + 右下 -> 急停
///
/// 依赖注意
/// --------
/// 本组件**不能**读 gimbal_controller 的任何输出（如 /gimbal/yaw/control_angle_error），
/// 否则构成循环依赖 -> 执行器 FATAL。需要对照时用独立 ValueCollector。
class AngleReferenceInjector
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AngleReferenceInjector()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        get_parameter_or("csv_path", csv_path_, std::string{"/tmp/angle_command.csv"});
        get_parameter_or("hold_before_s", hold_before_, 2.0);
        get_parameter_or("hold_after_s", hold_after_, 1.0);
        // 结束时的斜坡淡出时长（s）。**必须有**：目标角偏移若从最终值阶跃回 0，
        // 云台会以极高速度回退该角度（实测 offset=0.60 时回退冲击严重，
        // kp=35 下速度参考可达 35*0.6=21 rad/s），即"失控高速旋转"。
        // 用线性淡出把回退速度限制在 offset/ramp_down_s 以内。
        get_parameter_or("ramp_down_s", ramp_down_s_, 2.0);
        get_parameter_or("arming_delay_s", arming_delay_, 2.0);

        get_parameter_or("step_enabled", step_enabled_, true);
        get_parameter_or("step_from", step_from_, 0.0);
        get_parameter_or("step_count", step_count_, 1);
        get_parameter_or("step_delta", step_delta_, 0.0);
        get_parameter_or("step_hold_s", step_hold_s_, 5.0);

        get_parameter_or("sweep_enabled", sweep_enabled_, false);
        get_parameter_or("sweep_start_freq", sweep_start_freq_, 0.5);
        get_parameter_or("sweep_end_freq", sweep_end_freq_, 10.0);
        get_parameter_or("sweep_duration_s", sweep_duration_, 30.0);
        get_parameter_or("sweep_amplitude", sweep_amplitude_, 0.2);
        get_parameter_or("sweep_logarithmic", sweep_logarithmic_, true);

        // 角度偏移量程（rad）。信号发生器自身的限幅，不是控制环限幅。
        get_parameter_or("max_command", max_command_, 1.5);

        if (!step_enabled_ && !sweep_enabled_)
            throw std::runtime_error("at least one of step_enabled / sweep_enabled must be true");
        if (!(max_command_ > 0.0))
            throw std::runtime_error("max_command must be > 0");
        if (!(hold_before_ >= 0.0) || !(hold_after_ >= 0.0))
            throw std::runtime_error("hold_before_s / hold_after_s must be >= 0");
        if (!(ramp_down_s_ > 0.0))
            throw std::runtime_error("ramp_down_s must be > 0");
        if (step_enabled_) {
            if (step_count_ < 1)
                throw std::runtime_error("step_count must be >= 1");
            if (!(step_hold_s_ > 0.0))
                throw std::runtime_error("step_hold_s must be > 0");
            if (step_delta_ == 0.0 && step_count_ == 1)
                step_delta_ = step_from_ == 0.0 ? 1.0 : step_from_; // 单级时兜底
        }
        if (sweep_enabled_) {
            if (!(sweep_start_freq_ > 0.0) || !(sweep_end_freq_ > 0.0))
                throw std::runtime_error("sweep frequencies must be > 0");
            if (!(sweep_duration_ > 0.0))
                throw std::runtime_error("sweep_duration_s must be > 0");
        }

        // ---- 接口注册（必须在构造函数体，见执行器注册规则说明）----------
        // 语义 = yaw **目标角偏移**（rad），叠加到求解器的目标角上。
        // 与旧的 "angle_command 速率注入" 相比：这里是精确角阶跃，且不依赖世界坐标系。
        register_output("/gimbal/yaw/angle_target_offset", angle_command_, kNaN);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/predefined/update_rate", update_rate_, false);
        register_input("/gimbal/yaw/angle", measured_angle_, false);
        register_input("/gimbal/yaw/velocity", measured_velocity_, false);
        register_input("/gimbal/yaw/velocity_imu", measured_velocity_imu_, false);
        register_input("/gimbal/yaw/torque", measured_torque_, false);
        register_input("/gimbal/yaw/temperature", measured_temperature_, false);
        register_input("/gimbal/yaw/max_torque", measured_max_torque_, false);
        // pitch 监测（kp 调高后需确认 yaw 未把 pitch 带偏）
        register_input("/gimbal/pitch/angle", measured_pitch_angle_, false);
        register_input("/gimbal/pitch/velocity_imu", measured_pitch_velocity_, false);
        register_input("/gimbal/pitch/torque", measured_pitch_torque_, false);

        csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open())
            throw std::runtime_error("failed to open csv: " + csv_path_);
        csv_file_ << std::setprecision(17);
        csv_file_ << "update_count,elapsed_s,angle_command,"
                  << "/gimbal/yaw/angle,"
                  << "/gimbal/yaw/velocity_imu,"
                  << "/gimbal/yaw/velocity,"
                  << "/gimbal/yaw/torque,"
                  << "/gimbal/yaw/temperature,"
                  << "/gimbal/yaw/max_torque,"
                  << "/gimbal/pitch/angle,"
                  << "/gimbal/pitch/velocity_imu,"
                  << "/gimbal/pitch/torque\n";

        RCLCPP_INFO(
            get_logger(),
            "angle reference injector ready: step=%d preload=%.4f then %d steps of delta=%+.4f "
            "every %.2f s; sweep=%d %.2f->%.2f Hz over %.1f s amp=%.4f; csv=%s",
            static_cast<int>(step_enabled_), step_from_, step_count_, step_delta_, step_hold_s_,
            static_cast<int>(sweep_enabled_), sweep_start_freq_, sweep_end_freq_, sweep_duration_,
            sweep_amplitude_, csv_path_.c_str());
    }

    auto before_updating() -> void override {
        *angle_command_ = kNaN;
        phase_ = Phase::kIdle;
        sample_index_ = 0;
        run_start_time_ = timestamp_now();
        last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    }

    auto update() -> void override {
        const auto dt = update_dt();
        const auto sw_l = switch_left_.ready() ? *switch_left_ : rmcs_msgs::Switch::UNKNOWN;
        const auto sw_r = switch_right_.ready() ? *switch_right_ : rmcs_msgs::Switch::UNKNOWN;
        handle_remote(sw_l, sw_r);

        switch (phase_) {
        case Phase::kIdle: *angle_command_ = kNaN; break;
        case Phase::kRunning: *angle_command_ = run_and_log(dt); break;
        case Phase::kAborted: *angle_command_ = kNaN; break;
        }
        last_switch_left_ = sw_l;
        last_switch_right_ = sw_r;
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
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

    static auto switch_name(rmcs_msgs::Switch v) -> const char* {
        switch (v) {
        case rmcs_msgs::Switch::UNKNOWN: return "UNKNOWN";
        case rmcs_msgs::Switch::UP: return "UP";
        case rmcs_msgs::Switch::DOWN: return "DOWN";
        case rmcs_msgs::Switch::MIDDLE: return "MIDDLE";
        }
        return "?";
    }

    auto phase_name() const -> const char* {
        switch (phase_) {
        case Phase::kIdle: return "idle";
        case Phase::kRunning: return "running";
        case Phase::kAborted: return "aborted";
        }
        return "?";
    }

    auto handle_remote(rmcs_msgs::Switch left, rmcs_msgs::Switch right) -> void {
        using rmcs_msgs::Switch;
        if (left != last_switch_left_ || right != last_switch_right_)
            RCLCPP_INFO(
                get_logger(), "remote switches: left=%s right=%s (phase=%s)", switch_name(left),
                switch_name(right), phase_name());

        if (left == Switch::DOWN && right == Switch::DOWN) {
            if (phase_ != Phase::kAborted && phase_ != Phase::kIdle)
                RCLCPP_WARN(get_logger(), "E-STOP: switches left+right DOWN");
            phase_ = Phase::kAborted;
            return;
        }

        const bool armed = last_switch_left_ == Switch::MIDDLE
                        && last_switch_right_ == Switch::MIDDLE;
        const bool fire = left == Switch::MIDDLE && right == Switch::UP;
        if (armed && fire && phase_ != Phase::kRunning) {
            const auto elapsed = timestamp_now() - run_start_time_;
            if (elapsed < arming_delay_) {
                RCLCPP_WARN(
                    get_logger(),
                    "ignoring start request (t=%.2f s): still within the %.2f s arming delay",
                    elapsed, arming_delay_);
                return;
            }
            start_run();
        }
    }

    auto start_run() -> void {
        t_ = 0.0;
        sample_index_ = 0;
        phase_ = Phase::kRunning;
        RCLCPP_INFO(
            get_logger(), "ANGLE INJECTION START: step=%d sweep=%d",
            static_cast<int>(step_enabled_), static_cast<int>(sweep_enabled_));
    }

    auto run_and_log(double dt) -> double {
        t_ += dt;

        double command = 0.0;
        if (t_ < hold_before_) {
            command = step_enabled_ ? step_from_ : 0.0;
        } else if (sweep_enabled_) {
            command = sweep_value(t_ - hold_before_);
        } else {
            const double tau = t_ - hold_before_;
            int level = static_cast<int>(tau / step_hold_s_);
            if (level > step_count_)
                level = step_count_;
            command = step_from_ + step_delta_ * static_cast<double>(level);
        }

        if (!std::isfinite(command))
            command = 0.0;
        command = std::clamp(command, -max_command_, max_command_);

        const double event_span =
            sweep_enabled_ ? sweep_duration_
                           : (step_enabled_ ? step_hold_s_ * step_count_ : hold_after_);
        const double event_end = hold_before_ + event_span;

        // 结束阶段：把 command 线性淡出到 0，避免目标角偏移阶跃回零造成回退冲击。
        const double ramp_begin = event_end - hold_after_;
        if (t_ >= ramp_begin) {
            const double progress = (t_ - ramp_begin) / ramp_down_s_;
            if (progress >= 1.0) {
                RCLCPP_INFO(
                    get_logger(), "ANGLE INJECTION FINISHED at t=%.4f s (ramp-down complete)",
                    t_);
                write_row(t_, 0.0);
                phase_ = Phase::kIdle;
                csv_file_.flush();
                return 0.0;
            }
            command *= (1.0 - progress); // 线性淡出
        }

        write_row(t_, command);
        return command;
    }

    auto sweep_value(double tau) const -> double {
        const double f0 = sweep_start_freq_, f1 = sweep_end_freq_, T = sweep_duration_;
        if (tau >= T)
            return 0.0;
        double phase = 0.0;
        if (sweep_logarithmic_) {
            const double r = f1 / f0;
            const double k = 2.0 * std::numbers::pi * f0 * T / std::log(r);
            phase = k * (std::pow(r, tau / T) - 1.0);
        } else {
            const double rate = (f1 - f0) / T;
            phase = 2.0 * std::numbers::pi * (f0 * tau + 0.5 * rate * tau * tau);
        }
        return sweep_amplitude_ * std::sin(phase);
    }

    auto write_row(double elapsed, double command) -> void {
        csv_file_ << sample_index_++ << "," << elapsed << "," << command << ","
                  << value_or_nan(measured_angle_) << "," << value_or_nan(measured_velocity_imu_)
                  << "," << value_or_nan(measured_velocity_) << ","
                  << value_or_nan(measured_torque_) << "," << value_or_nan(measured_temperature_)
                  << "," << value_or_nan(measured_max_torque_) << ","
                  << value_or_nan(measured_pitch_angle_) << ","
                  << value_or_nan(measured_pitch_velocity_) << ","
                  << value_or_nan(measured_pitch_torque_) << "\n";
        if (++flush_tick_ >= 100) {
            flush_tick_ = 0;
            csv_file_.flush();
        }
    }

    template <typename Interface>
    static auto value_or_nan(const Interface& i) -> double {
        if (!i.ready())
            return kNaN;
        return *i;
    }

    // ---- 参数 ----
    std::string csv_path_{"/tmp/angle_command.csv"};
    double hold_before_ = 2.0, hold_after_ = 1.0, arming_delay_ = 2.0;
    double ramp_down_s_ = 2.0; // 结束时的斜坡淡出时长（见构造函数说明）
    bool step_enabled_ = true;
    double step_from_ = 0.0, step_delta_ = 0.0, step_hold_s_ = 5.0;
    int step_count_ = 1;
    bool sweep_enabled_ = false;
    double sweep_start_freq_ = 0.5, sweep_end_freq_ = 10.0, sweep_duration_ = 30.0;
    double sweep_amplitude_ = 0.2;
    bool sweep_logarithmic_ = true;
    double max_command_ = 1.5;

    // ---- 状态 ----
    Phase phase_ = Phase::kIdle;
    double run_start_time_ = 0.0, t_ = 0.0;
    int sample_index_ = 0, flush_tick_ = 0;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    // ---- 接口 ----
    rmcs_executor::Component::OutputInterface<double> angle_command_;
    rmcs_executor::Component::InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    rmcs_executor::Component::InputInterface<double> update_rate_;
    rmcs_executor::Component::InputInterface<double> measured_angle_, measured_velocity_,
        measured_velocity_imu_, measured_torque_, measured_temperature_, measured_max_torque_;

    InputInterface<double> measured_pitch_angle_;
    InputInterface<double> measured_pitch_velocity_;
    InputInterface<double> measured_pitch_torque_;
    std::ofstream csv_file_;
};

} // namespace rmcs_core::controller::identification

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::AngleReferenceInjector, rmcs_executor::Component)
