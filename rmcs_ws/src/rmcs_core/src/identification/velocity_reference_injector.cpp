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

/// yaw 速度环「参考注入」激励与录制组件。
///
/// 为什么需要它
/// ------------
/// 生产配置下 yaw 速度环的参考由 gimbal_controller 的**角度环**产生，无法直接编程。
/// 要辨识/调优**速度环本身**，必须能在保持速度环结构不变的前提下，给一个可编程的
/// 速度参考。本组件就是那个信号源。
///
/// 配合的开关（gimbal_controller 侧，默认关闭，生产行为不受影响）
/// --------------------------------------------------------------
///     gimbal_controller:
///       ros__parameters:
///         yaw_velocity_command_input: true
/// 打开后，gimbal_controller 会在 /gimbal/yaw/control_velocity 可读且数值有限时，
/// 直接把它当作速度参考（角度环退出并 reset），从而暴露真正的速度环。
///
/// 重要：不要与 yaw_control_enabled 混淆
/// -------------------------------------
///   yaw_control_enabled: false   -> 本组件让出 yaw 的**全部输出**，gimbal_controller
///                                  不再参与控制（用于开环辨识）
///   yaw_velocity_command_input:  -> 保持 gimbal_controller 在线，只替换速度参考
///                                  这就是本组件要用的模式
/// 两者必须搭配正确，否则本组件写出的参考没人读、或与其它组件抢占符号而启动失败。
///
/// 本组件注册的输出
/// ----------------
///   /gimbal/yaw/control_velocity  (速度参考, rad/s)
///   注意：硬件层 LkMotor 也把同名符号注册为**输入**（用于其速度模式）。执行器允许
///   一个输出被多个输入绑定，所以不冲突。生产模式下电机走**力矩**通道
///   (control_angle 恒为 nan 时优先级落到 control_torque)，因此该速度值不会被电机
///   自己采信，只作为 gimbal_controller 的参考。
///
/// 触发方式（与既有辨识组件一致）
/// ------------------------------
///   左中 + 右中  ->  待机
///   左中 + 右上  ->  触发一次激励（上升沿，必须先经过中位）
///   左下 + 右下  ->  急停
class VelocityReferenceInjector
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    VelocityReferenceInjector()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        // 参数可在此安全读取：rclcpp::Node 基类已构造完毕。
        get_parameter_or("csv_path", csv_path_, std::string{"/tmp/velocity_reference.csv"});
        get_parameter_or("hold_before_s", hold_before_, 1.0);
        get_parameter_or("hold_after_s", hold_after_, 2.5);
        get_parameter_or("arming_delay_s", arming_delay_, 2.0);

        get_parameter_or("step_enabled", step_enabled_, true);
        get_parameter_or("step_from", step_from_, 0.0);        // 预载
        // 连续多级阶跃（staircase）。
        //   预载 step_from_ 持续 hold_before_ 秒，然后依次跳到
        //   step_from_+step_delta_*k (k=1..step_count_)，每级 step_hold_s_ 秒。
        //   step_delta_ 为负即为「先升后降」，可覆盖多个工作点。
        get_parameter_or("step_to", step_to_, 1.0);            // 兼容旧参数：单级时的目标
        get_parameter_or("step_count", step_count_, 1);
        get_parameter_or("step_delta", step_delta_, 0.0);      // 0 -> 用 step_to_ 作单级
        get_parameter_or("step_hold_s", step_hold_s_, 5.0);    // 每级持续（留给电机响应）

        get_parameter_or("sweep_enabled", sweep_enabled_, false);
        get_parameter_or("sweep_start_freq", sweep_start_freq_, 0.5);
        get_parameter_or("sweep_end_freq", sweep_end_freq_, 20.0);
        get_parameter_or("sweep_duration_s", sweep_duration_, 30.0);
        get_parameter_or("sweep_amplitude", sweep_amplitude_, 1.0);
        get_parameter_or("sweep_logarithmic", sweep_logarithmic_, true);

        // 参考幅值上限。这是**信号发生器自身的量程**，避免手滑写出巨大参考把速度环
        // 推爆；不是给控制环加限幅。
        get_parameter_or("max_reference", max_reference_, 20.0);

        if (!step_enabled_ && !sweep_enabled_)
            throw std::runtime_error("at least one of step_enabled / sweep_enabled must be true");
        if (step_enabled_) {
            if (step_count_ < 1)
                throw std::runtime_error("step_count must be >= 1");
            if (!(step_hold_s_ > 0.0))
                throw std::runtime_error("step_hold_s must be > 0");
            // step_delta 为 0 时退化为单级阶跃（沿用 step_to_）
            if (step_delta_ == 0.0 && step_count_ == 1)
                step_delta_ = step_to_ - step_from_;
        }
        if (!(max_reference_ > 0.0))
            throw std::runtime_error("max_reference must be > 0");
        if (!(hold_before_ >= 0.0) || !(hold_after_ >= 0.0))
            throw std::runtime_error("hold_before_s / hold_after_s must be >= 0");
        if (sweep_enabled_) {
            if (!(sweep_start_freq_ > 0.0) || !(sweep_end_freq_ > 0.0))
                throw std::runtime_error("sweep frequencies must be > 0");
            if (sweep_logarithmic_ && sweep_start_freq_ == sweep_end_freq_)
                throw std::runtime_error("logarithmic sweep requires start != end frequency");
            if (!(sweep_duration_ > 0.0))
                throw std::runtime_error("sweep_duration_s must be > 0");
        }

        // ---- 接口注册 -------------------------------------------------------
        // 【关键】必须在本构造函数体里注册，不能放到 before_pairing。
        // 执行器 init() 的顺序是：
        //   1) 遍历所有组件的 output_list_ / input_list_ 建立 output_map
        //   2) 调用各组件的 before_pairing(user_output_map)
        //   3) 遍历 input_list_ 做绑定（output_map.find）
        // output_map 在 before_pairing 之前就已定型，所以在 before_pairing 里
        // register_output 的输出【不会】进入 output_map；消费方的输入会 find 失败，
        // 而 required=false 的输入会被静默跳过（表现为 ready() 永远 false、无任何报错）。
        // （输入在 before_pairing 里注册没问题，因为绑定是第 3 步才做的 ——
        //   ValueCollector 就是这么用的。）
        register_output("/gimbal/yaw/velocity_command", control_velocity_, kNaN);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/predefined/update_rate", update_rate_, false);
        register_input("/gimbal/yaw/velocity", measured_velocity_, false);
        register_input("/gimbal/yaw/velocity_imu", measured_velocity_imu_, false);
        register_input("/gimbal/yaw/torque", measured_torque_, false);
        register_input("/gimbal/yaw/angle", measured_angle_, false);
        register_input("/gimbal/yaw/temperature", measured_temperature_, false);
        register_input("/gimbal/yaw/max_torque", measured_max_torque_, false);
        // 注意：这里【不能】读 /gimbal/yaw/control_torque 或 control_angle_error。
        // 本组件给 gimbal_controller 提供 /gimbal/yaw/velocity_command，若再去读
        // gimbal_controller 的输出就构成循环依赖，执行器会直接 FATAL 退出
        // （它把 required=false 的输入也算作依赖边）。
        // 需要对照记录这些量时，用独立的 ValueCollector 组件按名字订阅即可
        // —— 它只读不写，不参与依赖环。

        csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open())
            throw std::runtime_error("failed to open csv: " + csv_path_);
        csv_file_ << std::setprecision(17);
        csv_file_ << "update_count,elapsed_s,velocity_reference,"
                  << "/gimbal/yaw/velocity_imu,"
                  << "/gimbal/yaw/velocity,"
                  << "/gimbal/yaw/torque,"
                  << "/gimbal/yaw/angle,"
                  << "/gimbal/yaw/temperature,"
                  << "/gimbal/yaw/max_torque\n";

        RCLCPP_INFO(
            get_logger(),
            "velocity reference injector ready: step=%d preload=%.3f then %d steps of "
            "delta=%+.3f every %.2f s; sweep=%d %.2f->%.2f Hz over %.1f s; csv=%s",
            static_cast<int>(step_enabled_), step_from_, step_count_, step_delta_, step_hold_s_,
            static_cast<int>(sweep_enabled_), sweep_start_freq_, sweep_end_freq_, sweep_duration_,
            csv_path_.c_str());
    }

    auto before_updating() -> void override {
        *control_velocity_ = kNaN;
        phase_ = Phase::kIdle;
        sample_index_ = 0;
        // 每次（重）进入 update 循环重新建立时间基准，避免使用未初始化的时间。
        run_start_time_ = timestamp_now();
        last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    }

    auto update() -> void override {
        const auto dt = update_dt();

        const auto switch_left =
            switch_left_.ready() ? *switch_left_ : rmcs_msgs::Switch::UNKNOWN;
        const auto switch_right =
            switch_right_.ready() ? *switch_right_ : rmcs_msgs::Switch::UNKNOWN;

        handle_remote(switch_left, switch_right);

        switch (phase_) {
        case Phase::kIdle: *control_velocity_ = kNaN; break;
        case Phase::kRunning: *control_velocity_ = run_and_log(dt); break;
        case Phase::kAborted: *control_velocity_ = kNaN; break;
        }

        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;
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

    static auto switch_name(rmcs_msgs::Switch value) -> const char* {
        switch (value) {
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
            get_logger(), "INJECTION START: step=%d sweep=%d (hold %.2f s -> event -> %.2f s)",
            static_cast<int>(step_enabled_), static_cast<int>(sweep_enabled_), hold_before_,
            hold_after_);
    }

    /// 一个控制拍：算出参考值、限幅、落盘。
    auto run_and_log(double dt) -> double {
        t_ += dt;

        // 参考波形。sweep 优先；否则走「连续多级阶跃」。
        //   step_from_ 用作激励前的**预载**：让轴脱离静摩擦、进入速度环线性区
        //   （库仑摩擦会吃掉小信号响应，这是之前扫频实验的教训）。
        //   随后每 step_hold_s_ 秒跳一级，共 step_count_ 级。
        double reference = 0.0;
        if (t_ < hold_before_) {
            reference = step_enabled_ ? step_from_ : 0.0;
        } else if (sweep_enabled_) {
            reference = sweep_value(t_ - hold_before_);
        } else {
            const double tau = t_ - hold_before_;
            int level = static_cast<int>(tau / step_hold_s_);
            if (level > step_count_)
                level = step_count_;
            reference = step_from_ + step_delta_ * static_cast<double>(level);
        }

        if (!std::isfinite(reference))
            reference = 0.0;
        // 信号发生器自身量程（不是控制环限幅）
        reference = std::clamp(reference, -max_reference_, max_reference_);

        write_row(t_, reference);

        const double event_span =
            sweep_enabled_ ? sweep_duration_
                           : (step_enabled_ ? step_hold_s_ * step_count_ : hold_after_);
        if (t_ >= hold_before_ + event_span) {
            RCLCPP_INFO(get_logger(), "INJECTION FINISHED at t=%.4f s (idle)", t_);
            phase_ = Phase::kIdle;
            csv_file_.flush();
        }
        return reference;
    }

    /// 对数/线性扫频参考。phase(t) = 2*pi*f0*T/ln(r) * (r^(t/T) - 1)
    auto sweep_value(double tau) const -> double {
        const double f0 = sweep_start_freq_;
        const double f1 = sweep_end_freq_;
        const double T = sweep_duration_;
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

    auto write_row(double elapsed, double reference) -> void {
        csv_file_ << sample_index_++ << "," << elapsed << "," << reference << ","
                  << value_or_nan(measured_velocity_imu_) << ","
                  << value_or_nan(measured_velocity_) << ","
                  << value_or_nan(measured_torque_) << ","
                  << value_or_nan(measured_angle_) << ","
                  << value_or_nan(measured_temperature_) << ","
                  << value_or_nan(measured_max_torque_) << "\n";

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

    // ---- 参数 ---------------------------------------------------------------
    std::string csv_path_{"/tmp/velocity_reference.csv"};
    double hold_before_ = 1.0;
    double hold_after_ = 2.5;
    double arming_delay_ = 2.0;

    bool step_enabled_ = true;
    double step_from_ = 0.0;
    double step_to_ = 1.0;
    int step_count_ = 1;
    double step_delta_ = 0.0;
    double step_hold_s_ = 5.0;

    bool sweep_enabled_ = false;
    double sweep_start_freq_ = 0.5;
    double sweep_end_freq_ = 20.0;
    double sweep_duration_ = 30.0;
    double sweep_amplitude_ = 1.0;
    bool sweep_logarithmic_ = true;

    double max_reference_ = 20.0;

    // ---- 运行时状态 ---------------------------------------------------------
    Phase phase_ = Phase::kIdle;
    double run_start_time_ = 0.0;
    double t_ = 0.0;
    int sample_index_ = 0;
    int flush_tick_ = 0;

    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    // ---- 接口 ---------------------------------------------------------------
    rmcs_executor::Component::OutputInterface<double> control_velocity_;

    rmcs_executor::Component::InputInterface<rmcs_msgs::Switch> switch_left_;
    rmcs_executor::Component::InputInterface<rmcs_msgs::Switch> switch_right_;
    rmcs_executor::Component::InputInterface<double> update_rate_;

    rmcs_executor::Component::InputInterface<double> measured_velocity_;
    rmcs_executor::Component::InputInterface<double> measured_velocity_imu_;
    rmcs_executor::Component::InputInterface<double> measured_torque_;
    rmcs_executor::Component::InputInterface<double> measured_angle_;
    rmcs_executor::Component::InputInterface<double> measured_temperature_;
    rmcs_executor::Component::InputInterface<double> measured_max_torque_;

    std::ofstream csv_file_;
};

} // namespace rmcs_core::controller::identification

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::VelocityReferenceInjector,
    rmcs_executor::Component)
