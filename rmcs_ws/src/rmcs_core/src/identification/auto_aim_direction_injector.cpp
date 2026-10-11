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

#include <eigen3/Eigen/Dense>

namespace rmcs_core::controller::identification {

/// 通过**自瞄路径**注入 yaw/pitch **目标角阶跃**的激励与录制组件。
///
/// 为什么走自瞄路径
/// ----------------
/// gimbal_controller 有两条控制路径（见 deformable_infantry_gimbal_controller.cpp）：
///
///   手动：  gimbal_solver_.update(SetControlShift{yaw_shift, pitch_shift})
///           —— 速率式，目标角**以 shift 为速率累积（斜坡）**
///   自瞄：  gimbal_solver_.update(SetControlDirection{绝对方向})
///           —— **直接给定绝对目标方向**，方向跳变即目标角**阶跃**
///
/// 自瞄真实工况下目标角确实会出现锯齿/跳变，所以"阶跃"是有实际意义的测试工况。
/// 而 SetControlDirection 的输入 `/auto_aim/control_direction` 是**已有的公开接口**，
/// 因此本方案**完全不需要修改 gimbal_controller**（零侵入），只需新增本组件。
///
/// 方向向量的构造（关键）
/// ----------------------
/// 求解器算 yaw 误差：`yaw_angle_error = atan2(y, x)`（yaw_link 系）。
/// 所以要让目标 yaw 角为 ψ，方向应取
///
///     dir = ( cos ψ, sin ψ, 0 )
///
/// **z 分量取 0** 使目标 pitch 保持水平，从而 pitch 误差≈0 —— 这是刻意的：
/// 之前用 yaw_shift 持续旋转时把 pitch 甩偏、引发 pitch 锁点高频抖动，
/// 这里让 pitch 目标恒为水平可避免 pitch 参与。
///
/// 注意：`(cos ψ, 0, sin ψ)` 是**错的** —— 那会得到 yaw_error = atan2(0, x) = 0，
/// 完全不产生 yaw 指令。
///
/// 配置
/// ----
/// 需要 `switch_right = UP`（本组件即"自瞄请求"的触发位，与生产一致）；
/// **左开关保持中位**，避免 `update_pitch_lock_state()` 在 `左DOWN+右上` 时
/// 切换 suspension_on_by_switch_。
///
/// 触发：左中+右中 -> 待机；左中+右上 -> 触发；左下+右下 -> 急停
class AutoAimDirectionInjector
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AutoAimDirectionInjector()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        get_parameter_or("csv_path", csv_path_, std::string{"/tmp/autoaim_step.csv"});
        get_parameter_or("hold_before_s", hold_before_, 1.0);
        get_parameter_or("arming_delay_s", arming_delay_, 2.0);

        // 目标 yaw 角阶梯（绝对角，rad）：初始角从 /gimbal/yaw/angle 读取，
        // 之后按 step_delta 逐级累加。
        get_parameter_or("step_count", step_count_, 5);
        get_parameter_or("step_delta", step_delta_, 0.05);
        get_parameter_or("step_hold_s", step_hold_s_, 5.0);
        get_parameter_or("pitch_target", pitch_target_, 0.0); // 目标 pitch（rad，0=水平）

        // 目标角偏离实测角的允许上限（rad）。超过则中止，防止跑飞。
        get_parameter_or("angle_limit", angle_limit_, 1.0);
        // 误差持续不下降超过此时长（s）即中止，防止"静默卡死"
        get_parameter_or("nonconverge_timeout_s", nonconverge_timeout_, 4.0);
        // 触发自瞄后先等待 settle_s，让求解器把云台拉到"水平/当前方向"（这一步
        // 会有一次较大偏转），之后再开始阶梯，否则会被收敛保护误判。
        get_parameter_or("settle_s", settle_s_, 3.0);
        // 电机编码器角 -> 世界系 YawLink 角的固定偏移（rad），需实测标定。
        get_parameter_or("angle_offset_rad", angle_offset_rad_, 0.0);

        if (step_count_ < 1)
            throw std::runtime_error("step_count must be >= 1");
        if (!(step_hold_s_ > 0.0))
            throw std::runtime_error("step_hold_s must be > 0");
        if (!(angle_limit_ > 0.0))
            throw std::runtime_error("angle_limit must be > 0");
        if (!(hold_before_ >= 0.0))
            throw std::runtime_error("hold_before_s must be >= 0");

        // ---- 接口注册（必须在构造函数体，见执行器注册规则）------------------
        // 输出：自瞄路径的两个输入。写这个接口即"扮演自瞄"。
        register_output("/auto_aim/should_control", should_control_, false);
        register_output(
            "/auto_aim/control_direction", control_direction_,
            Eigen::Vector3d{kNaN, kNaN, kNaN});

        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/predefined/update_rate", update_rate_, false);
        register_input("/gimbal/yaw/angle", measured_yaw_, false);
        register_input("/gimbal/yaw/velocity_imu", measured_yaw_velocity_, false);
        register_input("/gimbal/yaw/torque", measured_yaw_torque_, false);
        // 同时记录 pitch，便于第一时间发现耦合
        register_input("/gimbal/pitch/angle", measured_pitch_, false);
        register_input("/gimbal/pitch/velocity_imu", measured_pitch_velocity_, false);
        register_input("/gimbal/pitch/torque", measured_pitch_torque_, false);
        register_input("/gimbal/yaw/temperature", measured_temperature_, false);

        csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open())
            throw std::runtime_error("failed to open csv: " + csv_path_);
        csv_file_ << std::setprecision(17);
        csv_file_ << "update_count,elapsed_s,target_yaw,dir_x,dir_y,"
                  << "/gimbal/yaw/angle,/gimbal/yaw/velocity_imu,/gimbal/yaw/torque,"
                  << "/gimbal/pitch/angle,/gimbal/pitch/velocity_imu,/gimbal/pitch/torque,"
                  << "/gimbal/yaw/temperature\n";

        RCLCPP_INFO(
            get_logger(),
            "auto-aim direction injector ready: %d steps of %.4f rad every %.1f s "
            "(pitch_target=%.4f, angle_limit=%.3f), csv=%s",
            step_count_, step_delta_, step_hold_s_, pitch_target_, angle_limit_, csv_path_.c_str());
    }

    auto before_updating() -> void override {
        *should_control_ = false;
        *control_direction_ = Eigen::Vector3d{kNaN, kNaN, kNaN};
        phase_ = Phase::kIdle;
        psi_ = kNaN;
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

        if (phase_ == Phase::kRunning) {
            run_and_log(dt);
        } else {
            // 非运行态：明确"不请求自瞄"，让 gimbal_controller 走原手动路径。
            *should_control_ = false;
            *control_direction_ = Eigen::Vector3d{kNaN, kNaN, kNaN};
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

    auto start_run() -> bool {
        // 起始 yaw 角：只读一次实测值作为基准，之后按 step_delta 累加，
        // 因此不依赖连续角度测量，也不涉及解缠。
        if (!measured_yaw_.ready() || !std::isfinite(*measured_yaw_)) {
            RCLCPP_WARN(get_logger(), "start rejected: /gimbal/yaw/angle not ready");
            return false;
        }
        psi_ = *measured_yaw_;
        target_yaw_ = psi_;
        cmd_yaw_ = psi_;
        latched_level_ = -1;
        best_abs_err_ = 1e9;
        nonconverge_time_ = 0.0;
        t_ = 0.0;
        sample_index_ = 0;
        phase_ = Phase::kRunning;
        RCLCPP_INFO(
            get_logger(),
            "AUTO-AIM STEP START: psi0=%.6f rad, %d steps of %+.4f rad (final %.4f rad), "
            "%.1f s per step",
            psi_, step_count_, step_delta_, psi_ + step_delta_ * step_count_, step_hold_s_);
        return true;
    }

    /// 计算本级的目标 yaw 角并写出方向向量。
    auto run_and_log(double dt) -> void {
        t_ += dt;

        if (!measured_yaw_.ready() || !std::isfinite(*measured_yaw_)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000, "measured yaw not ready -> holding");
            return;
        }
        const double measured = *measured_yaw_;

        // 阶段划分（时间驱动，各阶段起点/时长明确）：
        //   [0, settle_s_)                                -> 保持当前方向，等初始偏转完成
        //   [settle_s_, settle_s_+hold_before_)            -> 第 0 级基准（offset=0）
        //   之后每 step_hold_s_ 跳一级，共 step_count_ 级
        //
        // 注意 level 必须在**第一次建立基准时**就置 1，否则那一级会被漏掉
        // （旧实现用累计时间算 level，导致第 1 级与实测阶跃时刻错开约 0.5 s）。
        const double t_level1 = settle_s_ + hold_before_; // 第 1 级起始时刻
        int level = 1;
        if (t_ < t_level1) {
            level = 0;
        } else {
            level = 1 + static_cast<int>((t_ - t_level1) / step_hold_s_);
            if (level > step_count_)
                level = step_count_;
        }

        // 每次进入新的级数时锁存目标角 = (本组件自己维护的指令角 cmd_yaw_ + 级偏移)。
        //
        // 为什么基准要用"自己维护的指令角"而不是实测角：
        //   若用实测角（target = measured + Δ），云台每动一点、下一拍目标又前移 Δ，
        //   目标永远跑在云台前面 -> 变成持续单方向旋转（实测 0.4 s 内冲了 0.9 rad）。
        //   改成累加式指令角后，目标是**固定值**（阶跃），云台到位后误差归零即停住。
        //
        // 为什么基准要从【触发后的实测角】初始化，而不是从一开始就固定：
        //   求解器有 clamp_yaw_limit()（two_axis_gimbal_solver.hpp），它用
        //   /gimbal/yaw/angle 算 cw 并把目标钳在 [yaw_cw_min_, yaw_cw_max_] 内，
        //   而云台控制器**从未调用 enable_yaw_limit()** -> 限位是默认 [0, 0]。
        //   触发自瞄瞬间求解器会把云台拉到"水平/朝向"，这个初始偏转会改变
        //   gimbal_yaw_angle_。所以基准取 settle 之后的实测角，与求解器当前
        //   状态对齐，偏差就不会撞钳位。之后各级都是纯累加，目标有界。
        if (level != latched_level_) {
            if (latched_level_ < 0) {
                // 首次基准 = 实测角 + mu_。
                //
                // 为什么需要 mu_：
                //   求解器算的 yaw 误差用**世界系** YawLink 角 ψ_true，而
                //   /gimbal/yaw/angle 是**电机编码器角** ψ_meas，两者差一个固定量：
                //       ψ_true = mu + ψ_meas
                //   发方向 ψ_cmd 后误差 = ψ_cmd - ψ_true，角度环 kp=10 驱动云台，
                //   实测该闭环的系统增益为 2，稳定点满足 2·ψ_cmd - ψ_solver = mu。
                //   因此要零误差，目标须取 ψ_cmd = ψ_meas + mu。
                //   mu 由实测标定（见 angle_offset_rad 参数）。
                cmd_yaw_ = measured + angle_offset_rad_;
                best_abs_err_ = 1e9;
            } else if (level > latched_level_) {
                cmd_yaw_ += step_delta_ * static_cast<double>(level - latched_level_);
                best_abs_err_ = 1e9; // 新一级，重置收敛判据
            }
            latched_level_ = level;
            target_yaw_ = cmd_yaw_;
            RCLCPP_INFO(
                get_logger(),
                "step %d/%d: measured=%.6f target=%.6f (measured-target=%.6f, offset=%.4f)", level,
                step_count_, measured, target_yaw_, measured - target_yaw_, angle_offset_rad_);
        }

        // 安全保护 1：目标角与实测角偏离过大 -> 中止（位置保护）
        const double err = std::remainder(target_yaw_ - measured, 2 * std::numbers::pi);
        if (std::abs(err) > angle_limit_) {
            RCLCPP_ERROR(
                get_logger(),
                "ANGLE LIMIT VIOLATED: |target %.4f - measured %.4f| = %.4f > %.4f -> abort",
                target_yaw_, measured, std::abs(err), angle_limit_);
            abort_run();
            return;
        }

        // 安全保护 2：持续不收敛检测。
        // 判据用【本级内出现过的历史最优误差】：只要还在刷新更小的误差，就算在收敛。
        // 不能拿相邻两拍的瞬时误差比较 —— 误差过零时会被误判为"没下降"。
        const double abs_err = std::abs(err);
        if (abs_err < best_abs_err_) {
            best_abs_err_ = abs_err;
            nonconverge_time_ = 0.0;
        } else if (abs_err > angle_limit_ * 0.5) {
            nonconverge_time_ += dt;
        } else {
            nonconverge_time_ = 0.0;
        }
        if (nonconverge_time_ > nonconverge_timeout_) {
            RCLCPP_ERROR(
                get_logger(),
                "NOT CONVERGING: |err|=%.4f (best %.4f) no improvement for %.2f s -> abort",
                abs_err, best_abs_err_, nonconverge_time_);
            abort_run();
            return;
        }

        // 方向向量：yaw 误差 = atan2(y, x)。z 由目标 pitch 决定。
        //   dir = (cos ψ · cos θ, sin ψ · cos θ, sin θ)
        // θ=0（pitch_target_=0）时退化为 (cos ψ, sin ψ, 0)：目标 pitch 水平。
        const double cy = std::cos(target_yaw_), sy = std::sin(target_yaw_);
        const double cp = std::cos(pitch_target_), sp = std::sin(pitch_target_);
        *control_direction_ = Eigen::Vector3d{cy * cp, sy * cp, sp};
        *should_control_ = true;

        write_row(t_, target_yaw_);

        if (t_ >= settle_s_ + hold_before_ + step_hold_s_ * step_count_) {
            RCLCPP_INFO(get_logger(), "AUTO-AIM STEP FINISHED at t=%.4f s (idle)", t_);
            phase_ = Phase::kIdle;
            *should_control_ = false;
            csv_file_.flush();
        }
    }

    auto abort_run() -> void {
        phase_ = Phase::kAborted;
        *should_control_ = false;
        *control_direction_ = Eigen::Vector3d{kNaN, kNaN, kNaN};
        csv_file_.flush();
    }

    auto write_row(double elapsed, double target_yaw) -> void {
        const auto& d = *control_direction_;
        csv_file_ << sample_index_++ << "," << elapsed << "," << target_yaw << "," << d.x() << ","
                  << d.y() << "," << value_or_nan(measured_yaw_) << ","
                  << value_or_nan(measured_yaw_velocity_) << "," << value_or_nan(measured_yaw_torque_)
                  << "," << value_or_nan(measured_pitch_) << ","
                  << value_or_nan(measured_pitch_velocity_) << ","
                  << value_or_nan(measured_pitch_torque_) << ","
                  << value_or_nan(measured_temperature_) << "\n";
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
    std::string csv_path_{"/tmp/autoaim_step.csv"};
    double hold_before_ = 1.0, arming_delay_ = 2.0;
    int step_count_ = 5;
    double step_delta_ = 0.05, step_hold_s_ = 5.0;
    double pitch_target_ = 0.0;
    double angle_limit_ = 1.0;

    // ---- 状态 ----
    Phase phase_ = Phase::kIdle;
    double run_start_time_ = 0.0, t_ = 0.0;
    double psi_ = kNaN;      // 起始 yaw 角（仅用于日志）
    double target_yaw_ = kNaN; // 当前锁定目标角
    double cmd_yaw_ = kNaN;    // 本组件维护的累加式指令角（阶梯基准）
    int latched_level_ = -1;   // 已锁存的级数
    double best_abs_err_ = 1e9;
    double nonconverge_time_ = 0.0;
    double nonconverge_timeout_ = 4.0; // s: 误差持续不下降超过此时长则中止
    double settle_s_ = 3.0;            // s: 触发后等待初始偏转完成的时间
    double angle_offset_rad_ = 0.0;    // rad: 编码器角 -> 世界系角的固定偏移
    int sample_index_ = 0, flush_tick_ = 0;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    // ---- 接口 ----
    rmcs_executor::Component::OutputInterface<bool> should_control_;
    rmcs_executor::Component::OutputInterface<Eigen::Vector3d> control_direction_;
    rmcs_executor::Component::InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    rmcs_executor::Component::InputInterface<double> update_rate_;
    rmcs_executor::Component::InputInterface<double> measured_yaw_, measured_yaw_velocity_,
        measured_yaw_torque_, measured_pitch_, measured_pitch_velocity_, measured_pitch_torque_,
        measured_temperature_;

    std::ofstream csv_file_;
};

} // namespace rmcs_core::controller::identification

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::AutoAimDirectionInjector, rmcs_executor::Component)
