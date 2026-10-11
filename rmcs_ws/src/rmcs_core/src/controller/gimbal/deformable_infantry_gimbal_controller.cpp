#include "controller/gimbal/two_axis_gimbal_solver.hpp"
#include "controller/pid/pid_calculator.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

class DeformableInfantryGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableInfantryGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        configure_pid("yaw_angle", yaw_angle_pid_);
        configure_pid("yaw_velocity", yaw_velocity_pid_);
        configure_pid("pitch_angle", pitch_angle_pid_);
        configure_pid("pitch_velocity", pitch_velocity_pid_);

        get_parameter("pitch_torque_control", pitch_torque_control_enabled_);
        get_parameter("manual_joystick_sensitivity", joystick_sensitivity_);
        get_parameter("manual_mouse_sensitivity", mouse_sensitivity_);

        get_parameter_or("pitch_gravity_ff_gain", pitch_gravity_ff_gain_, 0.0);
        get_parameter_or("pitch_gravity_ff_phase", pitch_gravity_ff_phase_, 0.0);
        get_parameter_or("yaw_ref_velocity_gain", yaw_ref_velocity_gain_, 1.0);
        get_parameter_or("pitch_ref_velocity_gain", pitch_ref_velocity_gain_, 1.0);
        get_parameter_or("yaw_velocity_ff_gain", yaw_velocity_ff_gain_, 0.0);
        get_parameter_or("yaw_acceleration_ff_gain", yaw_acceleration_ff_gain_, 0.0);
        get_parameter_or("pitch_velocity_ff_gain", pitch_velocity_ff_gain_, 0.0);
        get_parameter_or("pitch_acceleration_ff_gain", pitch_acceleration_ff_gain_, 0.0);
        get_parameter_or("ctrl_hold_pitch_target_angle", ctrl_hold_pitch_target_angle_, 0.0);

        // 辨识/测试开关：对应轴为 false 时，本组件完全让出该轴的全部输出，
        // 交给辨识组件（SweptFrequencyController / StepResponseController）独占。
        // 必须在函数体里读，且只有在为 true 时才注册接口（见 Output 注释）。
        // 默认都是 true，生产行为完全不变。
        get_parameter_or("yaw_control_enabled", yaw_control_enabled_, true);
        get_parameter_or("pitch_control_enabled", pitch_control_enabled_, true);
        output_.register_outputs(*this, yaw_control_enabled_, pitch_control_enabled_);

        // 速度参考注入开关（默认 false = 生产行为完全不变）。
        //
        // 用途：系统辨识需要在**不改变速度环本身**的前提下，给 yaw 速度环一个
        // 可直接编程的参考值（阶跃/扫频），而不是靠摇杆或角度环产生。
        //   false -> yaw 速度参考仍由角度环产生（原行为）
        //   true  -> 若 /gimbal/yaw/control_velocity 有发布者且数值有限，
        //            则直接把它当速度参考；否则回退到角度环输出，不会失控。
        get_parameter_or("yaw_velocity_command_input", yaw_velocity_command_input_, false);

        // 角度环辨识开关（默认 false = 生产行为完全不变）。
        // true 时若 /gimbal/yaw/angle_command 有有限值，则把它作为**目标角偏移**
        // 加到求解器给出的角误差上，等效于阶跃目标角，从而激励角度环本身。
        get_parameter_or("yaw_angle_command_input", yaw_angle_command_input_, false);
    }


    /// 输入接口在 before_pairing 里注册：这是执行器规定"可注册/可配对"的钩子，
    /// 且它在本组件的 OutputInfoMap 可用之后、配对循环之前被调用。
    auto before_pairing(const OutputInfoMap& output_map) -> void override {
        (void)output_map;
        input_.register_velocity_command_input(*this);
        input_.register_angle_command_input(*this);
    }

    auto update() -> void override {
        // 首拍打印一次注入通道的绑定状态，便于现场确认"速度参考注入"是否真正接通。
        // （绑定发生在 before_pairing 之后，因此只能在 update 里判断。）
        if (!velocity_command_reported_) {
            velocity_command_reported_ = true;
            RCLCPP_INFO(
                get_logger(),
                "yaw injection: velocity_ref enabled=%d bound=%d | angle_offset enabled=%d bound=%d",
                static_cast<int>(yaw_velocity_command_input_),
                static_cast<int>(input_.yaw_velocity_command.ready()),
                static_cast<int>(yaw_angle_command_input_),
                static_cast<int>(input_.yaw_angle_command.ready()));
        }
        const auto switch_right = *input_.switch_right;
        const auto switch_left = *input_.switch_left;
        const auto keyboard = *input_.keyboard;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }

        update_pitch_lock_state(switch_left, switch_right, keyboard);

        if (ctrl_hold_requested()) {
            update_ctrl_hold_control();
        } else {
            deactivate_ctrl_hold();
        }

        const auto auto_aim_active = auto_aim_requested() && input_.auto_aim_should_control.ready()
                                  && *input_.auto_aim_should_control
                                  && input_.auto_aim_control_direction.ready()
                                  && input_.auto_aim_control_direction->allFinite()
                                  && !input_.auto_aim_control_direction->isZero();
        const auto angle_error =
            auto_aim_active ? update_auto_aim_control() : update_manual_control();

        // 扫频辨识用开关：false 时 yaw 支路整体让给
        // rmcs_core::controller::identification::SweptFrequencyController
        // （由它独占 /gimbal/yaw/control_torque），本组件只保留 pitch 支路。
        // 默认 true，生产行为完全不变。
        if (yaw_control_enabled_) {
            *output_.yaw_angle_error = angle_error.yaw_angle_error;
        } else {
            yaw_angle_pid_.reset();
            yaw_velocity_pid_.reset();
        }
        if (!ctrl_hold_active_ && pitch_control_enabled_)
            *output_.pitch_angle_error = angle_error.pitch_angle_error;

        const auto trajectory_ff = trajectory_feedforward(auto_aim_active);

        if (yaw_control_enabled_) {
            // 速度参考注入：只在开关打开、且外部确实给出了有限值时生效。
            // 构造上保证「不注入时行为与原来逐字相同」：
            //   注入关闭            -> 走 else 分支，与改动前完全一致；
            //   注入开启但无人发布  -> yaw_velocity_command.ready() 为 false，回退；
            //   注入开启但值为 nan  -> std::isfinite 为 false，回退。
            // 三种回退都不会让速度环失去参考而失控。
            const bool vc_ready = input_.yaw_velocity_command.ready();
            const bool vc_finite = vc_ready && std::isfinite(*input_.yaw_velocity_command);
            const bool inject = yaw_velocity_command_input_ && vc_ready && vc_finite;

            if (!std::isfinite(angle_error.yaw_angle_error)) {
                yaw_angle_pid_.reset();
                yaw_velocity_pid_.reset();
                *output_.yaw_control_torque = kNaN;
            } else if (inject) {
                // 注入模式：角度环退出，直接采用外部速度参考。
                // 必须 reset 角度 PID，否则它会在旁边持续积分、状态发散，
                // 一旦切回非注入模式就会把残余积分量猛地加到速度参考上。
                yaw_angle_pid_.reset();
                const auto yaw_velocity_ref = *input_.yaw_velocity_command;
                *output_.yaw_control_torque =
                    yaw_velocity_pid_.update(yaw_velocity_ref - *input_.yaw_velocity_imu)
                    + trajectory_ff.yaw_velocity + trajectory_ff.yaw_acceleration;
            } else {
                const auto yaw_velocity_ref = yaw_angle_pid_.update(angle_error.yaw_angle_error)
                                            + trajectory_ff.yaw_ref_velocity;
                *output_.yaw_control_torque =
                    yaw_velocity_pid_.update(yaw_velocity_ref - *input_.yaw_velocity_imu)
                    + trajectory_ff.yaw_velocity + trajectory_ff.yaw_acceleration;
            }
        }

        if (pitch_control_enabled_ && !ctrl_hold_active_) {
            if (!std::isfinite(angle_error.pitch_angle_error)) {
                pitch_angle_pid_.reset();
                pitch_velocity_pid_.reset();
                *output_.pitch_control_velocity = kNaN;
                *output_.pitch_control_torque = kNaN;
            } else {
                const auto pitch_gravity_ff = pitch_gravity_feedforward();
                const auto pitch_velocity_ref =
                    pitch_angle_pid_.update(angle_error.pitch_angle_error)
                    + trajectory_ff.pitch_ref_velocity;

                if (pitch_torque_control_enabled_) {
                    *output_.pitch_control_velocity = kNaN;
                    *output_.pitch_control_torque =
                        pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity_imu)
                        + pitch_gravity_ff + trajectory_ff.pitch_velocity
                        + trajectory_ff.pitch_acceleration;
                } else {
                    pitch_velocity_pid_.reset();
                    *output_.pitch_control_velocity = pitch_velocity_ref;
                    *output_.pitch_control_torque = kNaN;
                }
            }
        }
    }

private:
    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr auto kDefaultDt = 1e-3;

    auto configure_pid(const std::string& prefix, pid::PidCalculator& calculator) -> void {
        get_parameter(prefix + "_integral_min", calculator.integral_min);
        get_parameter(prefix + "_integral_max", calculator.integral_max);
        get_parameter(prefix + "_integral_split_min", calculator.integral_split_min);
        get_parameter(prefix + "_integral_split_max", calculator.integral_split_max);
        get_parameter(prefix + "_output_min", calculator.output_min);
        get_parameter(prefix + "_output_max", calculator.output_max);
    }

    struct Input {
        explicit Input(rmcs_executor::Component& component) {
            component.register_input("/remote/joystick/left", joystick_left);
            component.register_input("/remote/keyboard", keyboard);
            component.register_input("/remote/switch/right", switch_right);
            component.register_input("/remote/switch/left", switch_left);
            component.register_input("/remote/mouse/velocity", mouse_velocity);
            component.register_input("/remote/mouse", mouse);
            component.register_input("/predefined/update_rate", update_rate, false);

            component.register_input("/gimbal/pitch/angle", pitch_angle);
            component.register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu);
            component.register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu);

            component.register_input("/auto_aim/should_control", auto_aim_should_control, false);
            component.register_input(
                "/auto_aim/control_direction", auto_aim_control_direction, false);
            component.register_input("/auto_aim/ff_v", auto_aim_ff_v, false);
            component.register_input("/auto_aim/ff_a", auto_aim_ff_a, false);

        }

        /// 注册 yaw 速度参考注入输入。
        ///
        /// 为什么放在这里（成员函数）而不是 Input 构造里就注册：
        ///   与 Output 同样的坑 —— Input 是嵌套结构体，其构造函数在**外层构造函数
        ///   体之前**执行。在外层构造函数体里调用本方法注册，时机明确、与其它
        ///   组件一致，避免出现"注册了 active=1 但配对时找不到"的现象。
        void register_velocity_command_input(rmcs_executor::Component& component) {
            // 注册为【非必需】：没有发布者时不会阻塞配对，ready() 返回 false，
            // 本组件自动回退到角度环参考，行为与改动前一致。
            component.register_input("/gimbal/yaw/velocity_command", yaw_velocity_command, false);
        }

        /// 注册 yaw 角度偏移注入输入（角度环辨识用）。同样必须在构造函数体里调用。
        void register_angle_command_input(rmcs_executor::Component& component) {
            component.register_input("/gimbal/yaw/angle_command", yaw_angle_command, false);
        }

        /// yaw 速度参考注入接口，由外部注入组件写入。
        InputInterface<double> yaw_velocity_command;
        /// yaw 角度偏移注入接口（rad）。加到求解器给出的角误差上，
        /// 等效于把目标角阶跃一个偏移量，见 update() 中的说明。
        InputInterface<double> yaw_angle_command;

        InputInterface<Eigen::Vector2d> joystick_left;
        InputInterface<rmcs_msgs::Keyboard> keyboard;
        InputInterface<rmcs_msgs::Switch> switch_right;
        InputInterface<rmcs_msgs::Switch> switch_left;
        InputInterface<Eigen::Vector2d> mouse_velocity;
        InputInterface<rmcs_msgs::Mouse> mouse;
        InputInterface<double> update_rate;

        InputInterface<double> pitch_angle;
        InputInterface<double> yaw_velocity_imu;
        InputInterface<double> pitch_velocity_imu;

        InputInterface<bool> auto_aim_should_control;
        InputInterface<Eigen::Vector3d> auto_aim_control_direction;
        InputInterface<Eigen::Vector3d> auto_aim_ff_v;
        InputInterface<Eigen::Vector3d> auto_aim_ff_a;
    } input_{*this};

    struct Output {
        explicit Output(rmcs_executor::Component& component) {
            // 这里【不】注册任何接口：所有输出都按需在构造函数体里注册
            // （见 register_outputs）。原因见下方注释。
            (void)component;
        }

        /// 由构造函数体调用，按开关注册各轴接口。
        ///
        /// 为什么不在本结构体的初始化器里注册：
        ///   嵌套类的成员初始化器属于"完整类上下文"，会在外层构造函数体之前求值。
        ///   那时 yaw_control_enabled_ / pitch_control_enabled_ 尚未从参数读取，
        ///   只会读到未初始化值（实测垃圾值 51），导致开关完全失效、并触发
        ///   "Duplicate Normal output" 启动崩溃。
        void register_outputs(
            rmcs_executor::Component& component, bool yaw_enabled, bool pitch_enabled) {
            if (yaw_enabled) {
                component.register_output("/gimbal/yaw/control_torque", yaw_control_torque, kNaN);
                component.register_output("/gimbal/yaw/control_angle", yaw_control_angle, kNaN);
                component.register_output("/gimbal/yaw/control_angle_error", yaw_angle_error, kNaN);
            }
            if (pitch_enabled) {
                component.register_output(
                    "/gimbal/pitch/control_velocity", pitch_control_velocity, kNaN);
                component.register_output(
                    "/gimbal/pitch/control_torque", pitch_control_torque, kNaN);
                component.register_output("/gimbal/pitch/control_angle", pitch_control_angle, kNaN);
                component.register_output(
                    "/gimbal/pitch/control_angle_error", pitch_angle_error, kNaN);
            }
        }

        OutputInterface<double> yaw_control_torque;
        OutputInterface<double> yaw_control_angle;
        OutputInterface<double> pitch_control_velocity;
        OutputInterface<double> pitch_control_torque;
        OutputInterface<double> pitch_control_angle;
        OutputInterface<double> yaw_angle_error;
        OutputInterface<double> pitch_angle_error;
    } output_{*this};

    auto ctrl_hold_requested() const -> bool { return pitch_lock_active_; }

    auto update_dt() const -> double {
        if (input_.update_rate.ready() && std::isfinite(*input_.update_rate)
            && *input_.update_rate > 1e-6)
            return 1.0 / *input_.update_rate;
        return kDefaultDt;
    }

    auto manual_yaw_shift() const -> double {
        return joystick_sensitivity_ * input_.joystick_left->y()
             + mouse_sensitivity_ * input_.mouse_velocity->y() + injected_yaw_shift();
    }

    /// 角度环辨识用的 yaw 速率指令注入（单位 rad/s）。
    ///
    /// 为什么要注入 yaw_shift 而不是角误差：
    ///   求解器 TwoAxisGimbalSolver 是**速率式**的 —— SetControlShift 把当前控制
    ///   方向每拍旋转 yaw_shift_（见 two_axis_gimbal_solver.hpp:76-98），
    ///   即**目标角以 yaw_shift 的速率累积（斜坡）**，空闲时 yaw_shift=0 目标角冻结。
    ///   这是生产环境唯一存在的指令形式（摇杆/鼠标都走这条路）。
    ///
    ///   曾经尝试过"给角误差叠加偏移"来等效阶跃目标角 —— 那是错的：
    ///   求解器的误差是根据实际角度实时算的，云台一动误差就减小，最终
    ///   solver_err ≈ -offset、净误差≈0，只在极慢速率下改变平衡点，
    ///   几乎不产生可观测的响应（实测 solver_err=-0.2500 vs offs=+0.2500）。
    ///
    ///   恒定 yaw_shift = c  <->  目标角以 c 匀速旋转  <->  速度环的**速度阶跃**，
    ///   正是角度环在生产中要跟踪的工况。
    ///
    /// 三重回退：开关关闭 / 无人发布 / 值为 nan 时返回 0，行为与改动前逐字相同。
    auto injected_yaw_shift() const -> double {
        if (!yaw_angle_command_input_ || !input_.yaw_angle_command.ready()
            || !std::isfinite(*input_.yaw_angle_command))
            return 0.0;
        return *input_.yaw_angle_command;
    }

    auto auto_aim_requested() const -> bool {
        return input_.mouse->right || *input_.switch_right == rmcs_msgs::Switch::UP;
    }

    auto update_auto_aim_control() -> TwoAxisGimbalSolver::AngleError {
        return gimbal_solver_.update(
            TwoAxisGimbalSolver::SetControlDirection{
                OdomImu::DirectionVector{*input_.auto_aim_control_direction}});
    }

    auto update_manual_control() -> TwoAxisGimbalSolver::AngleError {
        if (!gimbal_solver_.enabled())
            return gimbal_solver_.update(TwoAxisGimbalSolver::SetToLevel{});

        const auto yaw_shift = manual_yaw_shift();

        const auto pitch_shift = -joystick_sensitivity_ * input_.joystick_left->x()
                               + mouse_sensitivity_ * input_.mouse_velocity->x();
        return gimbal_solver_.update(TwoAxisGimbalSolver::SetControlShift{yaw_shift, pitch_shift});
    }

    auto pitch_gravity_feedforward() const -> double {
        if (ctrl_hold_active_)
            return 0.0;
        return pitch_gravity_ff_gain_
             * std::sin(gimbal_solver_.gimbal_world_pitch() - pitch_gravity_ff_phase_);
    }

    struct TrajectoryFeedforward {
        double yaw_ref_velocity = 0.0;
        double pitch_ref_velocity = 0.0;
        double yaw_velocity = 0.0;
        double yaw_acceleration = 0.0;
        double pitch_velocity = 0.0;
        double pitch_acceleration = 0.0;
    };

    auto trajectory_feedforward(bool auto_aim_active) const -> TrajectoryFeedforward {
        if (!auto_aim_active || !input_.auto_aim_ff_v.ready() || !input_.auto_aim_ff_a.ready()
            || !input_.auto_aim_ff_v->allFinite() || !input_.auto_aim_ff_a->allFinite())
            return {};

        const auto ff_v =
            gimbal_solver_.odom_to_yaw_link(OdomImu::DirectionVector{*input_.auto_aim_ff_v});
        const auto ff_a =
            gimbal_solver_.odom_to_yaw_link(OdomImu::DirectionVector{*input_.auto_aim_ff_a});
        return {
            .yaw_ref_velocity = yaw_ref_velocity_gain_ * ff_v->z(),
            .pitch_ref_velocity = pitch_ref_velocity_gain_ * ff_v->y(),
            .yaw_velocity = yaw_velocity_ff_gain_ * ff_v->z(),
            .yaw_acceleration = yaw_acceleration_ff_gain_ * ff_a->z(),
            .pitch_velocity = pitch_velocity_ff_gain_ * ff_v->y(),
            .pitch_acceleration = pitch_acceleration_ff_gain_ * ff_a->y(),
        };
    }

    auto update_pitch_lock_state(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) -> void {
        if (switch_left == rmcs_msgs::Switch::DOWN && switch_right == rmcs_msgs::Switch::UP
            && last_switch_right_ == rmcs_msgs::Switch::MIDDLE) {
            suspension_on_by_switch_ = !suspension_on_by_switch_;
        }

        pitch_lock_active_ = keyboard.ctrl || keyboard.e || suspension_on_by_switch_;
        last_switch_right_ = switch_right;
    }

    auto activate_ctrl_hold() -> void {
        ctrl_hold_active_ = true;
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
    }

    auto deactivate_ctrl_hold() -> void {
        if (!ctrl_hold_active_)
            return;

        ctrl_hold_active_ = false;
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        if (pitch_control_enabled_)
            *output_.pitch_control_angle = kNaN;
    }

    auto update_ctrl_hold_control() -> void {
        if (!ctrl_hold_active_)
            activate_ctrl_hold();

        if (yaw_control_enabled_)
            *output_.yaw_control_angle = kNaN;
        if (pitch_control_enabled_) {
            *output_.pitch_control_velocity = kNaN;
            *output_.pitch_control_torque = kNaN;
            *output_.pitch_control_angle = kNaN;
        }

        if (input_.pitch_angle.ready() && std::isfinite(*input_.pitch_angle)) {
            auto pitch_target_error = ctrl_hold_pitch_target_angle_ - *input_.pitch_angle;
            if (pitch_target_error > std::numbers::pi)
                pitch_target_error -= 2 * std::numbers::pi;
            else if (pitch_target_error < -std::numbers::pi)
                pitch_target_error += 2 * std::numbers::pi;

            *output_.pitch_angle_error = pitch_target_error;
            const auto pitch_velocity_ref = pitch_angle_pid_.update(pitch_target_error);
            if (pitch_torque_control_enabled_) {
                *output_.pitch_control_velocity = kNaN;
                *output_.pitch_control_torque =
                    pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity_imu)
                    + pitch_gravity_feedforward();
            } else {
                pitch_velocity_pid_.reset();
                *output_.pitch_control_velocity = pitch_velocity_ref;
                *output_.pitch_control_torque = kNaN;
            }
        }
    }

    auto reset_control_outputs() -> void {
        yaw_angle_pid_.reset();
        yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        if (yaw_control_enabled_) {
            *output_.yaw_control_torque = kNaN;
            *output_.yaw_control_angle = kNaN;
        }
        if (pitch_control_enabled_) {
            *output_.pitch_control_velocity = kNaN;
            *output_.pitch_control_torque = kNaN;
            *output_.pitch_control_angle = kNaN;
        }
    }

    auto reset_all_controls() -> void {
        deactivate_ctrl_hold();
        pitch_lock_active_ = false;
        suspension_on_by_switch_ = false;
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
        if (yaw_control_enabled_)
            *output_.yaw_angle_error = kNaN;
        if (pitch_control_enabled_)
            *output_.pitch_angle_error = kNaN;
        reset_control_outputs();
    }

    TwoAxisGimbalSolver gimbal_solver_{
        *this, get_parameter("upper_limit").as_double(), get_parameter("lower_limit").as_double()};

    pid::PidCalculator yaw_angle_pid_{
        get_parameter("yaw_angle_kp").as_double(),
        get_parameter("yaw_angle_ki").as_double(),
        get_parameter("yaw_angle_kd").as_double(),
    };
    pid::PidCalculator yaw_velocity_pid_{
        get_parameter("yaw_velocity_kp").as_double(),
        get_parameter("yaw_velocity_ki").as_double(),
        get_parameter("yaw_velocity_kd").as_double(),
    };
    pid::PidCalculator pitch_angle_pid_{
        get_parameter("pitch_angle_kp").as_double(),
        get_parameter("pitch_angle_ki").as_double(),
        get_parameter("pitch_angle_kd").as_double(),
    };
    pid::PidCalculator pitch_velocity_pid_{
        get_parameter("pitch_velocity_kp").as_double(),
        get_parameter("pitch_velocity_ki").as_double(),
        get_parameter("pitch_velocity_kd").as_double(),
    };

    double joystick_sensitivity_ = 0.003;
    double mouse_sensitivity_ = 0.5;
    bool pitch_torque_control_enabled_ = false;
    double ctrl_hold_pitch_target_angle_ = 0.0;
    double pitch_gravity_ff_gain_ = 0.0;
    double pitch_gravity_ff_phase_ = 0.0;
    double yaw_ref_velocity_gain_ = 1.0;
    double pitch_ref_velocity_gain_ = 1.0;
    double yaw_velocity_ff_gain_ = 0.0;
    double yaw_acceleration_ff_gain_ = 0.0;
    double pitch_velocity_ff_gain_ = 0.0;
    double pitch_acceleration_ff_gain_ = 0.0;
    /// 辨识/测试开关。默认 true = 生产行为（本组件负责该轴）。
    /// false = 该轴全部输出让给辨识组件（SweptFrequencyController /
    /// StepResponseController），本组件不再注册、也不再写该轴任何接口。
    bool yaw_control_enabled_ = true;
    bool pitch_control_enabled_ = true;

    /// yaw 速度参考注入开关。默认 false：yaw 速度参考仍由角度环产生，生产行为不变。
    /// true 时若 /gimbal/yaw/control_velocity 有有限值，则直接用它作为速度参考，
    /// 用于在不改动速度环的前提下做阶跃/扫频辨识。详见构造函数注释。
    bool yaw_velocity_command_input_ = false;
    bool velocity_command_reported_ = false;

    /// yaw 角度偏移注入开关。默认 false：角度环输入完全来自求解器，生产行为不变。
    bool yaw_angle_command_input_ = false;

    bool pitch_lock_active_ = false;
    bool suspension_on_by_switch_ = false;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    bool ctrl_hold_active_ = false;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::DeformableInfantryGimbalController, rmcs_executor::Component)
