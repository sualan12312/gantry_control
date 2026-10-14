#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gantry {

class PitchController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PitchController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/gantry/pitch/left/motor/velocity", left_velocity_);
        register_input("/gantry/pitch/right/motor/velocity", right_velocity_);

        register_output("/gantry/pitch/left/motor/control_torque", left_torque_, kNan);
        register_output("/gantry/pitch/right/motor/control_torque", right_torque_, kNan);

        get_parameter_or("pitch_sensitivity", pitch_sensitivity_, 3.0);
        get_parameter_or("pitch_max_velocity", pitch_max_velocity_, 3.0);
        get_parameter_or("joystick_deadzone", joystick_deadzone_, 0.02);
        get_parameter_or("sync_coefficient", sync_coefficient_, 0.0);
        get_parameter_or("feedback_source", feedback_source_, std::string{"average"});

        get_parameter_or("test_velocity", test_velocity_, 0.0);
        get_parameter_or("test_kp", test_kp_, 0.5);
        get_parameter_or("test_torque_limit", test_torque_limit_, 0.6);

        get_parameter_or("kp", pid_.kp, 1.0);
        get_parameter_or("ki", pid_.ki, 0.0);
        get_parameter_or("kd", pid_.kd, 0.0);
        get_parameter_or("output_max", pid_.output_max, 1.2);
        get_parameter_or("output_min", pid_.output_min, -1.2);
        pid_.reset();
    }

    void update() override {
        using namespace rmcs_msgs;

        const auto sw_left = *switch_left_;
        const auto sw_right = *switch_right_;

        if (sw_left == Switch::UNKNOWN || sw_right == Switch::UNKNOWN
            || (sw_left == Switch::DOWN && sw_right == Switch::DOWN)) {
            *left_torque_ = kNan;
            *right_torque_ = kNan;
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "[Pitch] disabled: L=%d R=%d | left_vel=%+.2f right_vel=%+.2f",
                static_cast<int>(sw_left), static_cast<int>(sw_right), *left_velocity_,
                *right_velocity_);
            return;
        }

        if (test_velocity_ != 0.0) {
            *left_torque_ = kNan;
            const double error = test_velocity_ - *right_velocity_;
            *right_torque_ =
                std::clamp(test_kp_ * error, -test_torque_limit_, test_torque_limit_);
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "[Pitch] TEST right motor: target=%+.2f right_vel=%+.2f right_torque=%+.2f Nm",
                test_velocity_, *right_velocity_, *right_torque_);
            return;
        }

        double target = pitch_sensitivity_ * deadzone(joystick_left_->x());
        target = std::clamp(target, -pitch_max_velocity_, pitch_max_velocity_);

        double feedback = 0.5 * (*left_velocity_ + *right_velocity_);
        if (feedback_source_ == "left")
            feedback = *left_velocity_;
        else if (feedback_source_ == "right")
            feedback = *right_velocity_;

        const double torque = pid_.update(target - feedback);

        const double correction = sync_coefficient_ * (*left_velocity_ - *right_velocity_);

        *left_torque_ = std::clamp(torque - correction, -kMaxTorque, kMaxTorque);
        *right_torque_ = std::clamp(torque + correction, -kMaxTorque, kMaxTorque);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[Pitch] stick=(%+.3f,%+.3f) target=%+.2f | left_vel=%+.2f right_vel=%+.2f | "
            "left_torque=%+.2f right_torque=%+.2f Nm",
            joystick_left_->x(), joystick_left_->y(), target, *left_velocity_, *right_velocity_,
            *left_torque_, *right_torque_);
    }

private:
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kMaxTorque = 1.8;

    double deadzone(double value) const {
        return std::abs(value) < joystick_deadzone_ ? 0.0 : value;
    }

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<double> left_velocity_;
    InputInterface<double> right_velocity_;

    OutputInterface<double> left_torque_;
    OutputInterface<double> right_torque_;

    pid::PidCalculator pid_;

    double pitch_sensitivity_ = 3.0;
    double pitch_max_velocity_ = 3.0;
    double joystick_deadzone_ = 0.02;
    double sync_coefficient_ = 0.0;
    std::string feedback_source_{"average"};

    double test_velocity_ = 0.0;
    double test_kp_ = 0.5;
    double test_torque_limit_ = 0.6;
};

} // namespace rmcs_core::controller::gantry

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gantry::PitchController, rmcs_executor::Component)
