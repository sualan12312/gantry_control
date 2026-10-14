#include <algorithm>
#include <cmath>
#include <limits>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gantry {

class GantryController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);

        register_output("/gantry/yaw/motor/control_velocity", yaw_velocity_, kNan);

        get_parameter_or("yaw_sensitivity", yaw_sensitivity_, 3.0);
        get_parameter_or("yaw_max_velocity", yaw_max_velocity_, 3.0);
        get_parameter_or("joystick_deadzone", joystick_deadzone_, 0.02);
    }

    void update() override {
        using namespace rmcs_msgs;

        const auto sw_left = *switch_left_;
        const auto sw_right = *switch_right_;

        if (sw_left == Switch::UNKNOWN || sw_right == Switch::UNKNOWN
            || (sw_left == Switch::DOWN && sw_right == Switch::DOWN)) {
            *yaw_velocity_ = kNan;
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000, "[Gantry] disabled: L=%d R=%d",
                static_cast<int>(sw_left), static_cast<int>(sw_right));
            return;
        }

        double velocity = yaw_sensitivity_ * deadzone(joystick_right_->y());
        velocity = std::clamp(velocity, -yaw_max_velocity_, yaw_max_velocity_);

        *yaw_velocity_ = velocity;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[Gantry] L=%d R=%d stick=(%+.3f,%+.3f) target=%+.2f rad/s",
            static_cast<int>(sw_left), static_cast<int>(sw_right), joystick_right_->x(),
            joystick_right_->y(), velocity);
    }

private:
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    double deadzone(double value) const {
        return std::abs(value) < joystick_deadzone_ ? 0.0 : value;
    }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    OutputInterface<double> yaw_velocity_;

    double yaw_sensitivity_ = 3.0;
    double yaw_max_velocity_ = 3.0;
    double joystick_deadzone_ = 0.02;
};

} // namespace rmcs_core::controller::gantry

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gantry::GantryController, rmcs_executor::Component)
