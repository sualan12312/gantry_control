#include <memory>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class GantryLauncher
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    GantryLauncher()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<GantryLauncherCommand>(
                  get_component_name() + "_command", *this))
        , yaw_motor_(*this, *command_component_, "/gantry/yaw/motor")
        , pitch_left_motor_(*this, *command_component_, "/gantry/pitch/left/motor")
        , pitch_right_motor_(*this, *command_component_, "/gantry/pitch/right/motor") {

        yaw_motor_.configure(device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 1});
        pitch_left_motor_.configure(device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 2});
        pitch_right_motor_.configure(device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 3});

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);
    }

    void update() override {
        yaw_motor_.update_status();
        pitch_left_motor_.update_status();
        pitch_right_motor_.update_status();
        dr16_.update_status();
        remote_control_->update();
    }

    void command_update() {
        if (!motor_online_)
            return;

        auto builder = board_->start_transmit();
        builder.can_transmit(
            Spec::kCans.kCan1, //
            {
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        yaw_motor_.generate_command(),
                        pitch_left_motor_.generate_command(),
                        pitch_right_motor_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });
    }

protected:
    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000, "[GantryLauncher] uart data: id=%d, %zu bytes",
            static_cast<int>(uart.data_id), data.uart_data.size());

        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
            [[unlikely]]
            return;

        if (can != Spec::kCans.kCan1)
            return;

        if (data.can_id == 0x201) {
            yaw_motor_.store_status(data.can_data);
        } else if (data.can_id == 0x202) {
            pitch_left_motor_.store_status(data.can_data);
        } else if (data.can_id == 0x203) {
            pitch_right_motor_.store_status(data.can_data);
        } else {
            return;
        }

        if (!motor_online_) {
            motor_online_ = true;
            RCLCPP_INFO(get_logger(), "[GantryLauncher] motor feedback received, transmitting");
        }
    }

private:
    class GantryLauncherCommand : public rmcs_executor::Component {
    public:
        explicit GantryLauncherCommand(GantryLauncher& launcher)
            : launcher_(launcher) {}

        void update() override { launcher_.command_update(); }

    private:
        GantryLauncher& launcher_;
    };

    std::shared_ptr<GantryLauncherCommand> command_component_;

    device::DjiMotor yaw_motor_;
    device::DjiMotor pitch_left_motor_;
    device::DjiMotor pitch_right_motor_;
    bool motor_online_ = false;
    std::unique_ptr<librmcs::board::CBoard> board_;

    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::GantryLauncher, rmcs_executor::Component)
