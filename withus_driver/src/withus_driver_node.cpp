#include "withus_driver/withus_driver_node.hpp"
#include "withus_driver/withus_protocol.hpp"

#include <stdexcept>
#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace withus_driver
{
WithusDriverNode::WithusDriverNode()
    : rclcpp::Node("withus_driver", "withus")
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "SocketCan interface name";
    descriptor.read_only = true;

    const std::string interface_name = this->declare_parameter<std::string>(
        "can_interface", "can0", descriptor
    );

    if(!transport_.open(interface_name)){
        throw std::runtime_error(
            "Failed to open CAN interface: " + interface_name
        );
    }

    RCLCPP_INFO(this->get_logger(),
     "CAN socket opened on : %s",
    interface_name.c_str());

    receive_thread_ = std::thread(&WithusDriverNode::receive_loop, this);
}

WithusDriverNode::~WithusDriverNode()
{
    stop_requested_.store(true);
    if(receive_thread_.joinable()){
        receive_thread_.join();
    }
    transport_.close();
}


void WithusDriverNode::receive_loop()
{
    while(!stop_requested_.load() && rclcpp::ok())
    {
        CanFrame frame{};
        const auto result = transport_.receive(frame, 100);

        if(stop_requested_.load() || !rclcpp::ok()){
            break;
        }

        if(result == ReceiveResult::Timeout){
            continue;
        }

        if(result == ReceiveResult::Error){
            RCLCPP_ERROR(
                this->get_logger(),
                "CAN receive failed; receive loop stopped."
            );
            break;
        }

        double speed_mps{0.0};
        if(decode_vehicle_speed(frame, speed_mps)){
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Speed: %.2f m/s",
                speed_mps
            );
        }
    }
}


} // namespace withus_driver