#include "withus_driver/withus_driver_node.hpp"
#include "withus_driver/withus_protocol.hpp"

#include <stdexcept>
#include <string>
#include <chrono>
#include <limits>

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

    status_publisher_ = this->create_publisher<withus_msgs::msg::VehicleStatus>(
        "vehicle_status", rclcpp::QoS(10)
    );

    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        [this](){
            this->publish_vehicle_status();
        }
    );

    receive_thread_ = std::thread(&WithusDriverNode::receive_loop, this);
}

WithusDriverNode::~WithusDriverNode()
{
    if(status_timer_){
        status_timer_->cancel();
    }
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
            const auto received_at = std::chrono::steady_clock::now();

            {
                std::lock_guard<std::mutex> lock(state_mutex_);

                vehicle_state_.speed_mps = speed_mps;
                vehicle_state_.speed_received = true;
                vehicle_state_.speed_received_at = received_at;
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Speed:%.2f m/s",
                speed_mps
            );
        }
    }
}

void WithusDriverNode::publish_vehicle_status()
{
    VehicleState snapshot{};
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        snapshot = vehicle_state_;
    }

    withus_msgs::msg::VehicleStatus msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.speed_mps = snapshot.speed_mps;
    msg.speed_received = snapshot.speed_received;
    msg.speed_age_sec = std::numeric_limits<double>::quiet_NaN();

    if(snapshot.speed_received){
        const auto now = std::chrono::steady_clock::now();
        msg.speed_age_sec = std::chrono::duration<double>(
            now - snapshot.speed_received_at).count();
    }

    status_publisher_->publish(msg);
}

} // namespace withus_driver