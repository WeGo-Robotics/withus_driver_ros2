#pragma once

#include <atomic>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "withus_driver/socket_can_transport.hpp"
#include "withus_driver/vehicle_state.hpp"
#include "withus_msgs/msg/vehicle_status.hpp"

namespace withus_driver
{
class WithusDriverNode : public rclcpp::Node
{
public:
    WithusDriverNode();
    ~WithusDriverNode() override;
private:
    void receive_loop();
    void publish_vehicle_status();

    SocketCanTransport transport_;

    std::mutex state_mutex_;
    VehicleState vehicle_state_;

    rclcpp::Publisher<withus_msgs::msg::VehicleStatus>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::atomic<bool> stop_requested_{false};
    std::thread receive_thread_;
};

}// namespace withus_driver