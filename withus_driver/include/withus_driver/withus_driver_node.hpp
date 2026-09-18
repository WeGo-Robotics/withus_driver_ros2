#pragma once

#include <atomic>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "withus_driver/socket_can_transport.hpp"

namespace withus_driver
{
class WithusDriverNode : public rclcpp::Node
{
public:
    WithusDriverNode();
    ~WithusDriverNode() override;
private:
    void receive_loop();

    SocketCanTransport transport_;
    std::atomic<bool> stop_requested_{false};
    std::thread receive_thread_;
};

}// namespace withus_driver