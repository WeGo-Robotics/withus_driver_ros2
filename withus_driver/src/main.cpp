#include <chrono>
#include <csignal>
#include <exception>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "withus_driver/withus_driver_node.hpp"

namespace {
volatile std::sig_atomic_t interrupted = 0;
void signal_handler(int) { interrupted = 1; }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    int result = 0;
    try {
        auto node = std::make_shared<withus_driver::WithusDriverNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        while (rclcpp::ok() && !interrupted) {
            executor.spin_some(std::chrono::milliseconds(5));
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        if (rclcpp::ok()) {
            node->request_shutdown();
            const auto deadline = std::chrono::steady_clock::now() + node->shutdown_timeout();
            while (!node->ready_to_shutdown() && std::chrono::steady_clock::now() < deadline) {
                executor.spin_some(std::chrono::milliseconds(5));
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            if (!node->control_released()) {
                RCLCPP_ERROR(node->get_logger(), "Shutdown without confirmed control release");
                result = 1;
            }
        }
        executor.remove_node(node);
        node.reset();
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("withus_driver"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
