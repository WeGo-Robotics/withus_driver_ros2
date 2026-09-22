#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "withus_driver/socket_can_transport.hpp"
#include "withus_driver/vehicle_state.hpp"
#include "withus_driver/control_manager.hpp"
#include "withus_driver/brake_test_controller.hpp"
#include "withus_msgs/msg/vehicle_status.hpp"
#include "withus_msgs/msg/driver_status.hpp"
#include "withus_msgs/msg/drive_command.hpp"
#include "withus_msgs/msg/body_command.hpp"
#include "withus_msgs/msg/brake_test_command.hpp"
#include "withus_msgs/srv/set_control.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace withus_driver
{
class WithusDriverNode : public rclcpp::Node
{
public:
    WithusDriverNode();
    ~WithusDriverNode() override;
    void request_shutdown();
    bool ready_to_shutdown() const;
    bool control_released() const {
        return control_->state() == ControlManager::State::Monitoring &&
               (!brake_test_ || (!brake_test_->active() && !brake_test_->releasing()));
    }
    std::chrono::milliseconds shutdown_timeout() const { return shutdown_timeout_; }
private:
    void receive_loop();
    void publish_vehicle_status();
    void publish_driver_status();
    void publish_auxiliary();
    ControlManager::Feedback feedback_snapshot();
    void update_control(const ControlManager::Feedback& feedback);
    bool valid_header(const std_msgs::msg::Header& header, std::int64_t last_stamp);
    void reject(const std::string& reason);
    void on_drive(const withus_msgs::msg::DriveCommand& msg);
    void on_body(const withus_msgs::msg::BodyCommand& msg);
    void on_brake_test(const withus_msgs::msg::BrakeTestCommand& msg);
    void transmit(bool body_only);
    void transmit_brake_test();
    SocketCanTransport transport_;
    std::unique_ptr<ControlManager> control_;
    std::unique_ptr<BrakeTestController> brake_test_;
    std::mutex state_mutex_;
    VehicleState vehicle_state_;
    std::uint64_t status_303_sequence_{0};
    double feedback_timeout_sec_{0.2};
    std::string base_frame_id_;
    std::string control_profile_;
    bool mode_only_{false};
    std::array<std::uint8_t, 5> counters_{};
    std::int64_t max_command_age_ns_{0}, future_tolerance_ns_{0}, active_ros_ns_{0};
    std::int64_t last_ros_ns_{0}, last_drive_stamp_{0}, last_body_stamp_{0};
    std::int64_t last_brake_test_stamp_{0};
    std::uint64_t brake_session_id_{0};
    std::uint64_t rejected_commands_{0}, tx_sent_frames_{0}, tx_failed_frames_{0};
    std::string last_rejection_;
    bool shutting_down_{false};
    std::chrono::milliseconds shutdown_timeout_{4500};
    rclcpp::Publisher<withus_msgs::msg::VehicleStatus>::SharedPtr status_publisher_;
    rclcpp::Publisher<withus_msgs::msg::DriverStatus>::SharedPtr driver_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::Subscription<withus_msgs::msg::DriveCommand>::SharedPtr drive_subscription_;
    rclcpp::Subscription<withus_msgs::msg::BodyCommand>::SharedPtr body_subscription_;
    rclcpp::Subscription<withus_msgs::msg::BrakeTestCommand>::SharedPtr brake_test_subscription_;
    rclcpp::Service<withus_msgs::srv::SetControl>::SharedPtr control_service_;
    rclcpp::TimerBase::SharedPtr status_timer_, tx_timer_, body_timer_, diagnostics_timer_;
    std::atomic<bool> stop_requested_{false}, can_connected_{false};
    std::thread receive_thread_;
};
}
