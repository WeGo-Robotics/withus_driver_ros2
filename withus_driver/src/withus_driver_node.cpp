#include "withus_driver/withus_driver_node.hpp"
#include "withus_driver/withus_protocol.hpp"

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace withus_driver
{
WithusDriverNode::WithusDriverNode()
    : rclcpp::Node("withus_driver", "withus")
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "SocketCAN interface name";
    descriptor.read_only = true;
    const auto interface_name = this->declare_parameter<std::string>(
        "can_interface", "can0", descriptor
    );

    rcl_interfaces::msg::ParameterDescriptor timeout_descriptor;
    timeout_descriptor.description = "Maximum age of feedback in milliseconds";
    timeout_descriptor.read_only = true;
    const auto feedback_timeout_ms = this->declare_parameter<std::int64_t>(
        "feedback_timeout_ms", 200, timeout_descriptor
    );
    if (feedback_timeout_ms <= 0) {
        throw std::invalid_argument("feedback_timeout_ms must be greater than zero");
    }
    feedback_timeout_sec_ = static_cast<double>(feedback_timeout_ms) / 1000.0;

    if (!transport_.open(interface_name)) {
        throw std::runtime_error("Failed to open CAN interface: " + interface_name);
    }
    RCLCPP_INFO(this->get_logger(), "CAN socket opened on: %s", interface_name.c_str());

    status_publisher_ = this->create_publisher<withus_msgs::msg::VehicleStatus>(
        "vehicle_status", rclcpp::QoS(10)
    );
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() { this->publish_vehicle_status(); }
    );
    receive_thread_ = std::thread(&WithusDriverNode::receive_loop, this);
}

WithusDriverNode::~WithusDriverNode()
{
    if (status_timer_) {
        status_timer_->cancel();
    }
    stop_requested_.store(true);
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    transport_.close();
}

void WithusDriverNode::receive_loop()
{
    while (!stop_requested_.load() && rclcpp::ok()) {
        CanFrame frame{};
        const auto result = transport_.receive(frame, 100);
        if (stop_requested_.load() || !rclcpp::ok()) {
            break;
        }
        if (result == ReceiveResult::Timeout) {
            continue;
        }
        if (result == ReceiveResult::Error) {
            RCLCPP_ERROR(this->get_logger(), "CAN receive failed; receive loop stopped.");
            break;
        }

        const auto received_at = std::chrono::steady_clock::now();
        const auto received_ros_nanoseconds = this->now().nanoseconds();

        switch (frame.id) {
        case kVehicleDiagnosisId: {
            VehicleDiagnosis decoded{};
            if (!decode_vehicle_diagnosis(frame, decoded)) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                ++vehicle_state_.status_301.rejected_frames;
                break;
            }
            std::lock_guard<std::mutex> lock(state_mutex_);
            vehicle_state_.diagnosis = decoded;
            vehicle_state_.status_301.mark_received(
                decoded.message_counter, received_at, received_ros_nanoseconds
            );
            break;
        }
        case kVehicleStatus1Id: {
            VehicleStatus1 decoded{};
            if (!decode_vehicle_status1(frame, decoded)) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                ++vehicle_state_.status_303.rejected_frames;
                break;
            }
            std::lock_guard<std::mutex> lock(state_mutex_);
            vehicle_state_.status1 = decoded;
            vehicle_state_.status_303.mark_received(
                decoded.message_counter, received_at, received_ros_nanoseconds
            );
            break;
        }
        case kVehicleStatus2Id: {
            VehicleStatus2 decoded{};
            if (!decode_vehicle_status2(frame, decoded)) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                ++vehicle_state_.status_304.rejected_frames;
                break;
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                vehicle_state_.status2 = decoded;
                vehicle_state_.status_304.mark_received(
                    decoded.message_counter, received_at, received_ros_nanoseconds
                );
            }
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Speed: %.2f m/s, Steering: %.3f rad, Brake: %.0f Pa",
                decoded.speed_mps, decoded.steering_angle_rad, decoded.brake_pressure_pa
            );
            break;
        }
        default:
            break;
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
    // Take time after the snapshot to avoid negative ages from a concurrent update.
    const auto steady_now = std::chrono::steady_clock::now();
    withus_msgs::msg::VehicleStatus msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    msg.speed_mps = snapshot.status2.speed_mps;
    msg.steering_angle_rad = snapshot.status2.steering_angle_rad;
    msg.brake_pressure_pa = snapshot.status2.brake_pressure_pa;

    msg.gear = snapshot.status1.gear;
    msg.drive_mode = snapshot.status1.drive_mode;
    msg.brake_hold_active = snapshot.status1.brake_hold_active;
    msg.requested_speed_mps = snapshot.status1.requested_speed_mps;
    msg.battery_soc = snapshot.status1.battery_soc;

    msg.battery_voltage_v = snapshot.diagnosis.battery_voltage_v;
    msg.emergency_stop_pressed = snapshot.diagnosis.emergency_stop_pressed;
    msg.motor_fault = snapshot.diagnosis.motor_fault;
    msg.remote_fault = snapshot.diagnosis.remote_fault;
    msg.ad_signal_received = snapshot.diagnosis.ad_signal_received;
    msg.dbs_fault = snapshot.diagnosis.dbs_fault;
    msg.parallel_driving_active = snapshot.diagnosis.parallel_driving_active;
    msg.bms_fault = snapshot.diagnosis.bms_fault;
    msg.eps_fault = snapshot.diagnosis.eps_fault;
    msg.front_bumper_triggered = snapshot.diagnosis.front_bumper_triggered;
    msg.rear_bumper_triggered = snapshot.diagnosis.rear_bumper_triggered;
    msg.headlight_on = snapshot.diagnosis.headlight_on;
    msg.left_indicator_on = snapshot.diagnosis.left_indicator_on;
    msg.brake_light_on = snapshot.diagnosis.brake_light_on;
    msg.oil_level_abnormal = snapshot.diagnosis.oil_level_abnormal;
    msg.horn_on = snapshot.diagnosis.horn_on;
    msg.right_indicator_on = snapshot.diagnosis.right_indicator_on;

    msg.status_301_received = snapshot.status_301.received;
    msg.status_301_valid = snapshot.status_301.valid(steady_now, feedback_timeout_sec_);
    msg.status_301_stamp = rclcpp::Time(snapshot.status_301.received_ros_nanoseconds, RCL_ROS_TIME);
    msg.status_301_age_sec = snapshot.status_301.age_sec(steady_now);
    msg.status_301_counter = snapshot.status_301.last_counter;
    msg.status_301_counter_discontinuities = snapshot.status_301.counter_discontinuities;
    msg.status_301_rejected_frames = snapshot.status_301.rejected_frames;

    msg.status_303_received = snapshot.status_303.received;
    msg.status_303_valid = snapshot.status_303.valid(steady_now, feedback_timeout_sec_);
    msg.status_303_stamp = rclcpp::Time(snapshot.status_303.received_ros_nanoseconds, RCL_ROS_TIME);
    msg.status_303_age_sec = snapshot.status_303.age_sec(steady_now);
    msg.status_303_counter = snapshot.status_303.last_counter;
    msg.status_303_counter_discontinuities = snapshot.status_303.counter_discontinuities;
    msg.status_303_rejected_frames = snapshot.status_303.rejected_frames;

    msg.status_304_received = snapshot.status_304.received;
    msg.status_304_valid = snapshot.status_304.valid(steady_now, feedback_timeout_sec_);
    msg.status_304_stamp = rclcpp::Time(snapshot.status_304.received_ros_nanoseconds, RCL_ROS_TIME);
    msg.status_304_age_sec = snapshot.status_304.age_sec(steady_now);
    msg.status_304_counter = snapshot.status_304.last_counter;
    msg.status_304_counter_discontinuities = snapshot.status_304.counter_discontinuities;
    msg.status_304_rejected_frames = snapshot.status_304.rejected_frames;

    // Preserve the existing speed-facing interface; reception is now grouped by ID.
    msg.speed_received = msg.status_304_received;
    msg.speed_valid = msg.status_304_valid;
    msg.speed_age_sec = msg.status_304_age_sec;
    status_publisher_->publish(msg);
}
} // namespace withus_driver