#include "withus_driver/withus_driver_node.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <chrono>
#include <algorithm>
#include "withus_driver/can_interface.hpp"
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace withus_driver
{
WithusDriverNode::WithusDriverNode() : rclcpp::Node("withus_driver", "withus")
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto interface_name = declare_parameter<std::string>("can_interface", "can0", descriptor);
    control_profile_ = declare_parameter<std::string>("control_profile", "monitor_only", descriptor);
    const bool brake_test_profile = control_profile_ == "hardware_brake_test";
    const bool remote_mode = control_profile_ == "hardware_remote_mode" || brake_test_profile;
    mode_only_ = control_profile_ == "hardware_mode_only" || remote_mode;
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link", descriptor);
    if (base_frame_id_.empty()) throw std::invalid_argument("base_frame_id is empty");
    auto milliseconds = [&](const char* name, int initial) {
        const auto value = declare_parameter<std::int64_t>(name, initial, descriptor);
        if (value <= 0 || value > 60000) throw std::invalid_argument(std::string(name) + " must be 1..60000");
        return std::chrono::milliseconds(value);
    };
    feedback_timeout_sec_ = milliseconds("feedback_timeout_ms", 200).count() / 1000.0;
    max_command_age_ns_ = milliseconds("max_command_age_ms", 200).count() * 1000000;
    future_tolerance_ns_ = milliseconds("future_stamp_tolerance_ms", 50).count() * 1000000;
    ControlManager::Config config;
    config.tx_enabled = declare_parameter<bool>("enable_tx", false, descriptor);
    if (control_profile_ != "monitor_only" && control_profile_ != "vcan_reference" && !mode_only_)
        throw std::invalid_argument("Unknown control_profile");
    config.policy = remote_mode ? ControlManager::Policy::RemoteMode :
        (mode_only_ ? ControlManager::Policy::ModeOnly : ControlManager::Policy::VcanReference);
    if (config.tx_enabled && control_profile_ == "monitor_only") {
        throw std::invalid_argument("monitor_only does not permit TX");
    }
    if (config.tx_enabled && control_profile_ == "vcan_reference" && !is_vcan_interface(interface_name)) {
        throw std::invalid_argument("vcan_reference TX requires kernel interface kind vcan");
    }
    config.command_timeout = milliseconds("command_timeout_ms", 200);
    config.body_timeout = milliseconds("body_command_timeout_ms", 500);
    config.request_timeout = milliseconds("control_request_timeout_ms", 1000);
    config.stop_timeout = milliseconds("stop_timeout_ms", 3000);
    config.release_timeout = milliseconds("release_timeout_ms", 1000);
    config.stopped_dwell = milliseconds("stop_confirm_duration_ms", 200);
    config.max_forward_mps = declare_parameter<double>("max_forward_speed_mps", 1.0, descriptor);
    config.max_reverse_mps = declare_parameter<double>("max_reverse_speed_mps", 0.5, descriptor);
    config.max_steering_rad = declare_parameter<double>("max_steering_angle_rad", 0.3, descriptor);
    config.stop_brake_percent = declare_parameter<double>("stop_brake_percent", 25.0, descriptor);
    config.stopped_speed_mps = declare_parameter<double>("stop_speed_threshold_mps", 0.05, descriptor);
    if (config.stop_timeout + config.release_timeout > std::chrono::milliseconds(10000))
        throw std::invalid_argument("stop_timeout_ms + release_timeout_ms must be <= 10000");
    shutdown_timeout_ = config.stop_timeout + config.release_timeout + std::chrono::milliseconds(500);
    std::random_device random;
    const std::uint64_t seed = (static_cast<std::uint64_t>(random()) << 32) | random();
    control_ = std::make_unique<ControlManager>(config, seed == 0 ? 1 : seed);
    if (brake_test_profile) {
        BrakeTestController::Config brake_config;
        brake_config.tx_enabled = config.tx_enabled;
        brake_config.command_timeout = config.command_timeout;
        brake_config.max_test_duration = milliseconds("brake_test_max_duration_ms", 2000);
        brake_config.release_duration = milliseconds("brake_test_release_duration_ms", 1000);
        brake_config.stopped_dwell = config.stopped_dwell;
        brake_config.stopped_threshold_mps = config.stopped_speed_mps;
        brake_config.max_brake_percent = declare_parameter<double>("brake_test_max_percent", 100.0, descriptor);
        if (brake_config.max_test_duration > std::chrono::milliseconds(5000) ||
            brake_config.release_duration > std::chrono::milliseconds(5000)) {
            throw std::invalid_argument("Brake test and invalidation intervals must be <= 5000 ms");
        }
        brake_test_ = std::make_unique<BrakeTestController>(brake_config, seed == 0 ? 1 : seed);
        brake_session_id_ = brake_test_->session_id();
        shutdown_timeout_ = std::max(shutdown_timeout_,
            brake_config.release_duration + std::chrono::milliseconds(500));
        active_ros_ns_ = now().nanoseconds();
    }
    if (!transport_.open(interface_name)) throw std::runtime_error("Failed to open " + interface_name);
    RCLCPP_INFO(get_logger(), "CAN socket opened on: %s; control_profile=%s; TX=%s",
        interface_name.c_str(), control_profile_.c_str(), config.tx_enabled ? "enabled" : "disabled");
    status_publisher_ = create_publisher<withus_msgs::msg::VehicleStatus>("vehicle_status", 10);
    driver_publisher_ = create_publisher<withus_msgs::msg::DriverStatus>("driver_status", 10);
    battery_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    drive_subscription_ = create_subscription<withus_msgs::msg::DriveCommand>("drive_command", rclcpp::QoS(1),
        [this](withus_msgs::msg::DriveCommand::ConstSharedPtr msg) { on_drive(*msg); });
    body_subscription_ = create_subscription<withus_msgs::msg::BodyCommand>("body_command", rclcpp::QoS(1),
        [this](withus_msgs::msg::BodyCommand::ConstSharedPtr msg) { on_body(*msg); });
    if (brake_test_) {
        brake_test_subscription_ = create_subscription<withus_msgs::msg::BrakeTestCommand>(
            "brake_test_command", rclcpp::QoS(1),
            [this](withus_msgs::msg::BrakeTestCommand::ConstSharedPtr msg) { on_brake_test(*msg); });
    }
    control_service_ = create_service<withus_msgs::srv::SetControl>("set_control",
        [this](const std::shared_ptr<withus_msgs::srv::SetControl::Request> req,
               std::shared_ptr<withus_msgs::srv::SetControl::Response> res) {
            const auto fb = feedback_snapshot();
            update_control(fb);
            if (shutting_down_ && req->enable) {
                res->accepted = false; res->message = "Shutdown is in progress";
            } else if (req->enable && now().nanoseconds() <= 0) {
                res->accepted = false; res->message = "ROS clock is not initialized";
            } else if (!can_connected_.load()) {
                res->accepted = false; res->message = "CAN receive connection is unavailable";
            } else {
                const auto result = control_->set_enabled(req->enable, fb, ControlManager::Clock::now());
                res->accepted = result.accepted; res->message = result.reason;
                if (result.accepted && brake_test_) {
                    brake_test_->set_allowed(req->enable, fb, ControlManager::Clock::now());
                }
            }
            res->session_id = brake_test_ ? brake_test_->session_id() : control_->session_id();
        });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
        publish_vehicle_status(); publish_driver_status();
    });
    tx_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { transmit(false); });
    body_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { transmit(true); });
    diagnostics_timer_ = create_wall_timer(std::chrono::milliseconds(1000), [this]() { publish_auxiliary(); });
    can_connected_.store(true);
    receive_thread_ = std::thread(&WithusDriverNode::receive_loop, this);
}

WithusDriverNode::~WithusDriverNode()
{
    for (const auto& timer : {status_timer_, tx_timer_, body_timer_, diagnostics_timer_}) {
        if (timer) timer->cancel();
    }
    stop_requested_.store(true);
    if (receive_thread_.joinable()) receive_thread_.join();
    transport_.close();
}

ControlManager::Feedback WithusDriverNode::feedback_snapshot()
{
    VehicleState snapshot;
    ControlManager::Feedback result;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        snapshot = vehicle_state_;
        result.status_303_sequence = status_303_sequence_;
    }
    const auto time = ControlManager::Clock::now();
    result.all_fresh = can_connected_.load() &&
        snapshot.status_301.valid(time, feedback_timeout_sec_) &&
        snapshot.status_303.valid(time, feedback_timeout_sec_) &&
        snapshot.status_304.valid(time, feedback_timeout_sec_);
    const auto& d = snapshot.diagnosis;
    result.blocked = d.emergency_stop_pressed || d.motor_fault || d.remote_fault ||
        d.dbs_fault || d.bms_fault || d.eps_fault || d.front_bumper_triggered ||
        d.rear_bumper_triggered || d.parallel_driving_active;
    result.ad_ready = snapshot.status1.drive_mode == 1 && d.ad_signal_received;
    result.drive_mode = snapshot.status1.drive_mode;
    result.remote_fresh = can_connected_.load() && snapshot.status_10a.valid(time, feedback_timeout_sec_);
    result.remote_selected = snapshot.remote_io.a_switch_raw == 1;
    result.remote_known = snapshot.remote_io.a_switch_raw <= 1;
    result.gear = static_cast<Gear>(snapshot.status1.gear);
    result.speed_mps = snapshot.status2.speed_mps;
    result.steering_angle_rad = snapshot.status2.steering_angle_rad;
    return result;
}

void WithusDriverNode::update_control(const ControlManager::Feedback& feedback)
{
    const auto previous = control_->state();
    const auto ros_ns = now().nanoseconds();
    if (!can_connected_.load()) {
        control_->on_disconnect();
        if (brake_test_) brake_test_->on_disconnect();
    } else if (last_ros_ns_ > 0 && ros_ns < last_ros_ns_ && control_->intent().transmit) {
        control_->on_tx_failure("ROS clock moved backwards");
        if (brake_test_) brake_test_->on_tx_failure("ROS clock moved backwards", ControlManager::Clock::now());
    } else {
        control_->tick(feedback, ControlManager::Clock::now());
        if (brake_test_) brake_test_->tick(feedback, ControlManager::Clock::now());
    }
    if (brake_test_ && brake_test_->session_id() != brake_session_id_) {
        brake_session_id_ = brake_test_->session_id();
        active_ros_ns_ = ros_ns;
        last_brake_test_stamp_ = 0;
    }
    last_ros_ns_ = ros_ns;
    if (control_->state() != previous) {
        RCLCPP_INFO(get_logger(), "Control state: %u (%s)",
            static_cast<unsigned int>(control_->state()), control_->reason().c_str());
        if (control_->state() == ControlManager::State::Active) {
            active_ros_ns_ = ros_ns; last_drive_stamp_ = 0; last_body_stamp_ = 0;
        }
    }
}

bool WithusDriverNode::valid_header(const std_msgs::msg::Header& header, std::int64_t last_stamp)
{
    if (shutting_down_) { reject("Shutdown is in progress"); return false; }
    if (header.frame_id != base_frame_id_) { reject("Incorrect command frame_id"); return false; }
    if (header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000U) {
        reject("Invalid command timestamp"); return false;
    }
    const auto stamp = rclcpp::Time(header.stamp).nanoseconds();
    const auto ros_now = now().nanoseconds();
    if (stamp <= 0 || stamp < active_ros_ns_ || stamp <= last_stamp ||
        ros_now - stamp > max_command_age_ns_ || stamp - ros_now > future_tolerance_ns_) {
        reject("Stale, future, duplicate or pre-activation command timestamp"); return false;
    }
    return true;
}

void WithusDriverNode::reject(const std::string& reason)
{
    ++rejected_commands_; last_rejection_ = reason;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Command rejected: %s", reason.c_str());
}

void WithusDriverNode::on_drive(const withus_msgs::msg::DriveCommand& msg)
{
    const auto feedback = feedback_snapshot();
    update_control(feedback);
    if (mode_only_) { reject(control_profile_ + " does not accept DriveCommand"); return; }
    if (!valid_header(msg.header, last_drive_stamp_)) return;
    ControlManager::DriveTarget target;
    target.session_id = msg.session_id; target.gear = static_cast<Gear>(msg.gear);
    target.speed_mps = msg.speed_mps; target.steering_angle_rad = msg.steering_angle_rad;
    target.brake_percent = msg.brake_percent;
    const auto result = control_->accept_drive(target, feedback, ControlManager::Clock::now());
    if (!result.accepted) reject(result.reason);
    else last_drive_stamp_ = rclcpp::Time(msg.header.stamp).nanoseconds();
}

void WithusDriverNode::on_body(const withus_msgs::msg::BodyCommand& msg)
{
    update_control(feedback_snapshot());
    if (mode_only_) { reject(control_profile_ + " does not accept BodyCommand"); return; }
    if (!valid_header(msg.header, last_body_stamp_)) return;
    BodyCommand body;
    body.headlight_on = msg.headlight_on; body.left_indicator_on = msg.left_indicator_on;
    body.right_indicator_on = msg.right_indicator_on; body.horn_on = msg.horn_on;
    const auto result = control_->accept_body(msg.session_id, body, ControlManager::Clock::now());
    if (!result.accepted) reject(result.reason);
    else last_body_stamp_ = rclcpp::Time(msg.header.stamp).nanoseconds();
}

void WithusDriverNode::on_brake_test(const withus_msgs::msg::BrakeTestCommand& msg)
{
    const auto feedback = feedback_snapshot();
    update_control(feedback);
    if (!brake_test_ || !control_->intent().transmit ||
        !control_->intent().request.request_control) {
        reject("Stationary brake testing is not enabled"); return;
    }
    if (!valid_header(msg.header, last_brake_test_stamp_)) return;
    const auto result = brake_test_->accept(
        msg.brake_percent, msg.session_id, feedback, ControlManager::Clock::now());
    if (!result.accepted) reject(result.reason);
    else last_brake_test_stamp_ = rclcpp::Time(msg.header.stamp).nanoseconds();
}

void WithusDriverNode::transmit(bool body_only)
{
    update_control(feedback_snapshot());
    const auto intent = control_->intent();
    if (!can_connected_.load()) return;
    if (!intent.transmit) {
        if (!body_only) transmit_brake_test();
        return;
    }
    // Enforce the profile at the egress boundary as well as in ControlManager.
    if (mode_only_ && body_only) return;
    std::array<CanFrame, 5> frames{};
    if (mode_only_) {
        if (!encode_control_request(intent.request, counters_[0], frames[0])) {
            control_->on_tx_failure("Control request encoding failed"); return;
        }
    } else if (body_only) {
        if (!encode_body_command(intent.body, counters_[4], frames[4])) {
            control_->on_tx_failure("Body encoding failed"); return;
        }
    } else if (!encode_control_request(intent.request, counters_[0], frames[0]) ||
        !encode_steering_command(intent.steering, counters_[1], frames[1]) ||
        !encode_brake_command(intent.brake, counters_[2], frames[2]) ||
        !encode_speed_command(intent.speed, counters_[3], frames[3])) {
        control_->on_tx_failure("Drive encoding failed"); return;
    }
    const std::size_t begin = body_only ? 4 : 0;
    const std::size_t end = mode_only_ ? 1 : (body_only ? 5 : 4);
    for (auto i = begin; i < end; ++i) {
        const auto result = transport_.send(frames[i]);
        if (result != SendResult::Sent) {
            ++tx_failed_frames_;
            control_->on_tx_failure(result == SendResult::WouldBlock ? "CAN TX queue unavailable" : "CAN TX failed");
            if (brake_test_) {
                brake_test_->on_tx_failure("Mode request transmission failed", ControlManager::Clock::now());
                transmit_brake_test();
            }
            return;
        }
        ++tx_sent_frames_;
        counters_[i] = static_cast<std::uint8_t>((counters_[i] + 1U) & 0x0FU);
        if (i == 0) {
            control_->on_control_request_sent(intent.request.request_control,
                feedback_snapshot(), ControlManager::Clock::now());
        }
    }
    if (!body_only) transmit_brake_test();
}

void WithusDriverNode::transmit_brake_test()
{
    if (!brake_test_ || !can_connected_.load()) return;
    brake_test_->tick(feedback_snapshot(), ControlManager::Clock::now());
    const auto intent = brake_test_->intent();
    if (!intent.transmit) return;
    CanFrame frame{};
    // The only extra egress in this profile is 0x503. No steering or gear/speed frame.
    if (!encode_brake_command(intent.command, counters_[2], frame)) {
        brake_test_->on_tx_failure("Brake test encoding failed", ControlManager::Clock::now());
        return;
    }
    const auto result = transport_.send(frame);
    if (result != SendResult::Sent) {
        ++tx_failed_frames_;
        brake_test_->on_tx_failure(result == SendResult::WouldBlock ?
            "Brake TX queue unavailable" : "Brake TX failed", ControlManager::Clock::now());
        return;
    }
    ++tx_sent_frames_;
    counters_[2] = static_cast<std::uint8_t>((counters_[2] + 1U) & 0x0FU);
}

void WithusDriverNode::publish_driver_status()
{
    const auto feedback = feedback_snapshot();
    update_control(feedback);
    withus_msgs::msg::DriverStatus msg;
    msg.header.stamp = now(); msg.header.frame_id = base_frame_id_;
    msg.state = static_cast<std::uint8_t>(control_->state()); msg.reason = control_->reason();
    msg.last_rejection = last_rejection_;
    msg.can_connected = can_connected_.load(); msg.enable_tx = control_->tx_enabled();
    msg.feedback_valid = feedback.all_fresh && (!mode_only_ || feedback.remote_fresh);
    msg.session_id = brake_test_ ? brake_test_->session_id() : control_->session_id();
    msg.control_profile = control_profile_;
    msg.control_requested = control_->intent().transmit && control_->intent().request.request_control;
    msg.command_age_sec = control_->command_age_sec(ControlManager::Clock::now());
    msg.rejected_commands = rejected_commands_; msg.tx_sent_frames = tx_sent_frames_;
    msg.tx_failed_frames = tx_failed_frames_;
    msg.brake_test_ready = brake_test_ && brake_test_->ready();
    msg.brake_test_active = brake_test_ && brake_test_->active();
    msg.brake_test_reason = brake_test_ ? brake_test_->reason() : "";
    msg.brake_test_command_age_sec = brake_test_ ?
        brake_test_->command_age_sec(ControlManager::Clock::now()) : std::numeric_limits<double>::quiet_NaN();
    driver_publisher_->publish(msg);
}

void WithusDriverNode::publish_auxiliary()
{
    VehicleState snapshot;
    { std::lock_guard<std::mutex> lock(state_mutex_); snapshot = vehicle_state_; }
    const auto time = ControlManager::Clock::now();
    if (snapshot.status_301.valid(time, feedback_timeout_sec_)) {
        sensor_msgs::msg::BatteryState battery;
        battery.header.stamp = now(); battery.header.frame_id = base_frame_id_;
        const auto unknown = std::numeric_limits<float>::quiet_NaN();
        battery.voltage = static_cast<float>(snapshot.diagnosis.battery_voltage_v);
        battery.percentage = snapshot.status_303.valid(time, feedback_timeout_sec_)
            ? static_cast<float>(snapshot.status1.battery_soc) : unknown;
        battery.temperature = unknown; battery.current = unknown; battery.charge = unknown;
        battery.capacity = unknown; battery.design_capacity = unknown;
        battery.present = true;
        battery_publisher_->publish(battery);
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name(); status.hardware_id = "withus_socketcan";
    const auto feedback = feedback_snapshot();
    const bool feedback_valid = feedback.all_fresh && (!mode_only_ || feedback.remote_fresh);
    status.level = (feedback.blocked || control_->state() == ControlManager::State::Fault ||
        !can_connected_.load()) ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
        (feedback_valid ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN);
    status.message = control_->reason();
    diagnostic_msgs::msg::KeyValue value;
    value.key = "feedback_valid"; value.value = feedback_valid ? "true" : "false";
    status.values.push_back(value);
    if (brake_test_) {
        value.key = "brake_test_ready"; value.value = brake_test_->ready() ? "true" : "false";
        status.values.push_back(value);
        value.key = "brake_test_reason"; value.value = brake_test_->reason();
        status.values.push_back(value);
    }
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
}

void WithusDriverNode::request_shutdown()
{
    shutting_down_ = true;
    const auto feedback = feedback_snapshot(); update_control(feedback);
    if (brake_test_) brake_test_->set_allowed(false, feedback, ControlManager::Clock::now());
    const auto result = control_->set_enabled(false, feedback, ControlManager::Clock::now());
    RCLCPP_INFO(get_logger(), "Shutdown: %s", result.reason.c_str());
}

bool WithusDriverNode::ready_to_shutdown() const
{
    return (!brake_test_ || (!brake_test_->active() && !brake_test_->releasing())) &&
        (control_->state() == ControlManager::State::Monitoring ||
        control_->state() == ControlManager::State::Fault ||
        control_->state() == ControlManager::State::Disconnected);
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
        case kRemoteControlIoId: {
            RemoteControlIo decoded{};
            if (!decode_remote_control_io(frame, decoded)) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                ++vehicle_state_.status_10a.rejected_frames;
                break;
            }
            std::lock_guard<std::mutex> lock(state_mutex_);
            vehicle_state_.remote_io = decoded;
            // 0x10A has no counter; do not use mark_received(counter, ...).
            auto& feedback = vehicle_state_.status_10a;
            feedback.received = true;
            feedback.received_at = received_at;
            feedback.received_ros_nanoseconds = received_ros_nanoseconds;
            break;
        }
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
            ++status_303_sequence_;
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
    can_connected_.store(false);
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
    msg.header.frame_id = base_frame_id_;

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

    msg.remote_a_raw = snapshot.remote_io.a_switch_raw;
    msg.remote_a_selected = (snapshot.remote_io.a_switch_raw == 1);
    msg.status_10a_received = snapshot.status_10a.received;
    msg.status_10a_valid = snapshot.status_10a.valid(steady_now, feedback_timeout_sec_);
    msg.status_10a_stamp = rclcpp::Time(
        snapshot.status_10a.received_ros_nanoseconds, RCL_ROS_TIME);
    msg.status_10a_age_sec = snapshot.status_10a.age_sec(steady_now);
    msg.status_10a_rejected_frames = snapshot.status_10a.rejected_frames;
    status_publisher_->publish(msg);
}
} // namespace withus_driver
