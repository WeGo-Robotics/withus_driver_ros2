#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "withus_driver/withus_protocol.hpp"

namespace withus_driver
{

// VcanReference controls the virtual vehicle. HardwareDrive continuously
// broadcasts 0x501 and observes the mode; HardwareDriveController separately
// owns real actuator commands. A mode request alone does not imply ownership.
class ControlManager
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    enum class State : std::uint8_t
    {
        Disconnected = 0, Monitoring = 1, Requesting = 2,
        Active = 3, Stopping = 4, Fault = 5
    };

    enum class Policy { VcanReference, HardwareDrive };

    struct Config
    {
        bool tx_enabled{false};
        double max_forward_mps{1.0};
        double max_reverse_mps{0.5};
        double max_steering_rad{0.3};
        double stop_brake_percent{25.0};
        double stopped_speed_mps{0.05};
        std::chrono::milliseconds command_timeout{200};
        std::chrono::milliseconds body_timeout{500};
        std::chrono::milliseconds request_timeout{1000};
        std::chrono::milliseconds stop_timeout{3000};
        std::chrono::milliseconds release_timeout{1000};
        std::chrono::milliseconds stopped_dwell{200};
        Policy policy{Policy::VcanReference};
    };

    struct Feedback
    {
        bool all_fresh{false};
        // Aggregate emergency stop, bumper and blocking faults.
        bool blocked{false};
        // Reference acknowledgement: AD drive mode AND AD signal received.
        bool ad_ready{false};
        Gear gear{Gear::Park};
        double speed_mps{0.0};
        double steering_angle_rad{0.0};
        // Increment for each accepted 0x303 frame; NOT the CAN counter nibble.
        std::uint64_t status_303_sequence{0};
        bool remote_fresh{false};
        bool remote_selected{false};
        std::uint8_t drive_mode{255};
        // Raw A values other than 0 and 1 are not interpreted as a selection.
        bool remote_known{false};
    };

    struct DriveTarget
    {
        std::uint64_t session_id{0};
        Gear gear{Gear::Park};
        double speed_mps{0.0};
        double steering_angle_rad{0.0};
        double brake_percent{0.0};
    };

    struct Result
    {
        bool accepted{false};
        std::string reason;
    };

    struct TxIntent
    {
        bool transmit{false};
        ControlRequest request{};
        SteeringCommand steering{};
        BrakeCommand brake{};
        SpeedCommand speed{};
        BodyCommand body{};
    };

    // seed should be nonzero and unique for each node process.
    ControlManager(const Config& config, std::uint64_t session_seed);
    Result set_enabled(bool enable, const Feedback& feedback, TimePoint now);
    Result accept_drive(const DriveTarget& target, const Feedback& feedback, TimePoint now);
    Result accept_body(std::uint64_t session, const BodyCommand& body, TimePoint now);
    void tick(const Feedback& feedback, TimePoint now);
    // The caller supplies a snapshot taken AFTER the successful 0x501 write.
    // Only later 0x303 feedback can acknowledge that request/release phase.
    void on_control_request_sent(bool request, const Feedback& feedback, TimePoint now);
    void on_tx_failure(const std::string& reason, TimePoint now = Clock::now());
    void on_disconnect();

    State state() const noexcept { return state_; }
    const std::string& reason() const noexcept { return reason_; }
    std::uint64_t session_id() const noexcept { return session_id_; }
    bool tx_enabled() const noexcept { return config_.tx_enabled; }
    double command_age_sec(TimePoint now) const noexcept;
    const TxIntent& intent() const noexcept { return intent_; }

private:
    bool acquisition_angle_allowed(double angle_rad) const noexcept;
    bool healthy(const Feedback& feedback) const noexcept;
    bool stopped(const Feedback& feedback) const noexcept;
    bool stationary_for_dwell(TimePoint now) const noexcept;
    void update_stationary(const Feedback& feedback, TimePoint now);
    void begin_stopping(const Feedback& feedback, TimePoint now, const std::string& reason);
    void fail(const std::string& reason);
    void clear_commands();
    void build_intent(const Feedback& feedback, TimePoint now);
    Result set_hardware_request_enabled(bool enable, const Feedback& feedback, TimePoint now);
    void tick_hardware_request(const Feedback& feedback, TimePoint now);
    void build_hardware_request_intent();

    Config config_;
    State state_{State::Monitoring};
    std::string reason_{"Monitoring; control has not been requested."};
    std::uint64_t session_id_;
    DriveTarget target_{};
    BodyCommand body_{};
    TxIntent intent_{};
    TimePoint phase_started_{};
    TimePoint active_started_{};
    TimePoint command_received_{};
    TimePoint body_received_{};
    TimePoint stationary_since_{};
    TimePoint gear_requested_at_{};
    bool command_received_once_{false};
    bool body_received_once_{false};
    bool stationary_{false};
    bool releasing_{false};
    bool gear_pending_{false};
    std::uint64_t phase_sequence_{0};
    Gear stopping_gear_{Gear::Park};
    double held_steering_rad_{0.0};
    bool hardware_release_sent_{false};
    bool hardware_request_enabled_{true};
    bool hardware_request_tx_failed_{false};
};

}  // namespace withus_driver
