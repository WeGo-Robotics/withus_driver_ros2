#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "withus_driver/control_manager.hpp"

namespace withus_driver
{

// Client-owned drive targets. This controller never invents a stopping brake
// percentage. Invalidating commands is not evidence that the vehicle stopped.
// The independent hardware mode-request policy owns the continuous 0x501 request.
class HardwareDriveController
{
public:
    using Clock = ControlManager::Clock;
    using TimePoint = ControlManager::TimePoint;
    using Feedback = ControlManager::Feedback;
    using DriveTarget = ControlManager::DriveTarget;
    using Result = ControlManager::Result;

    enum class State { Idle, Commanding, Invalidating, Disconnected };

    struct Config
    {
        bool tx_enabled{false};
        std::chrono::milliseconds command_timeout{200};
        std::chrono::milliseconds invalidation_duration{1000};
        std::chrono::milliseconds stopped_dwell{200};
        double stopped_threshold_mps{0.05};
        double max_forward_mps{1.0};
        double max_reverse_mps{0.5};
        double max_steering_rad{0.3};
        std::chrono::milliseconds mode_request_timeout{1000};
        std::chrono::milliseconds gear_timeout{1000};
    };

    struct Intent
    {
        bool transmit{false};
        SteeringCommand steering{};
        BrakeCommand brake{};
        SpeedCommand speed{false, Gear::Neutral, 0.0};
    };

    HardwareDriveController(const Config& config, std::uint64_t session_seed);
    Result accept(const DriveTarget& target, const Feedback& feedback, TimePoint now);
    void tick(const Feedback& feedback, TimePoint now);
    // Call only after a COMPLETE successful 0x502/503/504 batch. Supply a
    // snapshot taken after its final write; only later 0x303 can confirm mode
    // or a requested gear. Partial batches must call on_tx_failure instead.
    void on_commands_sent(const Feedback& feedback, TimePoint now);
    void on_tx_failure(const std::string& reason, TimePoint now);
    void set_allowed(bool allowed, const Feedback& feedback, TimePoint now);
    void on_disconnect();

    State state() const noexcept { return state_; }
    bool ready() const noexcept { return ready_; }
    bool active() const noexcept { return state_ == State::Commanding; }
    bool invalidating() const noexcept { return state_ == State::Invalidating; }
    bool mode_confirmed() const noexcept { return mode_confirmed_; }
    bool gear_pending() const noexcept { return gear_pending_; }
    std::uint64_t session_id() const noexcept { return session_id_; }
    const std::string& reason() const noexcept { return reason_; }
    double command_age_sec(TimePoint now) const noexcept;
    const Intent& intent() const noexcept { return intent_; }

private:
    std::string feedback_problem(const Feedback& feedback) const;
    std::string target_problem(const DriveTarget& target) const;
    bool stopped(const Feedback& feedback) const noexcept;
    bool stationary_for_dwell(TimePoint now) const noexcept;
    void update_stationary(const Feedback& feedback, TimePoint now);
    void clear_command();
    void renew_session();
    void retire_readiness();
    void begin_invalidation(TimePoint now, const std::string& reason);
    void build_intent();

    Config config_;
    State state_{State::Idle};
    std::string reason_{"Waiting for stationary hardware-drive feedback."};
    std::uint64_t session_id_;
    Intent intent_{};
    DriveTarget target_{};
    bool allowed_{true};
    bool ready_{false};
    bool stationary_{false};
    bool command_received_{false};
    bool session_exhausted_{false};
    bool mode_confirmed_{false};
    bool mode_batch_sent_{false};
    bool gear_pending_{false};
    bool gear_batch_sent_{false};
    Gear observed_gear_{Gear::Neutral};
    std::uint64_t mode_sequence_{0};
    std::uint64_t gear_sequence_{0};
    TimePoint stationary_since_{};
    TimePoint command_received_at_{};
    TimePoint command_started_at_{};
    TimePoint gear_started_at_{};
    TimePoint invalidation_started_at_{};
};

}  // namespace withus_driver
