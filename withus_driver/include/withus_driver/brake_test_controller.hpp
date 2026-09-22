#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "withus_driver/control_manager.hpp"

namespace withus_driver
{

// A bounded stationary brake-response test, independent of the 0x501 request.
// Invalidating a 0x503 command does not confirm physical brake release or a stop.
class BrakeTestController
{
public:
    using Clock = ControlManager::Clock;
    using TimePoint = ControlManager::TimePoint;
    using Feedback = ControlManager::Feedback;

    enum class State { Idle, Active, Releasing, Disconnected };

    struct Config
    {
        bool tx_enabled{false};
        std::chrono::milliseconds command_timeout{200};
        std::chrono::milliseconds max_test_duration{2000};
        std::chrono::milliseconds release_duration{1000};
        std::chrono::milliseconds stopped_dwell{200};
        double stopped_threshold_mps{0.05};
        double max_brake_percent{100.0};
    };

    struct Result
    {
        bool accepted{false};
        std::string reason;
    };

    struct Intent
    {
        bool transmit{false};
        BrakeCommand command{};
    };

    BrakeTestController(const Config& config, std::uint64_t session_seed);
    Result accept(double brake_percent, std::uint64_t session_id,
                  const Feedback& feedback, TimePoint now);
    void tick(const Feedback& feedback, TimePoint now);
    void on_tx_failure(const std::string& reason, TimePoint now);
    // Cancels the current test; a fresh command in a new session is needed.
    void cancel(TimePoint now);
    // Pausing also cancels; re-enabling never restores a cached command.
    void set_allowed(bool allowed, const Feedback& feedback, TimePoint now);
    void on_disconnect();

    State state() const noexcept { return state_; }
    bool active() const noexcept { return state_ == State::Active; }
    bool releasing() const noexcept { return state_ == State::Releasing; }
    bool ready() const noexcept { return ready_; }
    const std::string& reason() const noexcept { return reason_; }
    std::uint64_t session_id() const noexcept { return session_id_; }
    double command_age_sec(TimePoint now) const noexcept;
    const Intent& intent() const noexcept { return intent_; }

private:
    std::string feedback_problem(const Feedback& feedback) const;
    void begin_release(TimePoint now, const std::string& reason);
    void renew_session();
    void clear_command();
    void build_intent();

    Config config_;
    State state_{State::Idle};
    std::string reason_{"Waiting for stationary brake-test feedback."};
    std::uint64_t session_id_;
    Intent intent_{};
    bool allowed_{true};
    bool ready_{false};
    bool stationary_{false};
    bool command_received_{false};
    bool session_exhausted_{false};
    double brake_percent_{0.0};
    TimePoint stationary_since_{};
    TimePoint active_started_{};
    TimePoint command_received_at_{};
    TimePoint release_started_{};
};

}  // namespace withus_driver
