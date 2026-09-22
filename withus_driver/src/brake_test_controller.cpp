#include "withus_driver/brake_test_controller.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace withus_driver
{

BrakeTestController::BrakeTestController(const Config& config, std::uint64_t session_seed)
: config_(config), session_id_(session_seed)
{
    if (session_seed == 0 || config_.command_timeout.count() <= 0 ||
        config_.max_test_duration.count() <= 0 || config_.release_duration.count() <= 0 ||
        config_.stopped_dwell.count() <= 0 || !std::isfinite(config_.stopped_threshold_mps) ||
        config_.stopped_threshold_mps <= 0.0 || !std::isfinite(config_.max_brake_percent) ||
        config_.max_brake_percent <= 0.0 || config_.max_brake_percent > 100.0)
    {
        throw std::invalid_argument("Invalid brake-test controller configuration.");
    }
    if (!config_.tx_enabled) reason_ = "Brake-test transmission is disabled by configuration.";
}

std::string BrakeTestController::feedback_problem(const Feedback& feedback) const
{
    if (!feedback.all_fresh) return "Vehicle feedback 0x301/303/304 is stale or missing.";
    if (!feedback.remote_fresh) return "A-switch feedback 0x10A is stale or missing.";
    if (!feedback.remote_known) return "A-switch value is unknown.";
    if (!feedback.remote_selected) return "A switch is not selected.";
    if (feedback.blocked) return "Vehicle feedback reports a blocking condition.";
    if (feedback.gear != Gear::Neutral) return "Brake test requires Neutral gear.";
    if (!std::isfinite(feedback.speed_mps) ||
        std::abs(feedback.speed_mps) > config_.stopped_threshold_mps)
    {
        return "Vehicle speed is not a finite stationary measurement.";
    }
    if (feedback.drive_mode != 1 && feedback.drive_mode != 3) {
        return "Brake test requires observed drive_mode=1 or drive_mode=3.";
    }
    return {};
}

void BrakeTestController::clear_command()
{
    command_received_ = false;
    brake_percent_ = 0.0;
    ready_ = false;
    stationary_ = false;
}

void BrakeTestController::renew_session()
{
    if (session_id_ == std::numeric_limits<std::uint64_t>::max()) {
        session_exhausted_ = true;
    } else {
        ++session_id_;
    }
}

void BrakeTestController::begin_release(TimePoint now, const std::string& reason)
{
    if (state_ == State::Disconnected || state_ == State::Releasing) return;
    clear_command();
    renew_session();
    state_ = State::Releasing;
    release_started_ = now;
    reason_ = reason + " Sending invalid 0x503 for a bounded interval; physical release is not confirmed.";
    if (session_exhausted_) reason_ += " Session identifiers exhausted; restart required.";
    build_intent();
}

BrakeTestController::Result BrakeTestController::accept(
    double brake_percent, std::uint64_t session_id, const Feedback& feedback, TimePoint now)
{
    // Enforce the original total deadline even when a command arrives exactly
    // at expiry. Updating command_received_at_ must not extend active_started_.
    tick(feedback, now);
    if (!ready_) return {false, reason_};
    if (session_id == 0 || session_id != session_id_) {
        return {false, "Brake command requires the current nonzero brake-test session_id."};
    }
    if (!std::isfinite(brake_percent) || brake_percent < 0.0 ||
        brake_percent > config_.max_brake_percent)
    {
        return {false, "Brake percentage is outside the configured finite range."};
    }
    if (state_ != State::Active) {
        state_ = State::Active;
        active_started_ = now;
    }
    command_received_ = true;
    command_received_at_ = now;
    brake_percent_ = brake_percent;
    reason_ = "Stationary brake-response test active; a 0x503 valid command is requested.";
    build_intent();
    return {true, reason_};
}

void BrakeTestController::tick(const Feedback& feedback, TimePoint now)
{
    if (state_ == State::Disconnected) return;
    if (!config_.tx_enabled) {
        clear_command();
        reason_ = "Brake-test transmission is disabled by configuration.";
        build_intent();
        return;
    }
    if (state_ == State::Releasing) {
        if (now - release_started_ < config_.release_duration) {
            build_intent();
            return;
        }
        state_ = State::Idle;
        clear_command();
    }
    if (!allowed_ || session_exhausted_) {
        clear_command();
        reason_ = session_exhausted_ ? "Brake-test session identifiers exhausted; restart required." :
            "Brake test is paused; no brake-test command is active.";
        build_intent();
        return;
    }
    const auto problem = feedback_problem(feedback);
    if (!problem.empty()) {
        if (state_ == State::Active) {
            begin_release(now, problem);
        } else {
            // Retire a previous readiness window even if no command reached it.
            // A delayed command must not become eligible after A is reselected.
            if (stationary_ || ready_) renew_session();
            clear_command();
            reason_ = problem;
            build_intent();
        }
        return;
    }
    if (!stationary_) {
        stationary_ = true;
        stationary_since_ = now;
    }
    ready_ = now - stationary_since_ >= config_.stopped_dwell;
    if (state_ == State::Active) {
        if (now - active_started_ >= config_.max_test_duration) {
            begin_release(now, "Maximum brake-test duration reached.");
            return;
        }
        if (now - command_received_at_ >= config_.command_timeout) {
            begin_release(now, "Brake-test command timed out.");
            return;
        }
    } else {
        reason_ = ready_ ? "Ready for a fresh brake-test command; no 0x503 command is active." :
            "Waiting for continuously valid stationary brake-test feedback.";
    }
    build_intent();
}

void BrakeTestController::on_tx_failure(const std::string& reason, TimePoint now)
{
    if (state_ == State::Disconnected || !config_.tx_enabled) return;
    // Repeated failures during invalidation neither renew the session again nor
    // move the fixed deadline. A 0x501 success never reaches this controller.
    begin_release(now, "Brake test interrupted by TX failure: " + reason + ".");
}

void BrakeTestController::cancel(TimePoint now)
{
    if (state_ == State::Disconnected || state_ == State::Releasing) return;
    if (state_ == State::Active) {
        begin_release(now, "Brake test was cancelled.");
    } else {
        clear_command();
        renew_session();
        reason_ = "Brake test was cancelled; a fresh command in the new session is required.";
        build_intent();
    }
}

void BrakeTestController::set_allowed(bool allowed, const Feedback& feedback, TimePoint now)
{
    if (state_ == State::Disconnected) return;
    const bool was_allowed = allowed_;
    allowed_ = allowed;
    if (!allowed && was_allowed) cancel(now);
    // Feedback cannot restart a previous command. During invalidation this
    // merely advances the existing release phase, regardless of allowed_.
    tick(feedback, now);
}

void BrakeTestController::on_disconnect()
{
    clear_command();
    state_ = State::Disconnected;
    reason_ = "Brake test disconnected; restart after restoring the CAN receive connection.";
    intent_ = Intent{};
}

double BrakeTestController::command_age_sec(TimePoint now) const noexcept
{
    return command_received_ ? std::chrono::duration<double>(now - command_received_at_).count() :
        std::numeric_limits<double>::quiet_NaN();
}

void BrakeTestController::build_intent()
{
    intent_ = Intent{};
    if (!config_.tx_enabled || state_ == State::Disconnected) return;
    if (state_ == State::Releasing) {
        intent_.transmit = true;
    } else if (state_ == State::Active && allowed_ && command_received_) {
        intent_.transmit = true;
        intent_.command.valid = true;
        intent_.command.brake_percent = brake_percent_;
    }
}

}  // namespace withus_driver
