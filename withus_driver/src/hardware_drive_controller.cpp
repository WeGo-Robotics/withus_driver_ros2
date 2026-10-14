#include "withus_driver/hardware_drive_controller.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace withus_driver
{
namespace
{
bool valid_gear(Gear gear)
{
    return gear == Gear::Park || gear == Gear::Drive ||
        gear == Gear::Neutral || gear == Gear::Reverse;
}
}

HardwareDriveController::HardwareDriveController(const Config& config, std::uint64_t seed)
: config_(config), session_id_(seed)
{
    if (seed == 0 || config.command_timeout.count() <= 0 ||
        config.invalidation_duration.count() <= 0 || config.stopped_dwell.count() <= 0 ||
        config.mode_request_timeout.count() <= 0 || config.gear_timeout.count() <= 0 ||
        !std::isfinite(config.stopped_threshold_mps) || config.stopped_threshold_mps <= 0.0 ||
        config.stopped_threshold_mps > kMaxCommandSpeedMps ||
        !std::isfinite(config.max_forward_mps) || config.max_forward_mps <= 0.0 ||
        config.max_forward_mps > kMaxCommandSpeedMps ||
        !std::isfinite(config.max_reverse_mps) || config.max_reverse_mps <= 0.0 ||
        config.max_reverse_mps > kMaxCommandSpeedMps ||
        !std::isfinite(config.max_steering_rad) || config.max_steering_rad <= 0.0 ||
        config.max_steering_rad > kMaxCommandSteeringRad)
    {
        throw std::invalid_argument("Invalid hardware-drive controller configuration.");
    }
    if (!config_.tx_enabled) reason_ = "Hardware-drive transmission is disabled by configuration.";
}

std::string HardwareDriveController::feedback_problem(const Feedback& feedback) const
{
    if (!feedback.all_fresh) return "Vehicle feedback 0x301/303/304 is stale or missing.";
    if (!feedback.remote_fresh) return "A-switch feedback 0x10A is stale or missing.";
    if (!feedback.remote_known) return "A-switch value is unknown.";
    if (!feedback.remote_selected) return "A switch is not selected.";
    if (feedback.blocked) return "Vehicle feedback reports a blocking condition.";
    if (!valid_gear(feedback.gear)) return "Vehicle feedback reports an unknown gear.";
    if (!std::isfinite(feedback.speed_mps) || std::abs(feedback.speed_mps) > kMaxCommandSpeedMps)
        return "Vehicle speed feedback is outside the finite protocol range.";
    if (!std::isfinite(feedback.steering_angle_rad) ||
        std::abs(feedback.steering_angle_rad) > kMaxCommandSteeringRad)
        return "Vehicle steering feedback is outside the finite protocol range.";
    if (feedback.drive_mode != 1 && feedback.drive_mode != 3)
        return "Hardware drive requires observed drive_mode=1 or drive_mode=3.";
    return {};
}

std::string HardwareDriveController::target_problem(const DriveTarget& target) const
{
    if (!valid_gear(target.gear)) return "Drive command has an unknown gear.";
    if (!std::isfinite(target.speed_mps) || target.speed_mps < 0.0)
        return "Drive speed must be a finite nonnegative magnitude.";
    if ((target.gear == Gear::Park || target.gear == Gear::Neutral) && target.speed_mps != 0.0)
        return "Park and Neutral require a zero speed command.";
    if ((target.gear == Gear::Drive && target.speed_mps > config_.max_forward_mps) ||
        (target.gear == Gear::Reverse && target.speed_mps > config_.max_reverse_mps))
        return "Drive speed exceeds the configured gear-specific limit.";
    if (!std::isfinite(target.steering_angle_rad) ||
        std::abs(target.steering_angle_rad) > config_.max_steering_rad)
        return "Steering command exceeds the configured finite range.";
    if (!std::isfinite(target.brake_percent) || target.brake_percent < 0.0 ||
        target.brake_percent > 100.0)
        return "Brake command must be a finite percentage from 0 to 100.";
    if (target.speed_mps > 0.0 && target.brake_percent > 0.0)
        return "Nonzero speed and nonzero braking cannot be requested together.";
    return {};
}

bool HardwareDriveController::stopped(const Feedback& feedback) const noexcept
{
    return std::isfinite(feedback.speed_mps) &&
        std::abs(feedback.speed_mps) <= config_.stopped_threshold_mps;
}

bool HardwareDriveController::stationary_for_dwell(TimePoint now) const noexcept
{
    return stationary_ && now - stationary_since_ >= config_.stopped_dwell;
}

void HardwareDriveController::update_stationary(const Feedback& feedback, TimePoint now)
{
    if (!stopped(feedback)) {
        stationary_ = false;
    } else if (!stationary_) {
        stationary_ = true;
        stationary_since_ = now;
    }
}

void HardwareDriveController::clear_command()
{
    ready_ = false;
    stationary_ = false;
    command_received_ = false;
    target_ = DriveTarget{};
    mode_confirmed_ = false;
    mode_batch_sent_ = false;
    gear_pending_ = false;
    gear_batch_sent_ = false;
}

void HardwareDriveController::renew_session()
{
    if (session_id_ == std::numeric_limits<std::uint64_t>::max()) {
        session_exhausted_ = true;
    } else {
        ++session_id_;
    }
}

void HardwareDriveController::retire_readiness()
{
    // Even an unused readiness/dwell window must not accept delayed commands
    // following a transient A deselection or feedback fault.
    if (ready_ || stationary_) renew_session();
    clear_command();
}

void HardwareDriveController::begin_invalidation(TimePoint now, const std::string& reason)
{
    if (state_ == State::Disconnected || state_ == State::Invalidating) return;
    clear_command();
    renew_session();
    state_ = State::Invalidating;
    invalidation_started_at_ = now;
    reason_ = reason + " Sending invalid 0x502/503/504 for a bounded interval; a physical stop is not confirmed.";
    if (session_exhausted_) reason_ += " Session identifiers exhausted; restart required.";
    build_intent();
}

HardwareDriveController::Result HardwareDriveController::accept(
    const DriveTarget& target, const Feedback& feedback, TimePoint now)
{
    // Advance all deadlines before accepting an update; a command at the exact
    // deadline cannot revive the expired target or extend an acquisition phase.
    tick(feedback, now);
    if (!ready_) return {false, reason_};
    if (target.session_id == 0 || target.session_id != session_id_)
        return {false, "Drive command requires the current nonzero hardware-drive session_id."};
    const auto problem = target_problem(target);
    if (!problem.empty()) return {false, problem};

    if (state_ == State::Idle) {
        if (target.speed_mps != 0.0 || target.gear != feedback.gear)
            return {false, "The first command must have zero speed and preserve the observed gear."};
        state_ = State::Commanding;
        command_started_at_ = now;
        observed_gear_ = feedback.gear;
    } else {
        if (!mode_confirmed_ &&
            (target.speed_mps != 0.0 || target.gear != observed_gear_))
            return {false, "A fresh post-command mode=1 acknowledgement is required before driving or shifting."};
        if (gear_pending_) {
            if (target.speed_mps != 0.0 || target.gear != target_.gear)
                return {false, "Await the requested gear with zero-speed commands; a pending gear cannot be replaced."};
        } else if (target.gear != observed_gear_) {
            if (!mode_confirmed_ || target.speed_mps != 0.0 || !stationary_for_dwell(now))
                return {false, "A gear change requires confirmed AD mode and a zero-speed command after stationary dwell."};
            gear_pending_ = true;
            gear_batch_sent_ = false;
            gear_started_at_ = now;
        }
        if (target.speed_mps > 0.0 &&
            (!mode_confirmed_ || gear_pending_ || target.gear != feedback.gear))
            return {false, "Nonzero speed requires confirmed AD mode and matching observed Drive or Reverse gear."};
    }
    // No rejected command is cached or refreshes the command timeout.
    target_ = target;
    command_received_ = true;
    command_received_at_ = now;
    reason_ = gear_pending_ ? "Waiting for fresh requested-gear feedback; zero speed is commanded." :
        mode_confirmed_ ? "Transmitting the latest client drive command." :
        "Transmitting a stationary zero-speed command; awaiting fresh full AD-mode feedback.";
    build_intent();
    return {true, reason_};
}

void HardwareDriveController::tick(const Feedback& feedback, TimePoint now)
{
    if (state_ == State::Disconnected) return;
    if (!config_.tx_enabled) {
        clear_command();
        reason_ = "Hardware-drive transmission is disabled by configuration.";
        build_intent();
        return;
    }
    if (state_ == State::Invalidating) {
        if (now - invalidation_started_at_ < config_.invalidation_duration) {
            build_intent();
            return;
        }
        state_ = State::Idle;
        clear_command();
    }
    if (!allowed_ || session_exhausted_) {
        clear_command();
        reason_ = session_exhausted_ ? "Hardware-drive session identifiers exhausted; restart required." :
            "Hardware drive is paused; no drive command is active.";
        build_intent();
        return;
    }
    const auto problem = feedback_problem(feedback);
    if (!problem.empty()) {
        if (active()) begin_invalidation(now, problem);
        else {
            retire_readiness();
            reason_ = problem;
            build_intent();
        }
        return;
    }
    if (active()) {
        if (now - command_received_at_ >= config_.command_timeout) {
            begin_invalidation(now, "Drive command timed out.");
            return;
        }
        if (mode_confirmed_ && (feedback.drive_mode != 1 || !feedback.ad_ready)) {
            begin_invalidation(now, "Previously confirmed full AD mode was lost.");
            return;
        }
        if (!mode_confirmed_) {
            if (now - command_started_at_ >= config_.mode_request_timeout) {
                begin_invalidation(now, "Full AD-mode confirmation timed out.");
                return;
            }
            if (!stopped(feedback)) {
                begin_invalidation(now, "Vehicle moved before full AD mode was confirmed.");
                return;
            }
            if (mode_batch_sent_ && feedback.status_303_sequence > mode_sequence_ &&
                feedback.drive_mode == 1 && feedback.ad_ready) mode_confirmed_ = true;
        }
        if (gear_pending_) {
            if (now - gear_started_at_ >= config_.gear_timeout) {
                begin_invalidation(now, "Requested-gear confirmation timed out.");
                return;
            }
            if (feedback.gear != observed_gear_ && feedback.gear != target_.gear) {
                begin_invalidation(now, "An unexpected gear was observed during a requested shift.");
                return;
            }
            if (!stopped(feedback)) {
                begin_invalidation(now, "Vehicle moved while a gear change was pending.");
                return;
            }
            if (gear_batch_sent_ && feedback.status_303_sequence > gear_sequence_ &&
                feedback.gear == target_.gear) {
                observed_gear_ = feedback.gear;
                gear_pending_ = false;
                gear_batch_sent_ = false;
                // target_ still contains the accepted zero-speed gear request.
                // Feedback cannot activate a previously rejected speed target.
            }
        } else if (feedback.gear != observed_gear_) {
            begin_invalidation(now, "Observed gear changed without an accepted gear request.");
            return;
        }
        update_stationary(feedback, now);
        ready_ = true;
        reason_ = gear_pending_ ? "Waiting for fresh requested-gear feedback; zero speed is commanded." :
            mode_confirmed_ ? "Transmitting the latest client drive command." :
            "Transmitting a stationary zero-speed command; awaiting fresh full AD-mode feedback.";
    } else {
        if (!stopped(feedback)) {
            retire_readiness();
            reason_ = "A new drive session requires stationary vehicle feedback.";
        } else {
            update_stationary(feedback, now);
            ready_ = stationary_for_dwell(now);
            reason_ = ready_ ? "Ready for a fresh zero-speed command preserving the observed gear." :
                "Waiting for continuously valid stationary hardware-drive feedback.";
        }
    }
    build_intent();
}

void HardwareDriveController::on_commands_sent(const Feedback& feedback, TimePoint now)
{
    tick(feedback, now);
    if (!active()) return;
    if (!mode_batch_sent_) {
        mode_batch_sent_ = true;
        mode_sequence_ = feedback.status_303_sequence;
    }
    if (gear_pending_ && !gear_batch_sent_) {
        gear_batch_sent_ = true;
        gear_sequence_ = feedback.status_303_sequence;
    }
}

void HardwareDriveController::on_tx_failure(const std::string& reason, TimePoint now)
{
    if (state_ == State::Disconnected || !config_.tx_enabled || invalidating()) return;
    if (active()) {
        begin_invalidation(now, "Drive transmission failed: " + reason + ".");
    } else {
        retire_readiness();
        reason_ = "Drive transmission failed: " + reason + ". Waiting for a new readiness window.";
        build_intent();
    }
}

void HardwareDriveController::set_allowed(bool allowed, const Feedback& feedback, TimePoint now)
{
    if (state_ == State::Disconnected) return;
    const bool was_allowed = allowed_;
    allowed_ = allowed;
    if (!allowed && was_allowed && !invalidating()) {
        if (active()) begin_invalidation(now, "Hardware drive was paused or cancelled.");
        else {
            renew_session();
            clear_command();
        }
    }
    tick(feedback, now);
}

void HardwareDriveController::on_disconnect()
{
    if (state_ == State::Disconnected) return;
    renew_session();
    clear_command();
    state_ = State::Disconnected;
    reason_ = "Hardware drive disconnected; restart after restoring the CAN receive connection.";
    intent_ = Intent{};
}

double HardwareDriveController::command_age_sec(TimePoint now) const noexcept
{
    return command_received_ ? std::chrono::duration<double>(now - command_received_at_).count() :
        std::numeric_limits<double>::quiet_NaN();
}

void HardwareDriveController::build_intent()
{
    intent_ = Intent{};
    if (!config_.tx_enabled || state_ == State::Disconnected) return;
    if (invalidating()) {
        intent_.transmit = true;
    } else if (active() && allowed_ && command_received_) {
        intent_.transmit = true;
        intent_.steering = {true, target_.steering_angle_rad};
        intent_.brake = {true, target_.brake_percent};
        intent_.speed = {true, target_.gear, target_.speed_mps};
    }
}

}  // namespace withus_driver
