#include "withus_driver/control_manager.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace withus_driver
{
namespace
{
// Encoder rounding to the nearest 0.1 degree can put feedback half a step
// beyond the configured target bound. This allowance never expands protocol limits.
constexpr double kHalfSteeringStepRad = 3.14159265358979323846 / 3600.0;

bool valid_gear(Gear gear)
{
    return static_cast<unsigned>(gear) <= static_cast<unsigned>(Gear::Reverse);
}
}  // namespace

ControlManager::ControlManager(const Config& config, std::uint64_t session_seed)
: config_(config), session_id_(session_seed)
{
    const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
    if (!positive(config_.max_forward_mps) || !positive(config_.max_reverse_mps) ||
        config_.max_forward_mps > kMaxCommandSpeedMps ||
        config_.max_reverse_mps > kMaxCommandSpeedMps ||
        !positive(config_.max_steering_rad) || config_.max_steering_rad > kMaxCommandSteeringRad ||
        !positive(config_.stop_brake_percent) || config_.stop_brake_percent > 100.0 ||
        !positive(config_.stopped_speed_mps) ||
        config_.command_timeout.count() <= 0 || config_.body_timeout.count() <= 0 ||
        config_.request_timeout.count() <= 0 || config_.stop_timeout.count() <= 0 ||
        config_.release_timeout.count() <= 0 || config_.stopped_dwell.count() <= 0 ||
        config_.stopped_dwell >= config_.stop_timeout || session_seed == 0 ||
        (config_.policy != Policy::VcanReference && config_.policy != Policy::HardwareDrive))
    {
        throw std::invalid_argument("Invalid control manager configuration.");
    }
    if (config_.policy == Policy::HardwareDrive) {
        reason_ = config_.tx_enabled ?
            "Continuous 0x501 request=1; vehicle feedback is observation only, without an AD ownership claim." :
            "Transmission is disabled by configuration.";
        build_hardware_request_intent();
    }
}

bool ControlManager::acquisition_angle_allowed(double angle_rad) const noexcept
{
    return std::isfinite(angle_rad) && std::abs(angle_rad) <= kMaxCommandSteeringRad &&
           std::abs(angle_rad) <= config_.max_steering_rad + kHalfSteeringStepRad;
}

bool ControlManager::healthy(const Feedback& feedback) const noexcept
{
    return feedback.all_fresh && !feedback.blocked && valid_gear(feedback.gear) &&
           std::isfinite(feedback.speed_mps) && std::isfinite(feedback.steering_angle_rad) &&
           std::abs(feedback.steering_angle_rad) <= kMaxCommandSteeringRad;
}

bool ControlManager::stopped(const Feedback& feedback) const noexcept
{
    return healthy(feedback) && std::abs(feedback.speed_mps) <= config_.stopped_speed_mps;
}

void ControlManager::update_stationary(const Feedback& feedback, TimePoint now)
{
    if (!stopped(feedback)) {
        stationary_ = false;
    } else if (!stationary_) {
        stationary_ = true;
        stationary_since_ = now;
    }
}

bool ControlManager::stationary_for_dwell(TimePoint now) const noexcept
{
    return stationary_ && now - stationary_since_ >= config_.stopped_dwell;
}

void ControlManager::clear_commands()
{
    target_ = DriveTarget{};
    command_received_once_ = false;
    body_received_once_ = false;
    gear_pending_ = false;
    const bool headlight = body_.headlight_on;
    body_ = BodyCommand{};
    body_.headlight_on = headlight;
}

void ControlManager::fail(const std::string& reason)
{
    state_ = State::Fault;
    reason_ = reason;
    clear_commands();
    intent_ = TxIntent{};
}

void ControlManager::on_tx_failure(const std::string& reason, TimePoint)
{
    if (config_.policy == Policy::HardwareDrive) {
        if (state_ == State::Disconnected) {
            return;
        }
        hardware_request_tx_failed_ = true;
        if (state_ == State::Stopping) {
            // Retry the same zero request without moving the release deadline.
            hardware_release_sent_ = false;
            reason_ = "CAN transmission failed: " + reason +
                ". Retrying request=0 until the existing release deadline.";
        } else {
            state_ = State::Fault;
            reason_ = "CAN transmission failed: " + reason +
                ". The configured 0x501 request remains unchanged and will be retried.";
        }
        build_hardware_request_intent();
        return;
    }
    // No claim of a successful stop is possible after a failed CAN write.
    fail("CAN transmission failed: " + reason);
}

void ControlManager::on_disconnect()
{
    clear_commands();
    intent_ = TxIntent{};
    state_ = State::Disconnected;
    reason_ = "CAN receive loop stopped; restart the node after restoring the interface.";
}

ControlManager::Result ControlManager::set_enabled(
    bool enable, const Feedback& feedback, TimePoint now)
{
    if (config_.policy == Policy::HardwareDrive) {
        return set_hardware_request_enabled(enable, feedback, now);
    }
    update_stationary(feedback, now);
    if (!enable) {
        if (state_ == State::Monitoring) {
            return {true, "Already monitoring."};
        }
        if (state_ == State::Stopping) {
            return {true, "Stop/release already in progress; its deadline is unchanged."};
        }
        if (state_ == State::Disconnected) {
            return {false, "CAN receive loop is disconnected; restart the node."};
        }
        if (state_ == State::Fault) {
            if (!stopped(feedback)) {
                return {false, "Fault reset requires fresh, nonblocked, stopped feedback."};
            }
            if (!feedback.ad_ready) {
                clear_commands();
                state_ = State::Monitoring;
                reason_ = "Fault reset explicitly; a new enable request is required.";
                intent_ = TxIntent{};
                return {true, reason_};
            }
        }
        if (!healthy(feedback)) {
            fail("Cannot perform a verified stop/release without healthy fresh feedback.");
            return {false, reason_};
        }
        begin_stopping(feedback, now, "Explicit disable requested.");
        build_intent(feedback, now);
        return {true, reason_};
    }

    if (!config_.tx_enabled) {
        return {false, "Transmission is disabled by configuration."};
    }
    if (state_ != State::Monitoring) {
        return {false, "Enable requires MONITORING; disable/reset the current session first."};
    }
    if (!acquisition_angle_allowed(feedback.steering_angle_rad))
    {
        return {false, "Enable requires measured steering within the configured command angle limit."};
    }
    if (!stopped(feedback)) {
        return {false, "Enable requires fresh, nonblocked feedback and a stopped vehicle."};
    }
    if (feedback.ad_ready) {
        return {false, "AD is already active before this session; ownership is ambiguous."};
    }
    if (session_id_ == std::numeric_limits<std::uint64_t>::max()) {
        fail("Session identifier exhausted; restart the node.");
        return {false, reason_};
    }
    clear_commands();
    ++session_id_;
    state_ = State::Requesting;
    phase_started_ = now;
    phase_sequence_ = feedback.status_303_sequence;
    held_steering_rad_ = feedback.steering_angle_rad;
    target_.gear = feedback.gear;
    reason_ = "Waiting for fresh AD feedback after the control request.";
    build_intent(feedback, now);
    return {true, reason_};
}

ControlManager::Result ControlManager::accept_drive(
    const DriveTarget& target, const Feedback& feedback, TimePoint now)
{
    if (config_.policy != Policy::VcanReference) {
        return {false, "Hardware actuator commands are handled by HardwareDriveController."};
    }
    if (state_ != State::Active || target.session_id != session_id_) {
        return {false, "Drive command requires ACTIVE and the current session_id."};
    }
    if (!healthy(feedback) || !feedback.ad_ready) {
        fail("Drive command rejected because feedback or AD acknowledgement is invalid.");
        return {false, reason_};
    }
    if (!valid_gear(target.gear) || !std::isfinite(target.speed_mps) ||
        !std::isfinite(target.steering_angle_rad) || !std::isfinite(target.brake_percent) ||
        target.speed_mps < 0.0 || target.brake_percent < 0.0 || target.brake_percent > 100.0 ||
        std::abs(target.steering_angle_rad) > config_.max_steering_rad ||
        target.speed_mps > (target.gear == Gear::Reverse ?
        config_.max_reverse_mps : config_.max_forward_mps) ||
        ((target.gear == Gear::Park || target.gear == Gear::Neutral) && target.speed_mps != 0.0) ||
        (target.speed_mps > 0.0 && target.brake_percent > 0.0))
    {
        return {false, "Command is outside limits or combines nonzero speed with brake/P/N."};
    }
    update_stationary(feedback, now);
    if (target.gear != feedback.gear && !stationary_for_dwell(now)) {
        return {false, "Stop and hold standstill before changing gear."};
    }
    if (target.gear != feedback.gear) {
        if (!gear_pending_ || target.gear != target_.gear) {
            gear_requested_at_ = now;
        }
        gear_pending_ = true;
    } else {
        gear_pending_ = false;
    }
    target_ = target;
    command_received_ = now;
    command_received_once_ = true;
    build_intent(feedback, now);
    return {true, "Drive command accepted."};
}

ControlManager::Result ControlManager::accept_body(
    std::uint64_t session, const BodyCommand& body, TimePoint now)
{
    if (config_.policy != Policy::VcanReference) {
        return {false, "Body commands are unavailable in hardware_drive."};
    }
    if (state_ != State::Active || session != session_id_) {
        return {false, "Body command requires ACTIVE and the current session_id."};
    }
    body_ = body;
    body_received_ = now;
    body_received_once_ = true;
    intent_.body = body_;
    intent_.body.brake_light_on = body_.brake_light_on || intent_.brake.brake_percent > 0.0;
    return {true, "Body command accepted."};
}

void ControlManager::begin_stopping(
    const Feedback& feedback, TimePoint now, const std::string& reason)
{
    stopping_gear_ = feedback.gear;
    // Preserve a representable measured angle even outside the normal target limit.
    // Clamping here would turn a hold command into a steering movement.
    held_steering_rad_ = feedback.steering_angle_rad;
    clear_commands();
    state_ = State::Stopping;
    reason_ = reason;
    releasing_ = !feedback.ad_ready && stopped(feedback);
    phase_started_ = now;
    phase_sequence_ = feedback.status_303_sequence;
    // Require a complete new stopped dwell during STOPPING.
    stationary_ = stopped(feedback);
    stationary_since_ = now;
}

void ControlManager::tick(const Feedback& feedback, TimePoint now)
{
    if (config_.policy == Policy::HardwareDrive) {
        tick_hardware_request(feedback, now);
        return;
    }
    update_stationary(feedback, now);
    if (state_ == State::Monitoring || state_ == State::Fault || state_ == State::Disconnected) {
        intent_ = TxIntent{};
        return;
    }
    if (!healthy(feedback)) {
        fail("Feedback is stale, invalid, or reports a blocking vehicle condition.");
        return;
    }
    if (state_ == State::Requesting) {
        if (now - phase_started_ >= config_.request_timeout) {
            fail("Control request acknowledgement timed out.");
            return;
        }
        if (!stopped(feedback)) {
            fail("Vehicle moved while control ownership was being requested.");
            return;
        }
        if (feedback.ad_ready && feedback.status_303_sequence > phase_sequence_) {
            if (!acquisition_angle_allowed(feedback.steering_angle_rad)) {
                fail("Measured steering left the configured angle limit during the control request.");
                return;
            }
            held_steering_rad_ = feedback.steering_angle_rad;
            state_ = State::Active;
            active_started_ = now;
            reason_ = "Reference AD acknowledgement received; waiting for a current command.";
        }
    }
    if (state_ == State::Active) {
        if (!feedback.ad_ready) {
            fail("AD acknowledgement was lost during ACTIVE control.");
            return;
        }
        if (gear_pending_) {
            if (feedback.gear == target_.gear) {
                gear_pending_ = false;
            } else if (!stopped(feedback) || now - gear_requested_at_ >= config_.request_timeout) {
                fail("Gear acknowledgement timed out or vehicle moved before gear acknowledgement.");
                return;
            }
        } else if (command_received_once_ && feedback.gear != target_.gear) {
            fail("Feedback gear changed without a corresponding command.");
            return;
        }
        const TimePoint reference = command_received_once_ ? command_received_ : active_started_;
        if (now - reference >= config_.command_timeout) {
            begin_stopping(feedback, now, "Drive command timed out; stopping and releasing control.");
        }
    }
    if (state_ == State::Stopping) {
        if (releasing_) {
            if (now - phase_started_ >= config_.release_timeout) {
                fail("Control release acknowledgement timed out.");
                return;
            }
            if (!stopped(feedback)) {
                fail("Vehicle moved during control release.");
                return;
            }
            if (!feedback.ad_ready && feedback.status_303_sequence > phase_sequence_) {
                state_ = State::Monitoring;
                reason_ = "Stopped and release acknowledged; a new enable request is required.";
                intent_ = TxIntent{};
                return;
            }
        } else {
            if (now - phase_started_ >= config_.stop_timeout) {
                fail("Vehicle did not reach verified standstill before the stop deadline.");
                return;
            }
            if (!feedback.ad_ready && !stopped(feedback)) {
                fail("AD acknowledgement was lost before standstill.");
                return;
            }
            if (stationary_for_dwell(now) || !feedback.ad_ready) {
                releasing_ = true;
                phase_started_ = now;
                phase_sequence_ = feedback.status_303_sequence;
                reason_ = "Standstill held; waiting for fresh control release acknowledgement.";
            }
        }
    }
    build_intent(feedback, now);
}

void ControlManager::build_intent(const Feedback& feedback, TimePoint now)
{
    intent_ = TxIntent{};
    if (!config_.tx_enabled || state_ == State::Monitoring || state_ == State::Fault ||
        state_ == State::Disconnected)
    {
        return;
    }
    if (body_received_once_ && now - body_received_ >= config_.body_timeout) {
        const bool headlight = body_.headlight_on;
        body_ = BodyCommand{};
        body_.headlight_on = headlight;
        body_received_once_ = false;
    }
    intent_.transmit = true;
    intent_.body = body_;
    intent_.request.request_control = !(state_ == State::Stopping && releasing_);
    if (state_ == State::Requesting || (state_ == State::Stopping && releasing_)) {
        return;
    }
    intent_.steering.valid = true;
    intent_.brake.valid = true;
    intent_.speed.valid = true;
    if (state_ == State::Stopping || !command_received_once_) {
        intent_.steering.steering_angle_rad = held_steering_rad_;
        intent_.brake.brake_percent = config_.stop_brake_percent;
        intent_.speed.gear = state_ == State::Stopping ? stopping_gear_ : feedback.gear;
    } else {
        intent_.steering.steering_angle_rad = target_.steering_angle_rad;
        intent_.speed.gear = target_.gear;
        // Recheck feedback at each transmission, not only command acceptance.
        const bool gear_matches = feedback.gear == target_.gear && !gear_pending_;
        intent_.speed.speed_mps = gear_matches ? target_.speed_mps : 0.0;
        intent_.brake.brake_percent = gear_matches ? target_.brake_percent : config_.stop_brake_percent;
    }
    intent_.body.brake_light_on = body_.brake_light_on || intent_.brake.brake_percent > 0.0;
}

double ControlManager::command_age_sec(TimePoint now) const noexcept
{
    return command_received_once_ ? std::chrono::duration<double>(now - command_received_).count() :
           std::numeric_limits<double>::quiet_NaN();
}

void ControlManager::on_control_request_sent(
    bool request, const Feedback& feedback, TimePoint now)
{
    if (config_.policy == Policy::HardwareDrive) {
        if (state_ == State::Disconnected || !config_.tx_enabled) {
            return;
        }
        if (request && hardware_request_enabled_ && state_ != State::Stopping) {
            hardware_request_tx_failed_ = false;
            state_ = State::Monitoring;
            reason_ = "Continuous 0x501 request=1; vehicle feedback is observation only, without an AD ownership claim.";
        } else if (!request && state_ == State::Stopping && now >= phase_started_) {
            hardware_request_tx_failed_ = false;
            if (!hardware_release_sent_) {
                hardware_release_sent_ = true;
                phase_sequence_ = feedback.status_303_sequence;
            }
            reason_ = "Sending request=0 until the configured release deadline.";
        }
        build_hardware_request_intent();
        return;
    }
}

ControlManager::Result ControlManager::set_hardware_request_enabled(
    bool enable, const Feedback& feedback, TimePoint now)
{
    if (state_ == State::Disconnected) {
        return {false, "CAN receive loop is disconnected; restart the node."};
    }
    if (enable) {
        if (!config_.tx_enabled) {
            return {false, "Transmission is disabled by configuration."};
        }
        if (state_ == State::Stopping) {
            return {false, "Wait for the current mode release deadline before enabling."};
        }
        hardware_request_enabled_ = true;
        tick_hardware_request(feedback, now);
        return {true, "Continuous 0x501 request=1 enabled; A and vehicle feedback are observation only."};
    }
    if (state_ == State::Stopping) {
        return {true, "Mode release already in progress; its deadline is unchanged."};
    }
    if (!hardware_request_enabled_ || !config_.tx_enabled) {
        hardware_request_enabled_ = false;
        build_hardware_request_intent();
        return {true, "Continuous mode requests are already disabled."};
    }
    hardware_request_enabled_ = false;
    clear_commands();
    state_ = State::Stopping;
    phase_started_ = now;
    hardware_release_sent_ = false;
    hardware_request_tx_failed_ = false;
    reason_ = "Sending request=0 until the configured release deadline.";
    build_hardware_request_intent();
    return {true, reason_};
}

void ControlManager::tick_hardware_request(const Feedback& feedback, TimePoint now)
{
    if (!config_.tx_enabled || state_ == State::Disconnected) {
        build_hardware_request_intent();
        return;
    }
    if (state_ == State::Stopping) {
        if (now - phase_started_ >= config_.release_timeout) {
            // This feedback is only a release result, never a prerequisite for
            // broadcasting request=1. No ownership or braking is inferred.
            const bool acknowledged = hardware_release_sent_ && feedback.all_fresh &&
                feedback.drive_mode == 0 && feedback.status_303_sequence > phase_sequence_;
            state_ = acknowledged ? State::Monitoring : State::Fault;
            reason_ = acknowledged ?
                "0x501 transmission stopped after the release interval; fresh remote mode was observed." :
                "0x501 transmission stopped after the release interval; remote mode was not confirmed after a successful request=0 write.";
        }
        build_hardware_request_intent();
        return;
    }
    if (hardware_request_enabled_ && !hardware_request_tx_failed_) {
        state_ = State::Monitoring;
        reason_ = "Continuous 0x501 request=1; vehicle feedback is observation only, without an AD ownership claim.";
    }
    build_hardware_request_intent();
}

void ControlManager::build_hardware_request_intent()
{
    intent_ = TxIntent{};
    if (!config_.tx_enabled || state_ == State::Disconnected) {
        return;
    }
    if (state_ == State::Stopping) {
        intent_.transmit = true;
        intent_.request.request_control = false;
    } else if (hardware_request_enabled_) {
        intent_.transmit = true;
        intent_.request.request_control = true;
    }
}

}  // namespace withus_driver
