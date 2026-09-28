#include "withus_driver/hardware_drive_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

using C = withus_driver::HardwareDriveController;
using G = withus_driver::Gear;
using namespace std::chrono_literals;

namespace
{
int checks = 0;
void check(bool value, const char* message)
{
    ++checks;
    if (!value) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
C::Config configuration()
{
    C::Config config;
    config.tx_enabled = true;
    return config;
}
C::Feedback healthy()
{
    C::Feedback feedback;
    feedback.all_fresh = true;
    feedback.remote_fresh = true;
    feedback.remote_known = true;
    feedback.remote_selected = true;
    feedback.gear = G::Neutral;
    feedback.drive_mode = 3;
    feedback.status_303_sequence = 1;
    return feedback;
}
struct Fixture
{
    C controller;
    C::Feedback feedback{healthy()};
    C::TimePoint now{10s};
    explicit Fixture(C::Config config = configuration(), std::uint64_t seed = 77)
    : controller(config, seed) {}

    void tick(std::chrono::milliseconds elapsed = 1ms)
    {
        now += elapsed;
        controller.tick(feedback, now);
    }
    C::DriveTarget target(G gear = G::Neutral, double speed = 0.0,
                          double steering = 0.0, double brake = 0.0) const
    {
        return {controller.session_id(), gear, speed, steering, brake};
    }
    bool accept(const C::DriveTarget& value)
    {
        return controller.accept(value, feedback, now).accepted;
    }
    void prepare()
    {
        controller.tick(feedback, now);
        check(!controller.ready() && !controller.intent().transmit, "initial dwell has no drive TX");
        tick(199ms);
        check(!controller.ready(), "readiness requires the complete dwell");
        tick(1ms);
        check(controller.ready() && !controller.intent().transmit, "ready does not send a command");
    }
    void start()
    {
        prepare();
        check(accept(target(feedback.gear)), "zero-speed same-gear bootstrap accepted");
        check(controller.active() && controller.ready(), "bootstrap is active and accepts updates");
        check(!controller.mode_confirmed(), "mode3 bootstrap is not full AD confirmation");
    }
    void confirm_mode()
    {
        feedback.drive_mode = 1;
        feedback.ad_ready = true;
        ++feedback.status_303_sequence;
        tick();
        check(!controller.mode_confirmed(), "mode1 before a complete batch cannot confirm");
        controller.on_commands_sent(feedback, now);
        tick();
        check(!controller.mode_confirmed(), "same snapshot as the completed batch cannot confirm");
        ++feedback.status_303_sequence;
        tick();
        check(controller.mode_confirmed(), "later303 mode1 and AD signal confirm full mode");
    }
    void invalid()
    {
        check(controller.invalidating() && !controller.ready() && !controller.active(), "drive invalidates");
        const auto& intent = controller.intent();
        check(intent.transmit && !intent.steering.valid && !intent.brake.valid && !intent.speed.valid,
              "all three commands invalidated together");
        check(intent.steering.steering_angle_rad == 0.0 && intent.brake.brake_percent == 0.0 &&
              intent.speed.speed_mps == 0.0 && intent.speed.gear == G::Neutral,
              "invalid payload contains no invented brake target");
        check(std::isnan(controller.command_age_sec(now)), "discarded command has no age");
        check(!controller.mode_confirmed() && !controller.gear_pending(), "all acquisition state cleared");
    }
};

void break_feedback(C::Feedback& feedback, int kind)
{
    switch (kind) {
        case 0: feedback.all_fresh = false; break;
        case 1: feedback.remote_fresh = false; break;
        case 2: feedback.remote_known = false; break;
        case 3: feedback.remote_selected = false; break;
        case 4: feedback.blocked = true; break;
        case 5: feedback.gear = static_cast<G>(4); break;
        case 6: feedback.speed_mps = std::numeric_limits<double>::quiet_NaN(); break;
        case 7: feedback.speed_mps = withus_driver::kMaxCommandSpeedMps + 1; break;
        case 8: feedback.steering_angle_rad = std::numeric_limits<double>::infinity(); break;
        case 9: feedback.steering_angle_rad = -withus_driver::kMaxCommandSteeringRad - 0.01; break;
        case 10: feedback.drive_mode = 0; break;
        case 11: feedback.drive_mode = 2; break;
        case 12: feedback.drive_mode = 15; break;
    }
}
}

int main()
{
    {
        Fixture f;
        f.prepare();
        check(!f.accept(f.target(G::Drive, 0.2)), "first moving command rejected");
        check(!f.accept(f.target(G::Drive)), "first gear change rejected");
        auto old = f.target();
        const auto session = f.controller.session_id();
        f.feedback.remote_selected = false;
        f.tick();
        check(f.controller.session_id() == session + 1, "unused readiness session retired on A release");
        f.tick();
        check(f.controller.session_id() == session + 1, "prolonged A release retires session only once");
        f.feedback.remote_selected = true;
        f.prepare();
        check(!f.accept(old), "old command cannot return following A reselection");
        check(f.accept(f.target()), "fresh session zero command accepted");
    }
    {
        Fixture f;
        f.start();
        check(!f.accept(f.target(G::Drive)), "mode3 cannot shift gears");
        f.confirm_mode();
        check(f.accept(f.target(G::Drive, 0.0, 0.2, 37.0)), "client stationary gear request accepted");
        check(f.controller.gear_pending(), "gear awaits acknowledgement");
        check(f.controller.intent().brake.brake_percent == 37.0 &&
              f.controller.intent().steering.steering_angle_rad == 0.2, "client brake and steering preserved");
        check(!f.accept(f.target(G::Drive, 0.5)), "speed rejected before requested gear observed");
        check(!f.accept(f.target(G::Reverse)), "pending gear target cannot be replaced");
        f.feedback.gear = G::Drive;
        ++f.feedback.status_303_sequence;
        f.tick();
        check(f.controller.gear_pending(), "gear feedback before batch does not acknowledge");
        f.controller.on_commands_sent(f.feedback, f.now);
        f.tick();
        check(f.controller.gear_pending(), "gear feedback equal to batch baseline cannot acknowledge");
        ++f.feedback.status_303_sequence;
        f.tick();
        check(!f.controller.gear_pending(), "fresh post-batch requested gear acknowledged");
        check(f.controller.intent().speed.speed_mps == 0.0, "rejected nonzero target was never cached");
        check(!f.accept(f.target(G::Drive, 0.2, 0.0, 10.0)), "simultaneous speed and brake command rejected");
        check(f.controller.intent().speed.speed_mps == 0.0, "conflicting target is not cached");
        check(f.accept(f.target(G::Drive, 1.0, -0.3, 0.0)), "configured speed and steering boundaries accepted");
        f.feedback.speed_mps = 0.5;
        f.tick();
        check(f.controller.active() && f.controller.intent().speed.speed_mps == 1.0, "confirmed drive permits motion");
        check(!f.accept(f.target(G::Reverse)), "gear change rejected while moving");
        check(f.controller.intent().speed.speed_mps == 1.0, "rejection leaves latest accepted target intact");
        f.feedback.speed_mps = 0.0;
        f.tick();
        check(!f.accept(f.target(G::Reverse)), "gear change also requires new stationary dwell");
        check(f.accept(f.target(G::Drive)), "client zero-speed command accepted while dwell starts");
        f.tick(100ms);
        check(f.accept(f.target(G::Drive)), "client holds zero speed through dwell");
        f.tick(100ms);
        check(f.accept(f.target(G::Reverse, 0.0, 0.0, 100.0)), "100 percent client brake is allowed at a stopped shift");
        f.controller.on_commands_sent(f.feedback, f.now);
        f.feedback.gear = G::Reverse;
        ++f.feedback.status_303_sequence;
        f.tick();
        check(f.accept(f.target(G::Reverse, 0.5)), "reverse maximum accepted after fresh gear match");
        check(!f.accept(f.target(G::Reverse, 0.50001)), "reverse above configured limit rejected");
    }
    {
        Fixture f;
        f.start();
        f.controller.on_commands_sent(f.feedback, f.now);
        f.feedback.drive_mode = 1;
        ++f.feedback.status_303_sequence;
        f.tick();
        check(!f.controller.mode_confirmed(), "mode1 without AD signal is insufficient");
        f.feedback.ad_ready = true;
        f.tick();
        check(f.controller.mode_confirmed(), "fresh post-batch303 plus current AD signal confirms");
        f.feedback.drive_mode = 3;
        f.tick();
        f.invalid();
    }
    for (int failure = 0; failure < 13; ++failure) {
        Fixture initial;
        break_feedback(initial.feedback, failure);
        initial.controller.tick(initial.feedback, initial.now);
        initial.tick(1s);
        check(!initial.controller.ready() && !initial.controller.intent().transmit, "invalid initial feedback prevents drive");
        Fixture active;
        active.start();
        const auto session = active.controller.session_id();
        break_feedback(active.feedback, failure);
        active.tick();
        active.invalid();
        check(active.controller.session_id() == session + 1, "active feedback failure renews session once");
    }
    {
        Fixture f;
        f.start();
        const auto old = f.target();
        f.tick(199ms);
        check(f.controller.active(), "command remains valid before exact deadline");
        f.now += 1ms;
        check(!f.accept(old), "command arriving at timeout cannot renew expired target");
        f.invalid();
        const auto session = f.controller.session_id();
        f.tick(999ms);
        f.controller.on_tx_failure("would block during invalidation", f.now);
        check(f.controller.session_id() == session, "invalidation failures do not keep renewing session");
        f.tick(1ms);
        check(!f.controller.invalidating() && !f.controller.intent().transmit && !f.controller.ready(),
              "invalidation ceases at fixed deadline even after a TX failure");
        f.tick(199ms);
        check(!f.controller.ready(), "readiness dwell restarts after invalidation");
        f.tick(1ms);
        check(f.controller.ready() && !f.accept(old), "only new-session commands eligible after invalidation");
    }
    {
        Fixture f;
        f.start();
        const auto began = f.now;
        f.controller.on_commands_sent(f.feedback, f.now);
        for (int i = 0; i < 9; ++i) {
            f.tick(100ms);
            check(f.accept(f.target()), "zero updates accepted while waiting for full AD mode");
        }
        f.now = began + 1000ms;
        check(!f.accept(f.target()), "updates cannot extend fixed mode-confirmation deadline");
        f.invalid();
    }
    {
        Fixture f;
        f.start();
        f.confirm_mode();
        check(f.accept(f.target(G::Drive)), "start bounded gear request");
        const auto began = f.now;
        f.controller.on_commands_sent(f.feedback, f.now);
        for (int i = 0; i < 9; ++i) {
            f.tick(100ms);
            check(f.accept(f.target(G::Drive)), "zero updates keep command fresh during gear wait");
        }
        f.now = began + 1000ms;
        check(!f.accept(f.target(G::Drive)), "updates cannot extend fixed gear deadline");
        f.invalid();
    }
    {
        Fixture f;
        f.start();
        f.controller.on_tx_failure("partial successful batch", f.now);
        f.invalid();
        f.feedback.drive_mode = 1;
        f.feedback.ad_ready = true;
        ++f.feedback.status_303_sequence;
        f.controller.on_commands_sent(f.feedback, f.now);
        check(!f.controller.mode_confirmed(), "late success after partial failure cannot reactivate commands");
    }
    {
        Fixture f;
        f.start();
        f.controller.set_allowed(false, f.feedback, f.now);
        f.invalid();
        const auto session = f.controller.session_id();
        f.controller.set_allowed(true, f.feedback, f.now);
        f.tick(1000ms);
        check(!f.controller.intent().transmit, "re-enabling does not replay old command");
        f.tick(200ms);
        check(f.controller.ready() && f.controller.session_id() == session, "fresh readiness after explicit pause");
        check(f.accept(f.target()), "new zero command after pause accepted");
        f.controller.on_disconnect();
        check(f.controller.state() == C::State::Disconnected && !f.controller.intent().transmit, "disconnect stops output");
        f.controller.set_allowed(true, f.feedback, f.now);
        f.controller.on_commands_sent(f.feedback, f.now);
        f.controller.on_tx_failure("late callback", f.now);
        f.tick(10s);
        check(!f.controller.ready() && !f.accept(f.target()), "disconnection requires restart");
    }
    {
        Fixture f;
        f.feedback.gear = G::Drive;
        f.start();
        check(!f.accept(f.target(G::Drive, 0.1)), "mode3 cannot command speed even when gear already matches");
        f.feedback.speed_mps = 0.051;
        f.tick();
        f.invalid();
    }
    {
        Fixture f;
        f.start();
        f.confirm_mode();
        f.feedback.gear = G::Drive;
        f.tick();
        f.invalid();
    }
    {
        Fixture f;
        f.start();
        f.confirm_mode();
        check(f.accept(f.target(G::Drive)), "pending gear preparation");
        f.feedback.gear = G::Reverse;
        f.tick();
        f.invalid();
    }
    {
        Fixture f;
        f.prepare();
        for (int kind = 0; kind < 9; ++kind) {
            auto target = f.target();
            switch (kind) {
                case 0: target.speed_mps = -0.1; break;
                case 1: target.speed_mps = std::numeric_limits<double>::quiet_NaN(); break;
                case 2: target.steering_angle_rad = 0.30001; break;
                case 3: target.steering_angle_rad = std::numeric_limits<double>::infinity(); break;
                case 4: target.brake_percent = -0.01; break;
                case 5: target.brake_percent = 100.01; break;
                case 6: target.brake_percent = std::numeric_limits<double>::quiet_NaN(); break;
                case 7: target.gear = static_cast<G>(255); break;
                case 8: target.session_id = 0; break;
            }
            check(!f.accept(target), "malformed target rejected");
            check(!f.controller.active() && !f.controller.intent().transmit, "rejected target causes no TX");
        }
        check(f.accept(f.target()), "valid target still accepted after malformed attempts");
        f.tick(199ms);
        auto bad = f.target();
        bad.brake_percent = -1;
        check(!f.accept(bad), "invalid update rejected");
        f.tick(1ms);
        f.invalid();
    }
    {
        Fixture f(configuration(), std::numeric_limits<std::uint64_t>::max());
        f.start();
        f.controller.on_tx_failure("partial TX", f.now);
        f.invalid();
        f.tick(2s);
        f.tick(1s);
        check(!f.controller.ready() && !f.controller.intent().transmit, "exhausted session never wraps or rearms");
    }
    {
        auto config = configuration();
        config.tx_enabled = false;
        Fixture f(config);
        f.controller.tick(f.feedback, f.now);
        f.tick(1s);
        check(!f.accept(f.target()) && !f.controller.intent().transmit, "TX configuration is authoritative");
    }
    for (int kind = 0; kind < 12; ++kind) {
        auto config = configuration();
        std::uint64_t seed = 77;
        switch (kind) {
            case 0: seed = 0; break;
            case 1: config.command_timeout = 0ms; break;
            case 2: config.invalidation_duration = -1ms; break;
            case 3: config.stopped_dwell = 0ms; break;
            case 4: config.mode_request_timeout = 0ms; break;
            case 5: config.gear_timeout = 0ms; break;
            case 6: config.stopped_threshold_mps = std::numeric_limits<double>::quiet_NaN(); break;
            case 7: config.max_forward_mps = 0; break;
            case 8: config.max_reverse_mps = withus_driver::kMaxCommandSpeedMps + 1; break;
            case 9: config.max_steering_rad = withus_driver::kMaxCommandSteeringRad + 0.01; break;
            case 10: config.max_forward_mps = std::numeric_limits<double>::infinity(); break;
            case 11: config.max_steering_rad = -0.1; break;
        }
        bool threw = false;
        try { C controller(config, seed); } catch (const std::invalid_argument&) { threw = true; }
        check(threw, "invalid configuration rejected before any TX");
    }
    std::cout << "Hardware-drive controller: " << checks << " checks passed.\n";
}
