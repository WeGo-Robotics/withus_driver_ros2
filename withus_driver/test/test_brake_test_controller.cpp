#include "withus_driver/brake_test_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

using B = withus_driver::BrakeTestController;
using S = B::State;
using G = withus_driver::Gear;
using namespace std::chrono_literals;

namespace
{
int checks = 0;
void check(bool condition, const char* message)
{
    ++checks;
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
B::Config config(bool tx = true)
{
    B::Config result;
    result.tx_enabled = tx;
    return result;
}
B::Feedback healthy()
{
    B::Feedback feedback;
    feedback.all_fresh = true;
    feedback.remote_fresh = true;
    feedback.remote_known = true;
    feedback.remote_selected = true;
    feedback.gear = G::Neutral;
    feedback.drive_mode = 3;
    return feedback;
}
struct Fixture
{
    B controller{config(), 77};
    B::Feedback feedback{healthy()};
    B::TimePoint now{10s};

    void tick(std::chrono::milliseconds elapsed = 20ms)
    {
        now += elapsed;
        controller.tick(feedback, now);
    }
    void prepare()
    {
        controller.tick(feedback, now);
        check(!controller.ready() && !controller.intent().transmit, "dwell starts without 503");
        tick(199ms);
        check(!controller.ready(), "199 ms is not the full dwell");
        tick(1ms);
        check(controller.ready(), "continuous 200 ms dwell permits a new command");
    }
    void valid(double percent)
    {
        check(controller.active(), "test active");
        check(controller.ready(), "fresh active test accepts updates");
        check(controller.intent().transmit && controller.intent().command.valid, "valid 503 scheduled");
        check(controller.intent().command.brake_percent == percent, "only requested percentage used");
    }
    void invalid()
    {
        check(controller.releasing() && !controller.ready(), "bounded invalidation state");
        check(controller.intent().transmit && !controller.intent().command.valid,
              "503 invalid scheduled");
        check(controller.intent().command.brake_percent == 0.0, "invalid 503 has zero percentage");
        check(std::isnan(controller.command_age_sec(now)), "discarded command has no age");
    }
    void start(double percent = 25.0)
    {
        prepare();
        check(controller.accept(percent, controller.session_id(), feedback, now).accepted,
              "current session command accepted");
        valid(percent);
    }
};

void invalidate(B::Feedback& feedback, int failure)
{
    switch (failure) {
        case 0: feedback.all_fresh = false; break;
        case 1: feedback.remote_fresh = false; break;
        case 2: feedback.remote_known = false; break;
        case 3: feedback.remote_selected = false; break;
        case 4: feedback.blocked = true; break;
        case 5: feedback.gear = G::Drive; break;
        case 6: feedback.gear = G::Park; break;
        case 7: feedback.gear = G::Reverse; break;
        case 8: feedback.speed_mps = 0.051; break;
        case 9: feedback.speed_mps = -0.051; break;
        case 10: feedback.speed_mps = std::numeric_limits<double>::quiet_NaN(); break;
        case 11: feedback.speed_mps = std::numeric_limits<double>::infinity(); break;
        case 12: feedback.drive_mode = 0; break;
        case 13: feedback.drive_mode = 2; break;
        case 14: feedback.drive_mode = 255; break;
    }
}
}  // namespace

int main()
{
    {
        Fixture f;
        check(!f.controller.intent().transmit && !f.controller.ready(), "startup has no brake command");
        check(!f.controller.accept(10, 77, f.feedback, f.now).accepted, "command cannot bypass dwell");
        f.tick(100ms);
        f.feedback.remote_selected = false;
        f.tick();
        f.feedback.remote_selected = true;
        f.tick();
        f.tick(199ms);
        check(!f.controller.ready(), "interrupted dwell must restart");
        f.tick(1ms);
        check(f.controller.ready(), "new uninterrupted dwell completes");
        check(!f.controller.intent().transmit, "readiness alone never sends 503");
    }
    for (int failure = 0; failure < 15; ++failure) {
        Fixture f;
        invalidate(f.feedback, failure);
        f.controller.tick(f.feedback, f.now);
        f.tick(5s);
        check(!f.controller.ready() && !f.controller.intent().transmit, "invalid initial feedback cannot arm");
        check(!f.controller.accept(10, 77, f.feedback, f.now).accepted, "invalid initial feedback rejects commands");
    }
    {
        Fixture f;
        f.prepare();
        const auto old = f.controller.session_id();
        f.feedback.remote_selected = false;
        f.tick();
        check(f.controller.session_id() == old + 1,
              "unused readiness window is retired when A is deselected");
        f.tick(5s);
        check(f.controller.session_id() == old + 1, "continued A-off does not repeatedly retire sessions");
        check(!f.controller.intent().transmit, "unused readiness loss emits no brake frame");
        f.feedback.remote_selected = true;
        f.tick();
        f.tick(200ms);
        check(f.controller.ready(), "A reselection can establish a new readiness window");
        check(!f.controller.accept(25, old, f.feedback, f.now).accepted,
              "delayed command from previous readiness window is rejected");
        check(f.controller.accept(10, f.controller.session_id(), f.feedback, f.now).accepted,
              "fresh new-window command is accepted");
    }
    for (int failure = 0; failure < 15; ++failure) {
        Fixture f;
        f.start();
        const auto original_session = f.controller.session_id();
        invalidate(f.feedback, failure);
        f.tick();
        f.invalid();
        check(f.controller.session_id() == original_session + 1, "gate loss retires one session");
        const auto failure_reason = f.controller.reason();
        f.feedback = healthy();
        check(!f.controller.accept(50, original_session + 1, f.feedback, f.now).accepted,
              "new session cannot interrupt invalidation interval");
        f.tick(999ms);
        f.invalid();
        check(f.controller.reason() == failure_reason, "recovered feedback does not erase invalidation reason");
        f.tick(1ms);
        check(f.controller.state() == S::Idle && !f.controller.intent().transmit,
              "invalidation finishes at fixed deadline");
        check(!f.controller.ready(), "post-invalidation dwell starts again");
        f.tick(200ms);
        check(f.controller.ready() && !f.controller.intent().transmit,
              "feedback recovery alone never resumes the old command");
        check(!f.controller.accept(25, original_session, f.feedback, f.now).accepted,
              "retired session cannot reactivate brake");
        check(f.controller.accept(10, f.controller.session_id(), f.feedback, f.now).accepted,
              "new command in new session can start a new test");
        f.valid(10);
    }
    // Both observed AD and semi-autonomous modes are allowed for this explicit
    // stationary response test. This does not claim ownership of every axis.
    for (int mode : {1, 3}) {
        Fixture f;
        f.feedback.drive_mode = static_cast<std::uint8_t>(mode);
        f.feedback.speed_mps = -0.05;
        f.start(0.0);
        f.valid(0.0);
        check(f.controller.command_age_sec(f.now) == 0.0, "new command has age zero");
        f.tick(199ms);
        f.valid(0.0);
        f.tick(1ms);
        f.invalid();
        check(f.controller.reason().find("timed out") != std::string::npos, "lease expiration reported");
    }
    {
        Fixture f;
        f.start(10.0);
        const auto begun = f.now;
        const auto session = f.controller.session_id();
        for (int count = 1; count < 20; ++count) {
            f.now = begun + std::chrono::milliseconds(count * 100);
            check(f.controller.accept(20, session, f.feedback, f.now).accepted,
                  "fresh same-session command updates the lease");
            f.valid(20);
        }
        f.now = begun + 1999ms;
        check(f.controller.accept(30, session, f.feedback, f.now).accepted,
              "command just before total deadline can update percentage");
        f.now = begun + 2000ms;
        check(!f.controller.accept(40, session, f.feedback, f.now).accepted,
              "messages cannot extend the original maximum test duration");
        f.invalid();
        check(f.controller.reason().find("Maximum") != std::string::npos,
              "hard duration expiration has distinct reason");
        check(f.controller.session_id() == session + 1, "hard duration retires session once");
    }
    {
        Fixture f;
        f.prepare();
        for (double value : {-0.1, 100.1, std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()}) {
            check(!f.controller.accept(value, 77, f.feedback, f.now).accepted, "invalid percentage rejected");
            check(!f.controller.intent().transmit, "invalid percentage has no side effect");
        }
        check(!f.controller.accept(10, 0, f.feedback, f.now).accepted, "zero session rejected");
        check(!f.controller.accept(10, 78, f.feedback, f.now).accepted, "other session rejected");
        check(f.controller.accept(100, 77, f.feedback, f.now).accepted, "configured upper bound accepted");
        f.valid(100);
        f.tick(150ms);
        check(!f.controller.accept(101, 77, f.feedback, f.now).accepted, "bad update rejected");
        f.tick(50ms);
        f.invalid();
        check(f.controller.reason().find("timed out") != std::string::npos,
              "rejected update did not refresh command lease");
    }
    {
        Fixture f;
        f.start();
        const auto old = f.controller.session_id();
        f.controller.on_tx_failure("WouldBlock", f.now);
        f.invalid();
        const auto cause = f.controller.reason();
        check(cause.find("WouldBlock") != std::string::npos, "503 TX error remains visible");
        f.tick(900ms);
        f.controller.on_tx_failure("another failure", f.now);
        f.controller.cancel(f.now);
        f.controller.set_allowed(false, f.feedback, f.now);
        check(f.controller.session_id() == old + 1, "repeated invalidation does not renew session again");
        check(f.controller.reason() == cause, "first invalidation reason preserved");
        f.tick(100ms);
        check(f.controller.state() == S::Idle && !f.controller.intent().transmit,
              "TX errors and disable cannot extend invalidation deadline");
        f.tick(5s);
        check(!f.controller.ready() && !f.controller.intent().transmit, "paused feedback cannot restart test");
        f.controller.set_allowed(true, f.feedback, f.now);
        f.tick(200ms);
        check(f.controller.ready() && !f.controller.intent().transmit,
              "resume only restores readiness, never a previous target");
    }
    {
        Fixture f;
        f.prepare();
        const auto old = f.controller.session_id();
        f.controller.set_allowed(false, f.feedback, f.now);
        check(!f.controller.ready() && !f.controller.intent().transmit, "idle pause emits no 503");
        check(f.controller.session_id() == old + 1, "idle pause invalidates queued old-session commands");
        f.controller.set_allowed(false, f.feedback, f.now);
        check(f.controller.session_id() == old + 1, "repeated pause is idempotent");
        f.controller.set_allowed(true, f.feedback, f.now);
        f.tick(200ms);
        check(!f.controller.accept(10, old, f.feedback, f.now).accepted, "queued command from before pause rejected");
    }
    {
        Fixture f;
        f.start();
        f.controller.set_allowed(false, f.feedback, f.now);
        f.invalid();
        f.tick(500ms);
        f.controller.set_allowed(true, f.feedback, f.now);
        f.invalid();
        f.tick(500ms);
        check(!f.controller.intent().transmit && !f.controller.ready(),
              "re-enable did not cancel release or retain pre-release dwell");
    }
    {
        Fixture f;
        f.start();
        f.controller.on_disconnect();
        f.tick(5s);
        f.controller.on_tx_failure("ignored", f.now);
        f.controller.cancel(f.now);
        f.controller.set_allowed(true, f.feedback, f.now);
        check(f.controller.state() == S::Disconnected && !f.controller.intent().transmit,
              "disconnect permanently disables this controller instance");
        check(!f.controller.accept(10, f.controller.session_id(), f.feedback, f.now).accepted,
              "disconnected controller rejects command");
    }
    {
        Fixture f;
        B disabled(config(false), 1);
        disabled.tick(f.feedback, f.now);
        f.now += 5s;
        disabled.tick(f.feedback, f.now);
        check(!disabled.ready() && !disabled.intent().transmit, "enable_tx false prevents brake readiness");
        check(!disabled.accept(10, 1, f.feedback, f.now).accepted, "disabled configuration rejects command");
        disabled.on_tx_failure("ignored", f.now);
        disabled.set_allowed(true, f.feedback, f.now);
        check(!disabled.intent().transmit, "allowed cannot override enable_tx false");
    }
    {
        Fixture f;
        const auto last = std::numeric_limits<std::uint64_t>::max();
        B exhausted(config(), last);
        exhausted.tick(f.feedback, f.now);
        f.now += 200ms;
        check(exhausted.accept(10, last, f.feedback, f.now).accepted, "last unique session can be used once");
        exhausted.cancel(f.now);
        check(exhausted.session_id() == last, "session counter never wraps to zero");
        f.now += 1000ms;
        exhausted.tick(f.feedback, f.now);
        f.now += 5s;
        exhausted.tick(f.feedback, f.now);
        check(!exhausted.ready() && !exhausted.intent().transmit, "exhausted session requires restart");
        check(!exhausted.accept(10, last, f.feedback, f.now).accepted, "last retired session cannot be reused");
    }
    {
        Fixture f;
        auto limited_config = config();
        limited_config.max_brake_percent = 10;
        B limited(limited_config, 5);
        limited.tick(f.feedback, f.now);
        f.now += 200ms;
        limited.tick(f.feedback, f.now);
        check(!limited.accept(10.1, 5, f.feedback, f.now).accepted, "configured brake cap enforced");
        check(limited.accept(10, 5, f.feedback, f.now).accepted, "configured brake cap inclusive");
    }
    for (int invalid = 0; invalid < 12; ++invalid) {
        auto bad = config();
        std::uint64_t seed = 1;
        switch (invalid) {
            case 0: seed = 0; break;
            case 1: bad.command_timeout = 0ms; break;
            case 2: bad.max_test_duration = -1ms; break;
            case 3: bad.release_duration = 0ms; break;
            case 4: bad.stopped_dwell = 0ms; break;
            case 5: bad.stopped_threshold_mps = 0; break;
            case 6: bad.stopped_threshold_mps = std::numeric_limits<double>::infinity(); break;
            case 7: bad.stopped_threshold_mps = std::numeric_limits<double>::quiet_NaN(); break;
            case 8: bad.max_brake_percent = 0; break;
            case 9: bad.max_brake_percent = 100.1; break;
            case 10: bad.max_brake_percent = std::numeric_limits<double>::infinity(); break;
            case 11: bad.max_brake_percent = std::numeric_limits<double>::quiet_NaN(); break;
        }
        bool rejected = false;
        try { B controller(bad, seed); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "invalid configuration rejected");
    }
    std::cout << checks << " stationary brake-test controller assertions passed.\n";
}
