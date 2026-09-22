#include "withus_driver/control_manager.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using M = withus_driver::ControlManager;
using S = M::State;
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
M::Config config(bool tx = true)
{
    M::Config result;
    result.tx_enabled = tx;
    result.policy = M::Policy::RemoteMode;
    return result;
}
struct Fixture
{
    M manager{config(), 100};
    M::Feedback feedback{};
    M::TimePoint now{10s};

    void only(bool request)
    {
        const auto& intent = manager.intent();
        check(intent.transmit, "0x501 remains scheduled");
        check(intent.request.request_control == request, "expected request bit");
        check(!intent.steering.valid && !intent.brake.valid && !intent.speed.valid,
              "no actuator validity");
        check(!intent.body.headlight_on && !intent.body.horn_on &&
              !intent.body.left_indicator_on && !intent.body.right_indicator_on &&
              !intent.body.brake_light_on, "no body intent");
    }
    void tick(std::chrono::milliseconds elapsed = 20ms)
    {
        now += elapsed;
        manager.tick(feedback, now);
    }
    void disable()
    {
        check(manager.set_enabled(false, feedback, now).accepted, "disable accepted");
        check(manager.state() == S::Stopping, "disable starts bounded zero phase");
        only(false);
    }
    void confirm_zero()
    {
        manager.on_control_request_sent(false, feedback, now);
        feedback.all_fresh = true;
        feedback.drive_mode = 0;
        ++feedback.status_303_sequence;
    }
};
}  // namespace

int main()
{
    {
        Fixture f;
        // No CAN receive data, A switch or readiness service is required.
        f.only(true);
        f.tick(5s);
        check(f.manager.state() == S::Monitoring, "normal broadcast does not claim ACTIVE ownership");
        f.only(true);
        check(std::isnan(f.manager.command_age_sec(f.now)), "no drive-command age");
        M::DriveTarget drive;
        drive.session_id = f.manager.session_id();
        check(!f.manager.accept_drive(drive, f.feedback, f.now).accepted, "drive rejected");
        withus_driver::BodyCommand body;
        body.horn_on = true;
        check(!f.manager.accept_body(f.manager.session_id(), body, f.now).accepted, "body rejected");
        f.only(true);
    }
    // Every received value is observational in this profile. In particular,
    // 501-only operation can report semi-autonomous mode 3 on the real vehicle.
    for (int observation = 0; observation < 15; ++observation) {
        Fixture f;
        f.feedback.all_fresh = true;
        f.feedback.remote_fresh = true;
        f.feedback.remote_known = true;
        f.feedback.gear = G::Neutral;
        f.feedback.drive_mode = 0;
        switch (observation) {
            case 0: f.feedback.all_fresh = false; break;
            case 1: f.feedback.remote_fresh = false; break;
            case 2: f.feedback.remote_known = false; break;
            case 3: f.feedback.blocked = true; break;
            case 4: f.feedback.gear = G::Drive; break;
            case 5: f.feedback.speed_mps = 0.3; break;
            case 6: f.feedback.drive_mode = 1; break;
            case 7: f.feedback.drive_mode = 2; break;
            case 8: f.feedback.drive_mode = 3; break;
            case 9: f.feedback.drive_mode = 15; break;
            case 10: f.feedback.speed_mps = std::numeric_limits<double>::quiet_NaN(); break;
            case 11: f.feedback.steering_angle_rad = std::numeric_limits<double>::infinity(); break;
            case 12: f.feedback.gear = static_cast<G>(255); break;
            case 13: f.feedback.remote_selected = true; break;
            case 14: f.feedback.ad_ready = true; break;
        }
        f.tick();
        check(f.manager.state() == S::Monitoring, "observation cannot change broadcaster state");
        f.only(true);
        f.manager.on_control_request_sent(true, f.feedback, f.now);
        f.tick(5s);
        check(f.manager.state() == S::Monitoring, "no acquisition or command timeout");
        check(f.manager.session_id() == 100, "observed mode does not allocate ownership sessions");
        f.only(true);
    }
    {
        Fixture f;
        for (bool selected : {false, true, false, true}) {
            f.feedback.remote_selected = selected;
            f.feedback.drive_mode = selected ? 3 : 0;
            ++f.feedback.status_303_sequence;
            f.tick();
            f.only(true);
            check(f.manager.state() == S::Monitoring, "A changes do not trigger release or ownership claims");
        }
    }
    {
        Fixture f;
        f.manager.on_tx_failure("WouldBlock", f.now);
        check(f.manager.state() == S::Fault, "write failure visible");
        f.only(true);
        f.tick(5s);
        check(f.manager.state() == S::Fault, "feedback cannot erase TX error");
        f.only(true);
        check(f.manager.set_enabled(true, f.feedback, f.now).accepted, "enable has no readiness gates");
        check(f.manager.state() == S::Fault, "enable cannot invent a successful write");
        f.manager.on_control_request_sent(true, f.feedback, f.now);
        check(f.manager.state() == S::Monitoring, "successful retry clears TX diagnostic");
        f.only(true);
        f.manager.on_tx_failure("another write failure", f.now);
        f.disable();
        f.confirm_zero();
        f.tick(1000ms);
        check(f.manager.state() == S::Monitoring && !f.manager.intent().transmit,
              "explicit disable works after a TX error");
    }
    {
        Fixture f;
        f.disable();
        f.confirm_zero();
        // A/gear/fault flags do not gate the bounded release or its mode observation.
        f.feedback.remote_selected = true;
        f.feedback.remote_known = false;
        f.feedback.blocked = true;
        f.feedback.gear = G::Drive;
        f.feedback.speed_mps = 0.3;
        f.tick(999ms);
        check(f.manager.state() == S::Stopping, "full release duration retained");
        f.only(false);
        check(f.manager.set_enabled(false, f.feedback, f.now).accepted, "repeated disable accepted");
        check(!f.manager.set_enabled(true, f.feedback, f.now).accepted, "enable cannot interrupt zero phase");
        f.tick(1ms);
        check(f.manager.state() == S::Monitoring, "fresh post-send manual mode observed");
        check(!f.manager.intent().transmit, "release deadline stops all transmission");
        f.tick(5s);
        check(!f.manager.intent().transmit, "explicit disable remains in force");
        check(f.manager.set_enabled(true, f.feedback, f.now).accepted, "unconditional explicit resume");
        f.only(true);
    }
    // A successful write and a later fresh 303 mode=0 are needed only to claim
    // an observed release. All failures stop transmission at the same deadline.
    for (int missing = 0; missing < 5; ++missing) {
        Fixture f;
        f.disable();
        f.feedback.all_fresh = true;
        f.feedback.drive_mode = 0;
        if (missing != 0) f.manager.on_control_request_sent(false, f.feedback, f.now);
        if (missing != 1) ++f.feedback.status_303_sequence;
        if (missing == 2) f.feedback.all_fresh = false;
        if (missing == 3) f.feedback.drive_mode = 3;
        if (missing == 4) f.manager.on_tx_failure("release retry blocked", f.now);
        f.tick(1000ms);
        check(f.manager.state() == S::Fault, "unconfirmed release is reported truthfully");
        check(!f.manager.intent().transmit, "unconfirmed release is still bounded");
        f.tick(5s);
        check(!f.manager.intent().transmit, "no automatic restart after explicit disable");
        check(f.manager.set_enabled(false, f.feedback, f.now).accepted, "repeated terminal disable accepted");
        check(!f.manager.intent().transmit, "terminal disable cannot start another zero phase");
        check(f.manager.set_enabled(true, f.feedback, f.now).accepted, "fault resume needs no A toggle or reset");
        f.only(true);
        f.manager.on_control_request_sent(true, f.feedback, f.now);
        check(f.manager.state() == S::Monitoring, "successful resumed request returns to monitoring");
    }
    {
        Fixture f;
        f.disable();
        f.confirm_zero();
        f.tick(900ms);
        f.manager.on_tx_failure("WouldBlock", f.now);
        f.only(false);
        // A successful later zero write invalidates the earlier cached feedback.
        f.manager.on_control_request_sent(false, f.feedback, f.now);
        f.tick(99ms);
        check(f.manager.state() == S::Stopping, "retry did not shorten release phase");
        ++f.feedback.status_303_sequence;
        f.tick(1ms);
        check(f.manager.state() == S::Monitoring && !f.manager.intent().transmit,
              "successful release retry did not extend deadline");
    }
    {
        Fixture f;
        f.manager.on_disconnect();
        f.tick(5s);
        check(f.manager.state() == S::Disconnected && !f.manager.intent().transmit,
              "transport disconnect is terminal");
        check(!f.manager.set_enabled(true, f.feedback, f.now).accepted, "disconnected transport requires restart");
    }
    {
        Fixture f;
        M disabled(config(false), 1);
        disabled.tick(f.feedback, f.now);
        check(!disabled.intent().transmit, "enable_tx configuration remains authoritative");
        check(!disabled.set_enabled(true, f.feedback, f.now).accepted, "service cannot override enable_tx=false");
        check(disabled.set_enabled(false, f.feedback, f.now).accepted, "disabled configuration can shut down");
        check(disabled.state() == S::Monitoring && !disabled.intent().transmit,
              "already disabled profile needs no release transmission");
    }
    std::cout << checks << " continuous-request control manager assertions passed.\n";
}
