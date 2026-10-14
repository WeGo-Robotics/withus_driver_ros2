#include "withus_driver/control_manager.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

using withus_driver::ControlManager;
using withus_driver::Gear;
using Manager = ControlManager;
using State = Manager::State;
using namespace std::chrono_literals;

namespace
{
unsigned checks = 0;
void check(bool condition, const char* message)
{
    ++checks;
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

Manager::Config config()
{
    Manager::Config c;
    c.tx_enabled = true;
    return c;
}

struct Fixture
{
    Manager manager{config(), 1000};
    Manager::Feedback feedback{};
    Manager::TimePoint now{10s};

    Fixture()
    {
        feedback.all_fresh = true;
        feedback.gear = Gear::Drive;
        feedback.status_303_sequence = 1;
        manager.tick(feedback, now);
    }

    void activate()
    {
        now += 300ms;
        check(manager.set_enabled(true, feedback, now).accepted, "enable accepted");
        check(manager.state() == State::Requesting, "requesting state");
        check(manager.intent().request.request_control, "control flag enabled");
        check(!manager.intent().speed.valid, "no speed command before ownership");
        feedback.ad_ready = true;
        manager.tick(feedback, now);
        check(manager.state() == State::Requesting, "pre-request 303 cannot acknowledge");
        ++feedback.status_303_sequence;
        now += 20ms;
        manager.tick(feedback, now);
        check(manager.state() == State::Active, "fresh post-request acknowledgement");
        check(manager.intent().speed.speed_mps == 0.0, "active without command remains zero");
        check(manager.intent().brake.brake_percent == 25.0, "active without command holds brake");
    }

    Manager::DriveTarget target()
    {
        Manager::DriveTarget value;
        value.session_id = manager.session_id();
        value.gear = feedback.gear;
        value.speed_mps = 0.4;
        value.steering_angle_rad = 0.1;
        return value;
    }
};
}  // namespace

int main()
{
    {
        Fixture f;
        check(f.manager.state() == State::Monitoring, "initial monitoring");
        check(!f.manager.intent().transmit, "no initial transmission");
        check(std::isnan(f.manager.command_age_sec(f.now)), "no command age is NaN");
        auto target = f.target();
        check(!f.manager.accept_drive(target, f.feedback, f.now).accepted, "no early command");
        f.feedback.all_fresh = false;
        check(!f.manager.set_enabled(true, f.feedback, f.now).accepted, "no enable with stale feedback");
        f.feedback.all_fresh = true;
        f.feedback.blocked = true;
        check(!f.manager.set_enabled(true, f.feedback, f.now).accepted, "no enable with faults");
        f.feedback.blocked = false;
        f.feedback.speed_mps = 0.1;
        check(!f.manager.set_enabled(true, f.feedback, f.now).accepted, "no rolling enable");
        f.feedback.speed_mps = 0.0;
        f.feedback.ad_ready = true;
        check(!f.manager.set_enabled(true, f.feedback, f.now).accepted, "no ambiguous existing AD ownership");
        Manager disabled(Manager::Config{}, 9);
        f.feedback.ad_ready = false;
        check(!disabled.set_enabled(true, f.feedback, f.now).accepted, "configuration disables TX");
    }
    {
        Fixture f;
        f.activate();
        auto target = f.target();
        check(f.manager.accept_drive(target, f.feedback, f.now).accepted, "drive accepted");
        check(f.manager.intent().speed.speed_mps == 0.4, "drive speed intent");
        check(f.manager.intent().steering.steering_angle_rad == 0.1, "steering intent");
        check(f.manager.command_age_sec(f.now + 100ms) == 0.1, "steady command age");
        const auto previous = target;
        target.session_id--;
        check(!f.manager.accept_drive(target, f.feedback, f.now).accepted, "old session rejected");
        target = previous;
        target.speed_mps = std::numeric_limits<double>::quiet_NaN();
        check(!f.manager.accept_drive(target, f.feedback, f.now).accepted, "NaN speed rejected");
        target = previous;
        target.steering_angle_rad = std::numeric_limits<double>::infinity();
        check(!f.manager.accept_drive(target, f.feedback, f.now).accepted, "infinite steering rejected");
        target = previous;
        target.speed_mps = -0.1;
        check(!f.manager.accept_drive(target, f.feedback, f.now).accepted, "negative speed rejected");
        target = previous;
        target.speed_mps = 1.01;
        check(!f.manager.accept_drive(target, f.feedback, f.now).accepted, "forward speed limit");
        target = previous;
        target.brake_percent = 1.0;
        check(!f.manager.accept_drive(target, f.feedback, f.now).accepted, "simultaneous speed and brake rejected");
        target = previous;
        target.gear = Gear::Park;
        check(!f.manager.accept_drive(target, f.feedback, f.now).accepted, "park speed rejected");
        target = previous;
        target.gear = static_cast<Gear>(255);
        check(!f.manager.accept_drive(target, f.feedback, f.now).accepted, "invalid gear rejected");
        f.manager.tick(f.feedback, f.now + 200ms);
        check(f.manager.state() == State::Stopping, "exact command timeout enters stopping");
        check(f.manager.intent().speed.speed_mps == 0.0, "stop clears cached speed");
        check(f.manager.intent().brake.brake_percent == 25.0, "stop brake");
        check(!f.manager.accept_drive(previous, f.feedback, f.now + 201ms).accepted,
              "new commands cannot interrupt stopping");
        f.manager.tick(f.feedback, f.now + 400ms);
        check(!f.manager.intent().request.request_control, "release after stopped dwell");
        check(!f.manager.intent().speed.valid, "release disables speed validity");
        f.feedback.ad_ready = false;
        f.manager.tick(f.feedback, f.now + 420ms);
        check(f.manager.state() == State::Stopping, "release needs post-release 303");
        ++f.feedback.status_303_sequence;
        f.manager.tick(f.feedback, f.now + 440ms);
        check(f.manager.state() == State::Monitoring, "release acknowledgement completes stop");
        check(!f.manager.intent().transmit, "monitoring stops reference transmissions");
        check(f.manager.set_enabled(true, f.feedback, f.now + 460ms).accepted, "new explicit session starts");
        check(f.manager.session_id() != previous.session_id, "session changes on enable");
    }
    {
        Fixture f;
        f.activate();
        auto target = f.target();
        f.feedback.speed_mps = 0.2;
        target.gear = Gear::Reverse;
        check(!f.manager.accept_drive(target, f.feedback, f.now).accepted, "rolling reversal rejected");
        f.feedback.speed_mps = 0.0;
        check(!f.manager.accept_drive(target, f.feedback, f.now + 1ms).accepted, "gear needs stopped dwell");
        // Feed zero-speed commands while waiting for the stopped dwell.
        auto zero = f.target();
        zero.speed_mps = 0.0;
        check(f.manager.accept_drive(zero, f.feedback, f.now + 1ms).accepted, "zero command accepted");
        f.manager.tick(f.feedback, f.now + 100ms);
        check(f.manager.accept_drive(zero, f.feedback, f.now + 100ms).accepted, "refresh zero command");
        check(f.manager.accept_drive(target, f.feedback, f.now + 201ms).accepted, "stopped reversal accepted");
        check(f.manager.intent().speed.gear == Gear::Reverse, "request reverse gear");
        check(f.manager.intent().speed.speed_mps == 0.0, "no speed before gear acknowledgement");
        check(f.manager.intent().brake.brake_percent == 25.0, "brake while awaiting gear");
        f.feedback.gear = Gear::Reverse;
        f.manager.tick(f.feedback, f.now + 221ms);
        check(f.manager.intent().speed.speed_mps == 0.4, "speed permitted after matching gear feedback");
        target.speed_mps = 0.51;
        check(!f.manager.accept_drive(target, f.feedback, f.now + 222ms).accepted, "reverse speed limit");
        f.feedback.gear = Gear::Drive;
        f.manager.tick(f.feedback, f.now + 230ms);
        check(f.manager.state() == State::Fault, "unexpected gear change faults");
    }
    {
        Fixture f;
        f.activate();
        auto target = f.target();
        target.speed_mps = 0.0;
        withus_driver::BodyCommand body;
        body.headlight_on = true;
        body.left_indicator_on = true;
        body.horn_on = true;
        check(!f.manager.accept_body(999, body, f.now).accepted, "body old session rejected");
        check(f.manager.accept_body(f.manager.session_id(), body, f.now).accepted, "body accepted");
        check(f.manager.intent().body.horn_on, "horn on intent");
        for (int step = 0; step <= 5; ++step) {
            auto time = f.now + step * 100ms;
            check(f.manager.accept_drive(target, f.feedback, time).accepted, "refresh drive during body timeout");
            f.manager.tick(f.feedback, time);
        }
        check(!f.manager.intent().body.horn_on, "body timeout clears horn");
        check(!f.manager.intent().body.left_indicator_on, "body timeout clears indicators");
        check(f.manager.intent().body.headlight_on, "body timeout retains headlight");
        f.manager.on_tx_failure("WouldBlock");
        check(f.manager.state() == State::Fault, "TX failure faults");
        check(!f.manager.intent().transmit, "fault suppresses TX");
        check(std::isnan(f.manager.command_age_sec(f.now)), "fault clears cached command");
        f.manager.tick(f.feedback, f.now + 1s);
        check(f.manager.state() == State::Fault, "healthy feedback does not clear fault");
        check(!f.manager.set_enabled(true, f.feedback, f.now + 1s).accepted, "enable cannot clear fault");
        f.feedback.ad_ready = false;
        check(f.manager.set_enabled(false, f.feedback, f.now + 1s).accepted, "explicit stopped fault reset");
        check(f.manager.state() == State::Monitoring, "fault reset only monitors");
    }
    for (int failure = 0; failure != 3; ++failure) {
        Fixture f;
        f.activate();
        if (failure == 0) { f.feedback.all_fresh = false; }
        if (failure == 1) { f.feedback.blocked = true; }
        if (failure == 2) { f.feedback.ad_ready = false; }
        f.manager.tick(f.feedback, f.now + 1ms);
        check(f.manager.state() == State::Fault, "feedback issue faults active state");
        check(!f.manager.intent().transmit, "feedback fault suppresses TX");
    }
    {
        Fixture f;
        check(f.manager.set_enabled(true, f.feedback, f.now).accepted, "start request timeout test");
        f.manager.tick(f.feedback, f.now + 1s);
        check(f.manager.state() == State::Fault, "request deadline faults");
    }
    {
        Fixture f;
        f.activate();
        f.feedback.speed_mps = 0.3;
        check(f.manager.set_enabled(false, f.feedback, f.now).accepted, "rolling disable starts stop");
        check(f.manager.state() == State::Stopping, "rolling stopping");
        check(f.manager.set_enabled(false, f.feedback, f.now + 2s).accepted, "repeated disable idempotent");
        f.manager.tick(f.feedback, f.now + 3s);
        check(f.manager.state() == State::Fault, "repeated disable does not reset stop deadline");
        check(!f.manager.set_enabled(false, f.feedback, f.now + 4s).accepted, "rolling fault cannot reset");
    }
    {
        Fixture f;
        f.activate();
        check(f.manager.set_enabled(false, f.feedback, f.now).accepted, "disable for release timeout");
        f.manager.tick(f.feedback, f.now + 200ms);
        check(!f.manager.intent().request.request_control, "release intent generated");
        f.manager.tick(f.feedback, f.now + 1200ms);
        check(f.manager.state() == State::Fault, "release deadline faults");
    }
    {
        Fixture f;
        check(f.manager.set_enabled(true, f.feedback, f.now).accepted, "request before cancellation");
        check(f.manager.set_enabled(false, f.feedback, f.now + 20ms).accepted, "cancel unacknowledged request");
        check(!f.manager.intent().request.request_control, "cancel releases request");
        check(!f.manager.intent().speed.valid, "cancel cannot start drive commands");
        ++f.feedback.status_303_sequence;
        f.manager.tick(f.feedback, f.now + 40ms);
        check(f.manager.state() == State::Monitoring, "cancel ack completes");
    }
    {
        Fixture f;
        f.activate();
        f.manager.on_disconnect();
        check(f.manager.state() == State::Disconnected, "disconnect state");
        check(!f.manager.intent().transmit, "disconnect suppresses TX");
        check(!f.manager.set_enabled(false, f.feedback, f.now).accepted, "disconnected loop requires restart");
    }
    {
        Fixture f;
        // A 0.3 rad target encodes to 17.2 degrees, whose decoded angle is >0.3.
        const double quantized_angle = 17.2 * 3.14159265358979323846 / 180.0;
        f.feedback.steering_angle_rad = quantized_angle;
        check(f.manager.set_enabled(true, f.feedback, f.now).accepted,
              "enable tolerates half-step target quantization");
        f.feedback.ad_ready = true;
        ++f.feedback.status_303_sequence;
        f.manager.tick(f.feedback, f.now + 20ms);
        check(f.manager.state() == State::Active, "acquisition tolerates quantized bound");
        check(f.manager.intent().steering.steering_angle_rad == quantized_angle,
              "quantized measured angle is held without clamping");
    }
    {
        Fixture f;
        f.feedback.steering_angle_rad = -0.301;
        check(!f.manager.set_enabled(true, f.feedback, f.now).accepted,
              "more than half-step beyond configured limit is rejected");
        auto boundary_config = config();
        boundary_config.max_steering_rad = withus_driver::kMaxCommandSteeringRad;
        Manager boundary(boundary_config, 700);
        f.feedback.steering_angle_rad = withus_driver::kMaxCommandSteeringRad + 0.0001;
        check(!boundary.set_enabled(true, f.feedback, f.now).accepted,
              "quantization allowance cannot expand protocol angle bound");
        f.feedback.steering_angle_rad = withus_driver::kMaxCommandSteeringRad;
        check(boundary.set_enabled(true, f.feedback, f.now).accepted,
              "exact protocol bound is accepted");
    }
    {
        Fixture f;
        f.feedback.steering_angle_rad = 0.45;
        check(!f.manager.set_enabled(true, f.feedback, f.now).accepted,
              "enable rejects measured angle outside configured target limit");
        check(!f.manager.intent().transmit, "rejected enable sends no steering");
        f.feedback.steering_angle_rad = 0.6;
        check(!f.manager.set_enabled(true, f.feedback, f.now).accepted,
              "enable rejects unrepresentable measured steering");
    }
    {
        Fixture f;
        f.feedback.steering_angle_rad = 0.1;
        check(f.manager.set_enabled(true, f.feedback, f.now).accepted, "enable with nonzero angle");
        f.feedback.steering_angle_rad = 0.2;
        f.feedback.ad_ready = true;
        ++f.feedback.status_303_sequence;
        f.manager.tick(f.feedback, f.now + 20ms);
        check(f.manager.state() == State::Active, "changed but permitted acquisition angle");
        check(f.manager.intent().steering.steering_angle_rad == 0.2,
              "ACTIVE holds latest acquisition angle instead of pre-request angle");
        f.feedback.steering_angle_rad = 0.45;
        check(f.manager.set_enabled(false, f.feedback, f.now + 40ms).accepted,
              "stop accepts representable angle outside target limit");
        check(f.manager.intent().steering.steering_angle_rad == 0.45,
              "STOPPING does not clamp measured angle into a movement");
    }
    {
        Fixture f;
        check(f.manager.set_enabled(true, f.feedback, f.now).accepted, "request before steering drift");
        f.feedback.steering_angle_rad = 0.45;
        f.feedback.ad_ready = true;
        ++f.feedback.status_303_sequence;
        f.manager.tick(f.feedback, f.now + 20ms);
        check(f.manager.state() == State::Fault, "acquisition outside target angle policy faults");
        check(!f.manager.intent().transmit, "acquisition angle fault never sends clamped command");
    }
    for (const double angle : {-0.6, 0.6}) {
        Fixture f;
        f.activate();
        f.feedback.steering_angle_rad = angle;
        check(!f.manager.set_enabled(false, f.feedback, f.now + 20ms).accepted,
              "cannot stop with an unrepresentable hold angle");
        check(f.manager.state() == State::Fault, "unrepresentable stop angle faults");
        check(!f.manager.intent().transmit, "unrepresentable angle never encoded");
    }
    {
        Fixture f;
        f.activate();
        f.feedback.ad_ready = false;
        f.feedback.all_fresh = false;
        f.manager.tick(f.feedback, f.now + 20ms);
        check(f.manager.state() == State::Fault, "stale relinquished AD cannot imply successful release");
        check(!f.manager.set_enabled(false, f.feedback, f.now + 40ms).accepted,
              "stale AD0 feedback cannot reset ownership fault");
    }
    {
        bool threw = false;
        auto invalid = config();
        invalid.stop_brake_percent = 101.0;
        try { Manager manager(invalid, 1); } catch (const std::invalid_argument&) { threw = true; }
        check(threw, "invalid configuration rejected");
        Fixture f;
        Manager exhausted(config(), std::numeric_limits<std::uint64_t>::max());
        check(!exhausted.set_enabled(true, f.feedback, f.now).accepted, "session overflow rejected");
    }
    std::cout << checks << " control manager assertions passed.\n";
}
