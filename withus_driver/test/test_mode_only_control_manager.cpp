#include "withus_driver/control_manager.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using M = withus_driver::ControlManager;
using S = M::State;
using G = withus_driver::Gear;
using namespace std::chrono_literals;
int checks = 0;
void check(bool condition, const char* msg) {
  ++checks;
  if (!condition) { std::cerr << "FAIL " << msg << '\n'; std::exit(1); }
}
M::Config config(bool tx = true) {
  M::Config c; c.tx_enabled = tx; c.policy = M::Policy::ModeOnly; return c;
}
struct F {
  M m{config(), 100}; M::Feedback f{}; M::TimePoint now{10s};
  F() {
    f.all_fresh = true; f.remote_fresh = true; f.remote_selected = true;
    f.gear = G::Neutral; f.drive_mode = 0; f.status_303_sequence = 5;
    // Measured steering can exceed the virtual command bound in mode-only.
    f.steering_angle_rad = 0.6;
  }
  void only(bool request) {
    check(m.intent().transmit, "mode intent transmits");
    check(m.intent().request.request_control == request, "expected request bit");
    check(!m.intent().steering.valid && !m.intent().speed.valid && !m.intent().brake.valid,
          "no actuator valid fields");
    check(!m.intent().body.headlight_on && !m.intent().body.left_indicator_on &&
          !m.intent().body.right_indicator_on && !m.intent().body.horn_on &&
          !m.intent().body.brake_light_on, "no body fields");
  }
  void start() {
    check(m.set_enabled(true, f, now).accepted, "start accepted");
    check(m.state() == S::Requesting, "requesting"); only(true);
  }
  void activate() {
    start(); m.on_control_request_sent(true, f, now);
    f.drive_mode = 1; ++f.status_303_sequence; now += 20ms; m.tick(f, now);
    check(m.state() == S::Active, "active after sent request and new303"); only(true);
  }
  void release_ack(bool fault) {
    check(m.state() == S::Stopping, "releasing state"); only(false);
    auto begun = now; m.on_control_request_sent(false, f, now);
    ++f.status_303_sequence; f.drive_mode = 0;
    now = begun + 999ms; m.tick(f, now);
    check(m.state() == S::Stopping, "complete full release duration"); only(false);
    now = begun + 1000ms; m.tick(f, now);
    check(m.state() == (fault ? S::Fault : S::Monitoring), "release terminal state");
    check(!m.intent().transmit, "release terminal no transmission");
  }
};
int main() {
  {
    F v; check(!v.m.intent().transmit, "monitoring no transmission");
    check(std::isnan(v.m.command_age_sec(v.now)), "mode age NaN");
    v.start(); v.f.drive_mode = 1; ++v.f.status_303_sequence;
    v.m.tick(v.f, v.now); check(v.m.state() == S::Requesting, "no ack before first write");
    v.m.on_control_request_sent(true, v.f, v.now);
    v.m.tick(v.f, v.now); check(v.m.state() == S::Requesting, "cached pre-write no ack");
    ++v.f.status_303_sequence; v.m.tick(v.f, v.now);
    check(v.m.state() == S::Active, "fresh post-write acknowledgement");
    check(!v.f.ad_ready, "ad_ready is not required for real mode acknowledgement");
    v.now += 5s; v.m.tick(v.f, v.now);
    check(v.m.state() == S::Active, "no drive command watchdog in mode-only");
    M::DriveTarget command; command.session_id = v.m.session_id();
    check(!v.m.accept_drive(command, v.f, v.now).accepted, "reject drive");
    withus_driver::BodyCommand body; body.horn_on = true;
    check(!v.m.accept_body(v.m.session_id(), body, v.now).accepted, "reject body");
    v.only(true);
    check(v.m.set_enabled(false, v.f, v.now).accepted, "explicit disable accepted");
    v.f.ad_ready = true; // request reception flag may still be true on release.
    v.release_ack(false);
  }
  for (int condition = 0; condition < 10; ++condition) {
    F v;
    switch (condition) {
      case 0: v.f.remote_fresh = false; break;
      case 1: v.f.remote_selected = false; break;
      case 2: v.f.all_fresh = false; break;
      case 3: v.f.blocked = true; break;
      case 4: v.f.gear = G::Drive; break;
      case 5: v.f.speed_mps = 0.1; break;
      case 6: v.f.drive_mode = 1; break;
      case 7: v.f.drive_mode = 255; break;
      case 8: v.f.speed_mps = std::numeric_limits<double>::quiet_NaN(); break;
      case 9: v.f.steering_angle_rad = std::numeric_limits<double>::infinity(); break;
    }
    check(!v.m.set_enabled(true, v.f, v.now).accepted, "invalid enable condition rejected");
    check(v.m.state() == S::Monitoring && !v.m.intent().transmit, "rejection no transmission");
  }
  {
    F v; M disabled{config(false), 1};
    check(!disabled.set_enabled(true, v.f, v.now).accepted, "tx disabled rejected");
  }
  for (int condition = 0; condition < 7; ++condition) {
    F v; v.activate();
    switch (condition) {
      case 0: v.f.remote_fresh = false; break;
      case 1: v.f.remote_selected = false; break;
      case 2: v.f.all_fresh = false; break;
      case 3: v.f.blocked = true; break;
      case 4: v.f.gear = G::Drive; break;
      case 5: v.f.speed_mps = -0.1; break;
      case 6: v.f.drive_mode = 0; break;
    }
    v.m.tick(v.f, v.now); v.only(false);
    // Recover observations while releasing: the fault must remain latched.
    v.f.all_fresh = true; v.f.blocked = false; v.f.gear = G::Neutral; v.f.speed_mps = 0;
    v.release_ack(true);
    check(!v.m.set_enabled(true, v.f, v.now).accepted, "fault cannot reenable");
    v.f.drive_mode = 1;
    check(!v.m.set_enabled(false, v.f, v.now).accepted, "AD fault reset rejected");
    v.f.drive_mode = 0;
    check(v.m.set_enabled(false, v.f, v.now).accepted, "explicit safe reset");
    check(v.m.state() == S::Monitoring, "reset monitoring");
  }
  {
    F v; v.start(); v.now += 1000ms; v.m.tick(v.f, v.now);
    v.release_ack(true);
  }
  {
    F v; v.activate(); v.m.on_tx_failure("WouldBlock", v.now); v.only(false);
    v.m.on_control_request_sent(false, v.f, v.now);
    v.f.drive_mode = 0; ++v.f.status_303_sequence;
    v.now += 900ms;
    check(v.m.set_enabled(false, v.f, v.now).accepted, "repeated release accepted");
    v.m.on_tx_failure("retry failed", v.now);
    v.now += 100ms; v.m.tick(v.f, v.now);
    check(v.m.state() == S::Fault && !v.m.intent().transmit, "errors/repeated disable do not extend deadline");
  }
  for (bool successful_zero : {false, true}) {
    F v; v.activate(); v.m.set_enabled(false, v.f, v.now);
    v.f.drive_mode = 0; ++v.f.status_303_sequence;
    if (successful_zero) v.m.on_control_request_sent(false, v.f, v.now);
    v.now += 1000ms; v.m.tick(v.f, v.now);
    check(v.m.state() == S::Fault, "no successful zero or only cached zero cannot release");
  }
  {
    F v; v.activate(); v.m.on_disconnect();
    check(v.m.state() == S::Disconnected && !v.m.intent().transmit, "disconnect stops tx");
    check(!v.m.set_enabled(false, v.f, v.now).accepted, "disconnect requires restart");
  }
  std::cout << checks << " mode-only control manager assertions passed.\n";
}
