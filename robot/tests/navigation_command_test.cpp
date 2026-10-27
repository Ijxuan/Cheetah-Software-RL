#include <rt/NavigationCommand.h>
#include <rt/rt_rc_interface.h>
#include <linux/joystick.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>

static void check(bool ok, const char* message) {
  if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}
static bool near(double a, double b) { return std::fabs(a - b) < 1e-6; }

int main() {
  NavigationCommand nav;
  double vx, wz;
  check(!nav.receive(.2, .1, 0), "manual must ignore packets");
  nav.buttons(true, true, false, 1);
  check(nav.sample(1, &vx, &wz) && vx == 0, "entry must wait for fresh packet");
  check(nav.receive(9, -9, 1.1), "armed packet rejected");
  nav.buttons(true, true, false, 1.2);
  nav.sample(1.2, &vx, &wz);
  check(near(vx, .8) && near(wz, -.8), "clamp or held RB incorrect");
  check(!nav.receive(std::numeric_limits<double>::quiet_NaN(), 0, 2), "NaN accepted");
  check(!nav.receive(0, std::numeric_limits<double>::infinity(), 2), "Inf accepted");
  nav.sample(2.11, &vx, &wz);
  check(vx == 0 && wz == 0, "invalid packets refreshed timeout");
  check(nav.receive(.3, .2, 2.2), "short interruption should recover");
  check(!nav.receive(.5, .5, 5.21), "late packet re-armed timed-out navigation");
  check(nav.sample(5.3, &vx, &wz) && vx == 0, "timeout must hold zero");
  nav.buttons(true, true, false, 5.4);
  check(nav.state() == NavigationCommand::State::TimedOut, "held RB re-armed");
  nav.buttons(true, false, false, 5.5);
  nav.buttons(true, true, false, 5.6);
  check(nav.receive(.4, 0, 5.7), "fresh RB did not re-arm");
  nav.buttons(true, true, true, 5.8);
  check(!nav.sample(5.8, &vx, &wz), "RT must win over RB");
  nav.buttons(false, false, false, 6);
  nav.buttons(true, true, false, 7);
  nav.sample(10.01, &vx, &wz);
  check(nav.state() == NavigationCommand::State::TimedOut, "no-first-packet timeout missing");
  nav.cancel();
  check(!nav.receive(.2, 0, 11), "cancel did not disarm");

  // Exercise the production RC path using joystick events in a pipe. No device,
  // network publisher, robot runner or motor output is started by this test.
  int fd[2];
  check(pipe(fd) == 0, "pipe failed");
  auto event = [&](unsigned char type, unsigned char number, short value) {
    js_event e{}; e.type = type; e.number = number; e.value = value;
    check(write(fd[1], &e, sizeof(e)) == sizeof(e), "pipe write failed");
    js_complete(fd[0]);
  };
  auto button = [&](unsigned char n, short v) { event(JS_EVENT_BUTTON, n, v); };
  auto axis = [&](unsigned char n, short v) { event(JS_EVENT_AXIS, n, v); };
  rc_control_settings out{};
  request_rc_off();
  button(7, 1); button(7, 0); button(6, 1); button(6, 0);
  get_rc_control_settings(&out);
  check(out.mode == RC_mode::OFF, "START/BACK bypassed OFF");
  axis(2, 32767); button(0, 1); button(0, 0); axis(2, 0); // LT+A recover
  button(6, 1); button(6, 0); // BACK -> RL
  get_rc_control_settings(&out);
  check(out.mode == RC_mode::RL_JOINT_PD, "could not enter RL");
  check(out.mode == RC_mode::RL_STAIRS, "BACK did not select stairs");
  axis(1, -16384); // stairs manual vx=.3
  set_cmd_vel_command(.7, .1);
  get_rc_control_settings(&out);
  check(near(out.v_des[0], .3), "unarmed navigation overrode manual");
  button(5, 1); // RB
  get_rc_control_settings(&out);
  check(out.v_des[0] == 0, "entry used old packet/manual input");
  set_cmd_vel_command(.3, .2);
  get_rc_control_settings(&out);
  check(near(out.v_des[0], .3) && near(out.omega_des[2], .2), "navigation not sampled independently");
  set_cmd_vel_command(.4, -.1);
  get_rc_control_settings(&out);
  check(near(out.v_des[0], .4), "navigation required another joystick event");
  axis(0, 12000); // RB still held
  get_rc_control_settings(&out);
  check(near(out.v_des[0], .4) && out.v_des[1] == 0, "held RB cleared command");
  axis(5, 20000); // RT immediate manual
  get_rc_control_settings(&out);
  check(near(out.v_des[0], .3), "RT retained navigation speed");
  axis(5, 0); button(5, 0); button(5, 1);
  set_cmd_vel_command(.3, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(1050));
  get_rc_control_settings(&out);
  check(out.v_des[0] == 0, "production first timeout failed");
  std::this_thread::sleep_for(std::chrono::milliseconds(2050));
  get_rc_control_settings(&out);
  check(out.v_des[0] == 0, "production exit timeout failed");
  set_cmd_vel_command(.6, 0);
  get_rc_control_settings(&out);
  check(out.v_des[0] == 0, "late packet resumed motion");
  button(5, 0); button(5, 1); set_cmd_vel_command(.3, 0);
  request_rc_off(); set_cmd_vel_command(.8, .8);
  get_rc_control_settings(&out);
  check(out.mode == RC_mode::OFF && out.v_des[0] == 0, "OFF overridden");
  button(5, 0);
  axis(2, 32767); button(0, 1); button(0, 0); axis(2, 0);
  button(6, 1); button(6, 0);
  set_cmd_vel_command(.7, 0);
  get_rc_control_settings(&out);
  check(near(out.v_des[0], .3), "OFF recovery restored navigation");
  button(5, 1); set_cmd_vel_command(.3, 0);
  button(7, 1); // START selects flat, preserves navigation
  get_rc_control_settings(&out);
  const auto selection_sequence = out.rl_selection_sequence;
  check(out.mode == RC_mode::RL_FLAT &&
        near(out.v_des[0], .3), "START lost navigation or did not select flat");
  axis(0, 10000); // held START must not keep retrying
  get_rc_control_settings(&out);
  check(out.rl_selection_sequence == selection_sequence, "held START repeated selection");
  button(7, 0);
  button(6, 1); button(6, 0);
  get_rc_control_settings(&out);
  check(out.mode == RC_mode::RL_STAIRS && near(out.v_des[0], .3),
        "BACK lost navigation or did not select stairs");
  axis(2, 32767); button(3, 0); // no effect
  button(2, 1); button(2, 0); axis(2, 0); // LT+X -> recovery
  get_rc_control_settings(&out);
  check(out.mode == RC_mode::RECOVERY_STAND, "LT+X did not recover");
  button(5, 0); button(7, 1); button(7, 0); // START from recovery
  set_cmd_vel_command(.7, 0);
  get_rc_control_settings(&out);
  check(out.mode == RC_mode::RL_FLAT && near(out.v_des[0], .5),
        "recovery retained navigation or START failed from recovery");
  // Keep LY held through repeated mode changes: button/RX events must not
  // erase it, even when no new LY event arrives from the joystick driver.
  axis(0, 7252); axis(1, 2849); axis(3, 0);
  for (int i = 0; i < 12; ++i) {
    const unsigned char select = i % 2 ? 7 : 6;
    button(select, 1); button(select, 0);
    axis(3, i % 2 ? 12000 : -12000);
    get_rc_control_settings(&out);
    check(out.input_diagnostics.joystick_valid &&
          out.input_diagnostics.lx == 7252 && out.input_diagnostics.ly == 2849 &&
          out.input_diagnostics.rx == (i % 2 ? 12000 : -12000) &&
          out.input_diagnostics.navigation_state == 0 &&
          out.input_diagnostics.event_age_ms >= 0.,
          "input diagnostics did not match the manual RC snapshot");
    check(near(out.v_des[0], -(i % 2 ? 1. : .6) * 2849. / 32768.) &&
          near(out.v_des[1], -.5 * 7252. / 32768.),
          "held left stick was lost during mode/right-stick changes");
  }
  axis(1, -18392); get_rc_control_settings(&out);
  check(near(out.v_des[0], 18392. / 32768.), "forward input stayed latched at zero");
  axis(1, 18392); get_rc_control_settings(&out);
  check(near(out.v_des[0], -18392. / 32768.), "reverse input stayed latched at zero");
  // Full and half stick travel use the stairs range, not a flat-speed plateau.
  button(6, 1); button(6, 0); // physical BACK -> stairs
  axis(1, -32768); get_rc_control_settings(&out);
  check(near(out.v_des[0], .6), "stairs full forward is not 0.6 m/s");
  axis(1, -16384); get_rc_control_settings(&out);
  check(near(out.v_des[0], .3), "stairs half forward is not 0.3 m/s");
  axis(1, 32767); get_rc_control_settings(&out);
  check(near(out.v_des[0], -.6 * 32767. / 32768.), "stairs reverse scaling incorrect");
  axis(6, 32767); axis(7, -32768); get_rc_control_settings(&out);
  check(out.mode == RC_mode::RL_STAIRS && near(out.v_des[0], -.6 * 32767. / 32768.),
        "D-pad unexpectedly changed mode or velocity");
  button(7, 1); button(7, 0); axis(1, -32768); get_rc_control_settings(&out);
  check(out.mode == RC_mode::RL_FLAT && near(out.v_des[0], 1.), "physical START/flat range incorrect");
  button(5, 1); set_cmd_vel_command(.3, 0);
  axis(2, 32767); button(1, 1); button(1, 0); axis(2, 0); // LT+B
  get_rc_control_settings(&out);
  check(out.mode == RC_mode::OFF, "LT+B did not override policy/navigation");
  request_rc_off();
  std::thread publisher([] {
    for (int i = 0; i < 10000; ++i) set_cmd_vel_command(.8, .8);
  });
  for (int i = 0; i < 10000; ++i) {
    request_rc_off();
    get_rc_control_settings(&out);
    check(out.mode == RC_mode::OFF && out.v_des[0] == 0,
          "concurrent navigation defeated OFF");
  }
  publisher.join();
  close(fd[0]); close(fd[1]);
  std::cout << "Navigation arbitration and production RC tests passed\n";
}
