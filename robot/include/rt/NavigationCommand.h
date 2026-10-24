#pragma once

#include <cmath>

// All access must be serialized by the RC interface mutex. Time is monotonic
// seconds, supplied by the caller so timeout behavior can be tested offline.
class NavigationCommand {
 public:
  enum class State { Manual, Waiting, Active, Stale, TimedOut };
  static constexpr double kStopTimeout = 1.0;
  static constexpr double kExitTimeout = 3.0;
  static constexpr double kMaxVx = 0.8;  // m/s
  static constexpr double kMaxWz = 0.8;  // rad/s

  State state() const { return state_; }
  void cancel() {
    state_ = State::Manual;
    vx_ = wz_ = 0.;
  }

  void buttons(bool in_rl, bool rb, bool rt, double now) {
    const bool pressed = rb && !rb_;
    rb_ = rb;
    if (!in_rl || rt) {
      cancel();
    } else if (pressed) {
      state_ = State::Waiting;
      vx_ = wz_ = 0.;
      last_time_ = now;
    }
  }

  bool receive(double vx, double wz, double now) {
    if (!std::isfinite(vx) || !std::isfinite(wz)) return false;
    tick(now);  // A packet arriving after exit timeout cannot re-arm navigation.
    if (state_ == State::Manual || state_ == State::TimedOut) return false;
    vx_ = vx > kMaxVx ? kMaxVx : (vx < -kMaxVx ? -kMaxVx : vx);
    wz_ = wz > kMaxWz ? kMaxWz : (wz < -kMaxWz ? -kMaxWz : wz);
    last_time_ = now;
    state_ = State::Active;
    return true;
  }

  // true means override manual velocity, including zero while waiting/stale.
  bool sample(double now, double* vx, double* wz) {
    tick(now);
    *vx = state_ == State::Active ? vx_ : 0.;
    *wz = state_ == State::Active ? wz_ : 0.;
    return state_ != State::Manual;
  }

 private:
  void tick(double now) {
    if (state_ == State::Manual || state_ == State::TimedOut) return;
    const double age = now - last_time_;
    if (age >= kExitTimeout) state_ = State::TimedOut;
    else if (age >= kStopTimeout) state_ = State::Stale;
  }
  State state_ = State::Manual;
  bool rb_ = false;
  double vx_ = 0., wz_ = 0., last_time_ = 0.;
};
