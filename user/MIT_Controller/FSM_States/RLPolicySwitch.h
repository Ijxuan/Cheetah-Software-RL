#pragma once

#include <algorithm>
#include <rt/RLControllerSelection.h>

// Pure handover state machine; all times are monotonic microseconds.
class RLPolicySwitch {
 public:
  enum class Phase { Preparing, Running, Stopping, Ready, Blending, Blocked };
  static constexpr int64_t kSettledUs = 200000;
  static constexpr int64_t kStopTimeoutUs = 5000000;
  static constexpr int64_t kBlendUs = 500000;
  static constexpr int64_t kWarmupUs = 120000;

  void reset(RLController selected, uint64_t sequence, int64_t now) {
    active_ = requested_ = selected;
    available_ = true;
    sequence_ = sequence;
    phase_ = Phase::Preparing;
    started_ = now;
    settled_since_ = -1;
  }
  void request(RLController selected, uint64_t sequence, int64_t now, bool available) {
    if (sequence == sequence_ && selected == requested_) return;
    sequence_ = sequence;
    requested_ = selected;
    available_ = available && validRLController(selected);
    // Do not interrupt a blend midway. Queue the latest request until it ends.
    if (phase_ == Phase::Blending) return;
    started_ = now;
    settled_since_ = -1;
    if (!available_) phase_ = Phase::Blocked;
    else if (phase_ == Phase::Preparing) return;
    else phase_ = selected == active_ ? Phase::Running : Phase::Stopping;
  }
  void tick(int64_t now, bool stationary, bool histories_ready) {
    if (phase_ == Phase::Ready && (!stationary || !histories_ready)) {
      phase_ = Phase::Stopping;
      started_ = now;
      settled_since_ = -1;
    }
    if (phase_ == Phase::Preparing) {
      if (histories_ready && now - started_ >= kWarmupUs) {
        phase_ = active_ == requested_ ? Phase::Running : Phase::Stopping;
        started_ = now;
      }
    } else if (phase_ == Phase::Stopping) {
      if (now - started_ >= kStopTimeoutUs) { phase_ = Phase::Blocked; return; }
      if (!stationary || !histories_ready) settled_since_ = -1;
      else if (settled_since_ < 0) settled_since_ = now;
      if (settled_since_ >= 0 && now - settled_since_ >= kSettledUs) {
        // The old state remains owner until ControlFSM enters the new state.
        phase_ = Phase::Ready;
      }
    }
  }
  bool activate(RLController controller, int64_t now) {
    if (phase_ != Phase::Ready || controller != requested_ || !available_) return false;
    active_ = controller;
    phase_ = Phase::Blending;
    started_ = now;
    return true;
  }
  void finishBlend(int64_t now, bool target_reached) {
    if (phase_ != Phase::Blending || now - started_ < kBlendUs || !target_reached) return;
    phase_ = !available_ ? Phase::Blocked :
        active_ == requested_ ? Phase::Running : Phase::Stopping;
    started_ = now;
    settled_since_ = -1;
  }
  float blend(int64_t now) const {
    if (phase_ != Phase::Blending) return 1.f;
    return std::min(1.f, std::max(0.f, float(now - started_) / kBlendUs));
  }
  bool zeroCommand() const { return phase_ != Phase::Running; }
  Phase phase() const { return phase_; }
  RLController active() const { return active_; }
  RLController requested() const { return requested_; }
  static const char* phaseName(Phase phase) {
    switch (phase) {
      case Phase::Preparing: return "preparing history";
      case Phase::Running: return "running";
      case Phase::Stopping: return "stopping";
      case Phase::Ready: return "ready for FSM transfer";
      case Phase::Blending: return "blending";
      case Phase::Blocked: return "blocked: zero command; press START/BACK to retry";
    }
    return "invalid";
  }

 private:
  RLController active_ = RLController::Stairs, requested_ = RLController::Stairs;
  Phase phase_ = Phase::Preparing;
  uint64_t sequence_ = 0;
  bool available_ = true;
  int64_t started_ = 0, settled_since_ = -1;
};
