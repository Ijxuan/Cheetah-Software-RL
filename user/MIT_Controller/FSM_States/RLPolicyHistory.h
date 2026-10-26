#pragma once

#include <array>
#include <cstdint>
#include <Eigen/Dense>

#include "RapidRLPolicyConfig.h"

namespace rapid_rl {
using JointVector = Eigen::Matrix<float, 12, 1>;
using FlatObservation = Eigen::Matrix<float, 141, 1>;

// Robot order throughout this shared record. Policy adapters own all scaling,
// ordering and action semantics. Targets are those actually sent to joint PD.
struct RLStateSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t time_us = 0;
  Eigen::Matrix3f r_body = Eigen::Matrix3f::Identity();
  Eigen::Vector3f omega = Eigen::Vector3f::Zero();
  Eigen::Vector3f command = Eigen::Vector3f::Zero();
  JointVector q = JointVector::Zero();
  JointVector qd = JointVector::Zero();
  JointVector feet = JointVector::Zero();
  JointVector applied_target = JointVector::Zero();

  bool finite() const {
    return r_body.allFinite() && omega.allFinite() && command.allFinite() &&
           q.allFinite() && qd.allFinite() && feet.allFinite() &&
           applied_target.allFinite();
  }
};

// Six actual 10 ms samples. No catch-up loop invents intermediate robot states.
// HIM has its own six-frame 20 ms history in OnnxPolicyRunner.
class FlatPolicyHistory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void reset() { count_ = 0; }
  void append(const RLStateSample& sample) {
    if (count_ && (sample.time_us <= samples_[0].time_us ||
                  sample.time_us - samples_[0].time_us > 30000)) reset();
    for (int i = 5; i > 0; --i) samples_[i] = samples_[i - 1];
    samples_[0] = sample;
    if (count_ < 6) ++count_;
  }
  bool ready() const { return count_ == 6; }
  const RLStateSample& latest() const { return samples_[0]; }
  FlatObservation observation() const {
    FlatObservation obs;
    const auto& s = samples_[0];
    // Exactly the legacy rBody.transpose().row(2).transpose(), not HIM gravity.
    obs.segment<3>(0) = s.r_body.col(2);
    obs.segment<12>(3) = s.q;
    obs.segment<3>(15) = s.omega;
    obs.segment<12>(18) = s.qd;
    obs.segment<12>(30) = s.applied_target;
    obs.segment<12>(42) = samples_[1].applied_target;
    // Legacy oldest-first six-slot history selects slots 0,2,4: ages 5,3,1.
    for (int i = 0; i < 3; ++i) {
      const auto& old = samples_[5 - 2 * i];
      obs.segment<12>(54 + 24 * i) = old.applied_target - old.q;
      obs.segment<12>(66 + 24 * i) = old.qd;
    }
    obs.segment<12>(126) = s.feet;
    obs.segment<3>(138) = s.command.norm() < 0.1f ? Eigen::Vector3f::Zero().eval() : s.command;
    return obs;
  }

 private:
  std::array<RLStateSample, 6> samples_;
  int count_ = 0;
};

inline JointVector AppliedTargetToHimAction(const JointVector& target_robot) {
  return ClipPolicyAction((RobotToPolicyOrder(target_robot) -
                           DefaultJointPositionPolicyOrder()) / kActionScale);
}
}  // namespace rapid_rl
