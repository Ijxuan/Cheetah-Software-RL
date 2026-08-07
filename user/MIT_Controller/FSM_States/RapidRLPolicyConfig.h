#ifndef RAPID_RL_POLICY_CONFIG_H
#define RAPID_RL_POLICY_CONFIG_H

#include <array>

#include <eigen3/Eigen/Dense>

namespace rapid_rl {

constexpr int kCommandOffset = 0;
constexpr int kBaseAngularVelocityOffset = 3;
constexpr int kProjectedGravityOffset = 6;
constexpr int kDofPositionOffset = 9;
constexpr int kDofVelocityOffset = 21;
constexpr int kLastActionOffset = 33;
constexpr int kActionDim = 12;
constexpr int kOneStepObsDim = kLastActionOffset + kActionDim;
constexpr int kHistorySteps = 6;
constexpr int kObsDim = kOneStepObsDim * kHistorySteps;

constexpr float kPolicyDt = 0.02f;
constexpr float kStatePublishDt = kPolicyDt;
constexpr float kMaxCommandLinearX = 1.0f;
constexpr float kMaxCommandLinearY = 1.0f;
constexpr float kMaxCommandYaw = 3.14f;
// Must match MiniChRoughCfg.control.action_scale during training.
constexpr float kActionScale = 0.25f;

// PD gains belong to three different environments and must never be treated as
// one interchangeable setting.  The training profile is policy metadata only;
// RuntimePdGainProfile() can select only simulator or real-robot gains.
enum class PdGainProfile {
  kTraining,
  kSimulator,
  kRealRobot,
};

struct JointPdGains {
  float kp;
  float kd;
};

// Isaac Gym training gains.  These describe how the actor was trained, but are
// never sent to the Cheetah controller by RuntimePdGainProfile().
constexpr float kTrainingHipThighKp = 17.0f;
constexpr float kTrainingHipThighKd = 0.4f;
constexpr float kTrainingCalfKp = 34.0f;
constexpr float kTrainingCalfKd = 0.8f;

// Cheetah simulator gains.  Keep these as independent constants even while
// their current values match the real robot, so simulator tuning cannot alter
// the validated hardware profile.
constexpr float kSimulatorAbadKp = 20.0f;
constexpr float kSimulatorAbadKd = 0.5f;
constexpr float kSimulatorThighKp = 20.0f;
constexpr float kSimulatorThighKd = 0.5f;
constexpr float kSimulatorCalfKp = 20.0f;
constexpr float kSimulatorCalfKd = 0.5f;

// Real Mini Cheetah gains preserved from the previously deployed and validated
// RL_JOINT_PD controller.  Do not replace these with training/simulator gains.
constexpr float kRealRobotAbadKp = 20.0f;
constexpr float kRealRobotAbadKd = 0.5f;
constexpr float kRealRobotThighKp = 20.0f;
constexpr float kRealRobotThighKd = 0.5f;
constexpr float kRealRobotCalfKp = 20.0f;
constexpr float kRealRobotCalfKd = 0.5f;
constexpr float kDofPosScale = 1.0f;
constexpr float kDofVelScale = 0.05f;
constexpr float kLinVelScale = 2.0f;
constexpr float kAngVelScale = 0.25f;
constexpr float kClipObservations = 100.0f;
constexpr float kClipActions = 100.0f;

static_assert(kOneStepObsDim == 45,
              "HIMLoco Mini Cheetah policy expects 45 observations per step");
static_assert(kObsDim == 270,
              "HIMLoco Mini Cheetah policy expects six 45-D observation frames");

inline float Clamp(float value, float lower, float upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

// Isaac Gym 训练策略的实际顺序：左前腿、右前腿、左后腿、右后腿；
// Cheetah 控制器的实际顺序：右前腿、左前腿、右后腿、左后腿。
// 每条腿内部均为外展关节、大腿关节、小腿关节。
inline const std::array<int, kActionDim>& PolicyToRobotMap() {
  static const std::array<int, kActionDim> map = {
      3, 4, 5,
      0, 1, 2,
      9, 10, 11,
      6, 7, 8};
  return map;
}

inline const std::array<int, kActionDim>& RobotToPolicyMap() {
  static const std::array<int, kActionDim> map = {
      3, 4, 5,
      0, 1, 2,
      9, 10, 11,
      6, 7, 8};
  return map;
}

inline Eigen::Matrix<float, kActionDim, 1> DefaultJointPositionPolicyOrder() {
  Eigen::Matrix<float, kActionDim, 1> q;
  q <<  0.1f, -0.8f, 1.62f,
       -0.1f, -0.8f, 1.62f,
        0.1f, -0.8f, 1.62f,
       -0.1f, -0.8f, 1.62f;
  return q;
}

inline Eigen::Matrix<float, kActionDim, 1> RobotToPolicyOrder(
    const Eigen::Matrix<float, kActionDim, 1>& robot_order) {
  Eigen::Matrix<float, kActionDim, 1> policy_order;
  const auto& map = RobotToPolicyMap();
  for (int i = 0; i < kActionDim; ++i) {
    policy_order[i] = robot_order[map[i]];
  }
  return policy_order;
}

inline Eigen::Matrix<float, kActionDim, 1> PolicyToRobotOrder(
    const Eigen::Matrix<float, kActionDim, 1>& policy_order) {
  Eigen::Matrix<float, kActionDim, 1> robot_order;
  const auto& map = PolicyToRobotMap();
  for (int i = 0; i < kActionDim; ++i) {
    robot_order[map[i]] = policy_order[i];
  }
  return robot_order;
}

inline Eigen::Matrix<float, kActionDim, 1> ClipPolicyAction(
    const Eigen::Matrix<float, kActionDim, 1>& action) {
  Eigen::Matrix<float, kActionDim, 1> clipped;
  for (int i = 0; i < kActionDim; ++i) {
    clipped[i] = Clamp(action[i], -kClipActions, kClipActions);
  }
  return clipped;
}

inline Eigen::Matrix<float, kActionDim, 1> ActionToTargetQPolicyOrder(
    const Eigen::Matrix<float, kActionDim, 1>& action) {
  return DefaultJointPositionPolicyOrder() +
         ClipPolicyAction(action) * kActionScale;
}

inline PdGainProfile RuntimePdGainProfile(bool simulated) {
  return simulated ? PdGainProfile::kSimulator : PdGainProfile::kRealRobot;
}

inline const char* PdGainProfileName(PdGainProfile profile) {
  switch (profile) {
    case PdGainProfile::kTraining:
      return "training-metadata";
    case PdGainProfile::kSimulator:
      return "cheetah-simulator";
    case PdGainProfile::kRealRobot:
      return "real-mini-cheetah";
  }
  return "unknown";
}

inline JointPdGains JointPdGainsForProfile(PdGainProfile profile,
                                           int policy_index) {
  const int joint = policy_index % 3;
  switch (profile) {
    case PdGainProfile::kTraining:
      return joint == 2
                 ? JointPdGains{kTrainingCalfKp, kTrainingCalfKd}
                 : JointPdGains{kTrainingHipThighKp, kTrainingHipThighKd};
    case PdGainProfile::kSimulator:
      if (joint == 0) {
        return JointPdGains{kSimulatorAbadKp, kSimulatorAbadKd};
      }
      if (joint == 1) {
        return JointPdGains{kSimulatorThighKp, kSimulatorThighKd};
      }
      return JointPdGains{kSimulatorCalfKp, kSimulatorCalfKd};
    case PdGainProfile::kRealRobot:
      if (joint == 0) {
        return JointPdGains{kRealRobotAbadKp, kRealRobotAbadKd};
      }
      if (joint == 1) {
        return JointPdGains{kRealRobotThighKp, kRealRobotThighKd};
      }
      return JointPdGains{kRealRobotCalfKp, kRealRobotCalfKd};
  }
  return JointPdGains{0.0f, 0.0f};
}

inline Eigen::Matrix<float, kOneStepObsDim, 1>
BuildOneStepObservationPolicyOrder(
    const Eigen::Matrix<float, 3, 1>& base_angular_velocity,
    const Eigen::Matrix<float, 3, 1>& projected_gravity,
    const Eigen::Matrix<float, 3, 1>& command,
    const Eigen::Matrix<float, kActionDim, 1>& q_policy,
    const Eigen::Matrix<float, kActionDim, 1>& qd_policy,
    const Eigen::Matrix<float, kActionDim, 1>& last_action) {
  Eigen::Matrix<float, kOneStepObsDim, 1> obs;
  obs.template segment<3>(kCommandOffset) =
      command.cwiseProduct(Eigen::Matrix<float, 3, 1>(
          kLinVelScale, kLinVelScale, kAngVelScale));
  obs.template segment<3>(kBaseAngularVelocityOffset) =
      base_angular_velocity * kAngVelScale;
  obs.template segment<3>(kProjectedGravityOffset) = projected_gravity;
  obs.template segment<kActionDim>(kDofPositionOffset) =
      (q_policy - DefaultJointPositionPolicyOrder()) * kDofPosScale;
  obs.template segment<kActionDim>(kDofVelocityOffset) =
      qd_policy * kDofVelScale;
  obs.template segment<kActionDim>(kLastActionOffset) =
      ClipPolicyAction(last_action);

  for (int i = 0; i < kOneStepObsDim; ++i) {
    obs[i] = Clamp(obs[i], -kClipObservations, kClipObservations);
  }
  return obs;
}

}  // namespace rapid_rl

#endif  // RAPID_RL_POLICY_CONFIG_H
