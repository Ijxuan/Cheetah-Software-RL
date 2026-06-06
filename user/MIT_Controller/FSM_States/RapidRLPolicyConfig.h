#ifndef RAPID_RL_POLICY_CONFIG_H
#define RAPID_RL_POLICY_CONFIG_H

#include <array>

#include <eigen3/Eigen/Dense>

namespace rapid_rl {

constexpr int kObsDim = 42;
constexpr int kHistoryLength = 15;
constexpr int kObsHistoryDim = kObsDim * kHistoryLength;
constexpr int kLatentDim = 18;
constexpr int kActorInputDim = kObsDim + kLatentDim;
constexpr int kActionDim = 12;

constexpr float kPolicyDt = 0.02f;
constexpr float kStatePublishDt = 0.01f;
constexpr float kActionScale = 0.25f;
constexpr float kHipScaleReduction = 0.5f;
constexpr float kKp = 20.0f;
constexpr float kKd = 0.5f;
constexpr float kAbadHipKp = 40.0f;
constexpr float kAbadHipKd = 2.0f;
constexpr float kDofPosScale = 1.0f;
constexpr float kDofVelScale = 0.05f;
constexpr float kLinVelScale = 2.0f;
constexpr float kAngVelScale = 0.25f;

// Policy order follows the mini_cheetah DOF order observed in Isaac Gym:
// FL, FR, RL, RR; each leg is abad/hip, thigh, calf.
// The local controller order is FR, FL, RR, RL.
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

inline Eigen::Matrix<float, kActionDim, 1> ActionToTargetQPolicyOrder(
    const Eigen::Matrix<float, kActionDim, 1>& action) {
  Eigen::Matrix<float, kActionDim, 1> scaled = action * kActionScale;
  scaled[0] *= kHipScaleReduction;
  scaled[3] *= kHipScaleReduction;
  scaled[6] *= kHipScaleReduction;
  scaled[9] *= kHipScaleReduction;
  return DefaultJointPositionPolicyOrder() + scaled;
}

}  // namespace rapid_rl

#endif  // RAPID_RL_POLICY_CONFIG_H
