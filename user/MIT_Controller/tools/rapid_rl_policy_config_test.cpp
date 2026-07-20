#include <cmath>
#include <iostream>
#include <string>

#include "FSM_States/RapidRLPolicyConfig.h"

namespace {

bool near(float actual, float expected, float tolerance = 1e-6f) {
  return std::fabs(actual - expected) <= tolerance;
}

bool check(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "[LeggedGymRLConfigTest] FAIL: " << message << std::endl;
    return false;
  }
  return true;
}

}  // namespace

int main() {
  using Vec12f = Eigen::Matrix<float, rapid_rl::kActionDim, 1>;

  bool ok = true;
  ok &= check(rapid_rl::kObsDim == 48, "flat observation dimension");

  Vec12f robot_order;
  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    robot_order[i] = static_cast<float>(i);
  }
  const auto policy_order = rapid_rl::RobotToPolicyOrder(robot_order);
  const auto round_trip = rapid_rl::PolicyToRobotOrder(policy_order);
  ok &= check((round_trip - robot_order).cwiseAbs().maxCoeff() == 0.0f,
              "joint order round trip");
  ok &= check(near(policy_order[0], 3.0f) && near(policy_order[3], 0.0f) &&
                  near(policy_order[6], 9.0f) && near(policy_order[9], 6.0f),
              "FR/FL/RR/RL to FL/FR/RL/RR mapping");

  Eigen::Matrix<float, 3, 1> base_linear_velocity;
  base_linear_velocity << 1.0f, -2.0f, 60.0f;
  Eigen::Matrix<float, 3, 1> base_angular_velocity;
  base_angular_velocity << 4.0f, -8.0f, 400.0f;
  Eigen::Matrix<float, 3, 1> projected_gravity;
  projected_gravity << 0.1f, -0.2f, -1.0f;
  Eigen::Matrix<float, 3, 1> command;
  command << 2.0f, -2.0f, 2.0f;
  const auto q_default = rapid_rl::DefaultJointPositionPolicyOrder();
  Vec12f expected_q_default;
  expected_q_default << 0.1f, -0.8f, 1.62f,
                       -0.1f, -0.8f, 1.62f,
                        0.1f, -0.8f, 1.62f,
                       -0.1f, -0.8f, 1.62f;
  ok &= check((q_default - expected_q_default).cwiseAbs().maxCoeff() == 0.0f,
              "exact FL/FR/RL/RR default joint angles");
  Vec12f q_policy = q_default;
  Vec12f qd_policy;
  Vec12f last_action;
  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    q_policy[i] += 0.01f * static_cast<float>(i + 1);
    qd_policy[i] = static_cast<float>(i + 1);
    last_action[i] = 0.1f * static_cast<float>(i + 1);
  }

  const auto obs = rapid_rl::BuildObservationPolicyOrder(
      base_linear_velocity, base_angular_velocity, projected_gravity,
      command, q_policy, qd_policy, last_action);
  ok &= check(near(obs[0], 2.0f) && near(obs[1], -4.0f) &&
                  near(obs[2], 100.0f),
              "base linear velocity scale and observation clipping");
  ok &= check(near(obs[3], 1.0f) && near(obs[4], -2.0f) &&
                  near(obs[5], 100.0f),
              "base angular velocity scale and observation clipping");
  ok &= check(near(obs[6], 0.1f) && near(obs[7], -0.2f) &&
                  near(obs[8], -1.0f),
              "projected gravity segment");
  ok &= check(near(obs[9], 4.0f) && near(obs[10], -4.0f) &&
                  near(obs[11], 0.5f),
              "command scale without an extra deployment-side clamp");
  ok &= check(near(obs[12], 0.01f) && near(obs[23], 0.12f),
              "joint position segment");
  ok &= check(near(obs[24], 0.05f) && near(obs[35], 0.60f),
              "joint velocity segment");
  ok &= check(near(obs[36], 0.1f) && near(obs[47], 1.2f),
              "last action segment");
  Vec12f unit_action;
  unit_action.setOnes();
  const auto target = rapid_rl::ActionToTargetQPolicyOrder(unit_action);
  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    ok &= check(near(target[i], q_default[i] + 0.25f),
                "uniform action scale without hip reduction");
  }
  const auto training_abad = rapid_rl::JointPdGainsForProfile(
      rapid_rl::PdGainProfile::kTraining, 0);
  const auto training_thigh = rapid_rl::JointPdGainsForProfile(
      rapid_rl::PdGainProfile::kTraining, 1);
  const auto training_calf = rapid_rl::JointPdGainsForProfile(
      rapid_rl::PdGainProfile::kTraining, 2);
  ok &= check(near(training_abad.kp, 17.0f) &&
                  near(training_thigh.kp, 17.0f) &&
                  near(training_calf.kp, 34.0f) &&
                  near(training_abad.kd, 0.4f) &&
                  near(training_thigh.kd, 0.4f) &&
                  near(training_calf.kd, 0.8f),
              "training PD gains remain metadata only");

  const auto simulator_profile = rapid_rl::RuntimePdGainProfile(true);
  const auto real_robot_profile = rapid_rl::RuntimePdGainProfile(false);
  ok &= check(simulator_profile == rapid_rl::PdGainProfile::kSimulator,
              "simulator runtime profile selection");
  ok &= check(real_robot_profile == rapid_rl::PdGainProfile::kRealRobot,
              "real-robot runtime profile selection");
  for (int joint = 0; joint < 3; ++joint) {
    const auto simulator_gains =
        rapid_rl::JointPdGainsForProfile(simulator_profile, joint);
    const auto real_robot_gains =
        rapid_rl::JointPdGainsForProfile(real_robot_profile, joint);
    ok &= check(near(simulator_gains.kp, 20.0f) &&
                    near(simulator_gains.kd, 0.5f),
                "Cheetah simulator PD gains");
    ok &= check(near(real_robot_gains.kp, 20.0f) &&
                    near(real_robot_gains.kd, 0.5f),
                "validated real Mini Cheetah PD gains");
  }

  if (!ok) {
    return 1;
  }
  std::cout << "[LeggedGymRLConfigTest] PASS: obs=48 action=12 "
               "pd_profiles=training/simulator/real-robot"
            << std::endl;
  return 0;
}
