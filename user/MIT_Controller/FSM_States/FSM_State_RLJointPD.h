#ifndef FSM_STATE_RLJOINTPD_H
#define FSM_STATE_RLJOINTPD_H

#include <memory>

#include <lcm-cpp.hpp>

#include "FSM_State.h"
#include "RapidRLPolicyConfig.h"
#include "RapidRLPolicyRunner.h"
#include "rl_robot_state_lcmt.hpp"

/**
 * LibTorch state for the legged_gym Mini Cheetah policy.
 *
 * This FSM state remains the only writer to the leg controller, so all final
 * safety checks stay in the C++ control process. LCM is only used for optional
 * read-only robot state debugging.
 */
template <typename T>
class FSM_State_RLJointPD : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_RLJointPD(ControlFSMData<T>* _controlFSMData);
  ~FSM_State_RLJointPD();

  void onEnter();
  void run();
  FSM_StateName checkTransition();
  TransitionData<T> transition();
  void onExit();

 private:
  using Vec3f = Eigen::Matrix<float, 3, 1>;
  using Vec12f = Eigen::Matrix<float, rapid_rl::kActionDim, 1>;

  void publishRobotState(int64_t now_us);
  void commandTarget(const Vec12f& target_q_robot);
  bool runLibtorchPolicy(int64_t now_us);
  bool acceptPolicyOutput(const Vec12f& action_policy,
                          const Vec12f& target_policy,
                          bool stop_on_reject);
  Vec12f readRobotQ() const;
  Vec12f readRobotQd() const;
  Vec3f readBaseLinearVelocity() const;
  Vec3f readBaseAngularVelocity() const;
  Vec3f readVelocityCommand() const;
  Vec3f readProjectedGravity() const;
  bool isOrientationUnsafe() const;

  lcm::LCM _stateLCM;
  rapid_rl::PdGainProfile _pdGainProfile = rapid_rl::PdGainProfile::kRealRobot;
  bool _policyReady = false;
  std::unique_ptr<rapid_rl::LibtorchPolicyRunner> _libtorchRunner;

  int64_t _stateSequence = 0;
  int64_t _lastStatePublishTimeUs = 0;
  int64_t _lastPolicyRunTimeUs = 0;
  int64_t _lastPolicyWarningTimeUs = 0;

  Vec12f _lastActionPolicy;
  Vec12f _lastTargetQRobot;
  Vec12f _defaultQRobot;

  bool _emergencyStop = false;

  LegControllerCommand<float> _preCommands[4];
};

#endif  // FSM_STATE_RLJOINTPD_H
