#ifndef FSM_STATE_RLJOINTPD_H
#define FSM_STATE_RLJOINTPD_H

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <lcm-cpp.hpp>

#include "FSM_State.h"
#include "RapidRLPolicyConfig.h"
#include "RapidRLPolicyRunner.h"
#include "rl_policy_cmd_lcmt.hpp"
#include "rl_robot_state_lcmt.hpp"

/**
 * LCM bridge state for rapid-locomotion policies.
 *
 * Supports both the Python+LCM bridge and the in-process LibTorch policy path.
 * This FSM state remains the only writer to the leg controller, so all final
 * safety checks stay in the C++ control process.
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

  void lcmThreadLoop();
  void handlePolicyLCM(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const rl_policy_cmd_lcmt* msg);
  void publishRobotState(int64_t now_us);
  void commandTarget(const Vec12f& target_q_robot);
  bool acceptLatestPolicyCommand(int64_t now_us);
  bool runLibtorchPolicy(int64_t now_us);
  bool acceptPolicyOutput(const Vec12f& action_policy,
                          const Vec12f& target_policy,
                          bool stop_on_reject);
  Vec12f readRobotQ() const;
  Vec12f readRobotQd() const;
  Vec3f readVelocityCommand() const;
  Vec3f readProjectedGravity() const;
  bool isOrientationUnsafe() const;

  lcm::LCM _stateLCM;
  lcm::LCM _policyLCM;
  std::thread _lcmThread;
  std::atomic<bool> _lcmThreadRunning;

  rapid_rl::PolicyRuntimeConfig _policyConfig;
  bool _policyReady = false;
  std::unique_ptr<rapid_rl::LibtorchPolicyRunner> _libtorchRunner;

  std::mutex _policyMutex;
  rl_policy_cmd_lcmt _latestPolicyCmd;
  bool _hasPolicyCmd = false;
  int64_t _latestPolicyReceiveTimeUs = 0;
  int64_t _lastAcceptedPolicySequence = -1;

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
