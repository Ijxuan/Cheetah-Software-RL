#ifndef FSM_STATE_RLJOINTPD_H
#define FSM_STATE_RLJOINTPD_H

#include <memory>

#include <lcm-cpp.hpp>

#include "FSM_State.h"
#include "RapidRLAsyncPolicyRunner.h"
#include "RapidRLPolicyConfig.h"
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

  struct PolicyTimingWindow {
    uint64_t count = 0;
    double queue_sum_ms = 0.0;
    double inference_sum_ms = 0.0;
    double delivery_sum_ms = 0.0;
    double total_sum_ms = 0.0;
    float queue_max_ms = 0.0f;
    float inference_max_ms = 0.0f;
    float delivery_max_ms = 0.0f;
    float total_max_ms = 0.0f;
  };

  struct PolicyTimingSample {
    bool valid = false;
    uint64_t sequence = 0;
    float queue_ms = 0.0f;
    float inference_ms = 0.0f;
    float delivery_ms = 0.0f;
    float total_ms = 0.0f;
    uint64_t overwritten_request_count = 0;
    int worker_cpu = -1;
    int control_cpu = -1;
  };

  void publishRobotState(int64_t now_us);
  void commandTarget(const Vec12f& target_q_robot);
  bool runAsyncPolicy(int64_t now_us);
  void recordPolicyTiming(const rapid_rl::AsyncPolicyResult& result,
                          int64_t now_us);
  void publishPolicyTimingSummary(int64_t now_us);
  void publishPolicyTimeoutDiagnostics(int64_t now_us);
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
  std::unique_ptr<rapid_rl::AsyncLibtorchPolicyRunner> _asyncPolicyRunner;

  int64_t _stateSequence = 0;
  int64_t _lastStatePublishTimeUs = 0;
  int64_t _lastPolicyRequestTimeUs = 0;
  int64_t _lastPolicyWarningTimeUs = 0;
  int64_t _lastValidPolicyResultTimeUs = 0;
  int64_t _lastPolicyTimeoutCheckTimeUs = 0;
  int64_t _lastPolicyTimingSummaryTimeUs = 0;
  uint64_t _policyEpoch = 0;
  uint64_t _lastPolicyResultSequence = 0;
  uint64_t _lastReportedOverwrittenRequests = 0;
  int _consecutivePolicyTimeouts = 0;
  PolicyTimingWindow _policyTimingWindow;
  PolicyTimingSample _lastPolicyTimingSample;

  Vec12f _lastActionPolicy;
  Vec12f _lastTargetQRobot;
  Vec12f _defaultQRobot;

  bool _emergencyStop = false;

  LegControllerCommand<float> _preCommands[4];
};

#endif  // FSM_STATE_RLJOINTPD_H
