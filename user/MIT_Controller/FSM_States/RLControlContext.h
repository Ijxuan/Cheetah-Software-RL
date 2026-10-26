#ifndef RL_CONTROL_CONTEXT_H
#define RL_CONTROL_CONTEXT_H

#include <array>
#include <memory>
#include <functional>

#include <lcm-cpp.hpp>

#include "FSM_State.h"
#include "RapidRLPolicyConfig.h"
#include "RapidRLPolicyRunner.h"
#include "FlatPolicyRunner.h"
#include "RLPolicySwitch.h"
#include "rl_robot_state_lcmt.hpp"

// Shared data, runners, history and output protection. This is NOT an FSM state.
// Only an owning RL state may execute its fixed policy; activate() is called by
// the destination state's onEnter(), never by the history/switch preparation.
template <typename T>
class RLControlContext {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RLControlContext(ControlFSMData<T>* _controlFSMData,
                      bool publish_debug_state = true,
                      std::function<int64_t()> clock = {});
  ~RLControlContext();

  int64_t nowUs() const { return _clock(); }
  void enter(RLController controller);
  void leave(bool to_rl);
  void stop();
  void holdTarget();
  void runFlat() { runAtTime(RLController::Flat, nowUs()); }
  void runStairs() { runAtTime(RLController::Stairs, nowUs()); }
  void request(RLController controller, uint64_t sequence);
  bool transferReady(RLController controller) const;
  bool failed() const { return _emergencyStop; }
  int activeMode() const { return _switch.active() == RLController::Flat ? K_RL_FLAT : K_RL_STAIRS; }
  // Read-only diagnostics, also used by offline controller-flow tests.
  const RLPolicySwitch& policySwitch() const { return _switch; }
  const rapid_rl::FlatPolicyHistory& flatHistory() const { return _flatHistory; }
  const rapid_rl::OnnxPolicyRunner& stairsRunner() const { return *_onnxRunner; }

 private:
  using Vec3f = Eigen::Matrix<float, 3, 1>;
  using Vec12f = Eigen::Matrix<float, rapid_rl::kActionDim, 1>;

  void publishRobotState(int64_t now_us);
  void commandTarget(const Vec12f& target_q_robot);
  bool runOnnxPolicy(int64_t now_us);
  bool runFlatPolicy(int64_t now_us);
  bool controllerReady(RLController selected) const;
  bool stationaryFor(RLController selected) const;
  bool updateHistories(int64_t now_us);
  void runAtTime(RLController owner, int64_t now_us);
  void reportSwitch();
  bool acceptPolicyOutput(const Vec12f& action_policy,
                          const Vec12f& target_policy,
                          bool stop_on_reject);
  Vec12f readRobotQ() const;
  Vec12f readRobotQd() const;
  Vec3f readBaseAngularVelocity() const;
  Vec3f readVelocityCommand() const;
  Vec3f readProjectedGravity() const;
  bool isOrientationUnsafe() const;
  bool usesFlatSafety() const;
  void requestEmergencyStop();

  ControlFSMData<T>* _data;
  std::function<int64_t()> _clock;
  bool _sessionActive = false;
  lcm::LCM _stateLCM;
  rapid_rl::PdGainProfile _pdGainProfile = rapid_rl::PdGainProfile::kRealRobot;
  bool _policyReady = false;
  bool _publishDebugState = true;
  std::unique_ptr<rapid_rl::OnnxPolicyRunner> _onnxRunner;
  rapid_rl::FlatPolicyRunner _flatRunner;
  rapid_rl::FlatPolicyHistory _flatHistory;
  RLPolicySwitch _switch;
  RLPolicySwitch::Phase _reportedPhase = RLPolicySwitch::Phase::Blocked;
  RLController _reportedController = RLController::Stairs;
  int64_t _lastHistoryTimeUs = 0;
  int64_t _lastStairsHistoryTimeUs = 0;
  unsigned _historyTick = 0;
  int64_t _controlTimeUs = 0;
  bool _stairsObservationDue = false;
  bool _blendTargetReached = false;
  Vec12f _blendOrigin;
  RLController _blendFrom = RLController::Stairs;

  int64_t _stateSequence = 0;
  int64_t _lastStatePublishTimeUs = 0;
  int64_t _lastPolicyRunTimeUs = 0;
  int64_t _lastPolicyWarningTimeUs = 0;
  // Per-joint consecutive policy frames and peak error in the current streak.
  std::array<uint64_t, rapid_rl::kActionDim> _targetErrorCounts{};
  std::array<float, rapid_rl::kActionDim> _targetErrorPeaks{};

  Vec12f _lastActionPolicy;
  Vec12f _lastTargetQRobot;
  Vec12f _defaultQRobot;

  bool _emergencyStop = false;

  LegControllerCommand<float> _preCommands[4];
};

#endif  // RL_CONTROL_CONTEXT_H
