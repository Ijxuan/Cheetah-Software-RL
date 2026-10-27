/*=================== legged_gym RL Joint PD Bridge ===================*/

#include "RLControlContext.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <Configuration.h>
#include <rt/rt_rc_interface.h>
#include "RapidRLBuildConfig.h"
#include "Utilities/utilities.h"
#include "main_helper.h"

namespace {

// 发布只读 RL 调试状态的周期；与策略周期保持一致。
constexpr int64_t kStatePublishDtUs =
    static_cast<int64_t>(rapid_rl::kStatePublishDt * 1000000.0f);
// 单次推理超过 100 ms 视为控制失效，立即关闭 RL 输出。
constexpr int64_t kPolicyTimeoutUs = 100000;
// 慢推理告警的限频周期，避免控制台被重复日志淹没。
constexpr int64_t kPolicyWarningPeriodUs = 500000;
// 相邻两次策略输出对应的关节目标最多变化 0.80 rad，只做限幅而不急停。
constexpr float kMaxTargetDeltaPerPolicyStep = 0.80f;
// 新目标与实测角偏差超过 1.20 rad 时仅统计并告警，不裁剪偏差或急停。
constexpr float kMaxTargetCurrentError = 1.20f;
// 每个关节恢复正常时汇报连续超限段；持续超限每 50 个策略帧汇报一次。
constexpr uint64_t kTargetErrorReportFrames = 50;
// 策略关节顺序中 abad（外展）关节的允许目标角范围，单位 rad。
constexpr float kMinAbad = -1.5f;
constexpr float kMaxAbad = 1.5f;
// 策略关节顺序中 hip/thigh（髋/大腿）关节的允许目标角范围，单位 rad。
constexpr float kMinHip = -5.0f;
constexpr float kMaxHip = 5.0f;
// 策略关节顺序中 knee/calf（膝/小腿）关节的允许目标角范围，单位 rad。
constexpr float kMinKnee = -3.0f;
constexpr float kMaxKnee = 3.0f;
// 机身 roll 或 pitch 超过 45 度即急停；yaw 不参与该安全判断。
constexpr float kMaxRollPitchDeg = 45.0f;
constexpr float kMaxRollPitchRad =
    kMaxRollPitchDeg * static_cast<float>(M_PI) / 180.0f;
// Handover allows translation and leg motion; only body rotation must settle.
constexpr float kMaxHandoverAngularSpeed = .2f;  // rad/s
// Preserve the legacy flat policy's total body-tilt limit, including blending.
constexpr float kMaxFlatTiltDeg = 30.f;
constexpr float kMaxFlatTiltRad = kMaxFlatTiltDeg * static_cast<float>(M_PI) / 180.f;

// 返回不受系统时间调整影响的单调微秒时间，供策略节拍和超时判断共用。
int64_t monotonicTimeUs() {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<int64_t>(now.tv_sec) * 1000000LL +
         static_cast<int64_t>(now.tv_nsec) / 1000LL;
}

// infer() 返回毫秒；转换为微秒后与硬超时阈值比较。
bool shouldHardStopForInference(float inference_time_ms) {
  return inference_time_ms * 1000.0f > static_cast<float>(kPolicyTimeoutUs);
}

// policy_index 的每三个元素是一条腿，故 index % 3 分别对应
// abad、hip/thigh、knee/calf 三种关节。
float jointLowerLimit(int policy_index) {
  const int joint = policy_index % 3;
  if (joint == 0) return kMinAbad;
  if (joint == 1) return kMinHip;
  return kMinKnee;
}

float jointUpperLimit(int policy_index) {
  const int joint = policy_index % 3;
  if (joint == 0) return kMaxAbad;
  if (joint == 1) return kMaxHip;
  return kMaxKnee;
}

// 仿真 GUI 的通用参数表不限制数值范围。非法增益不能写到关节 PD，
// 因此 NaN、Inf 或负值会回退到已验证的 simulator profile 默认值。
float simulationPdGainOrFallback(double requested_gain, float fallback_gain) {
  const float gain = static_cast<float>(requested_gain);
  return std::isfinite(gain) && gain >= 0.0f ? gain : fallback_gain;
}

}  // namespace

template <typename T>
RLControlContext<T>::RLControlContext(
    ControlFSMData<T>* _controlFSMData, bool publish_debug_state,
    std::function<int64_t()> clock)
    : _data(_controlFSMData), _clock(clock ? std::move(clock) : monotonicTimeUs),
      _stateLCM(getLcmUrl(255)) {
  _publishDebugState = publish_debug_state && rapid_rl::build_config::kDebugLcmState;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;
  std::cout << "Setup legged_gym ONNX Runtime Joint PD" << std::endl;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;

  _lastActionPolicy.setZero();
  _defaultQRobot = rapid_rl::PolicyToRobotOrder(
      rapid_rl::DefaultJointPositionPolicyOrder());
  _lastTargetQRobot = _defaultQRobot;

  _pdGainProfile = rapid_rl::RuntimePdGainProfile(gMasterConfig.simulated);
  const auto abad_gains =
      rapid_rl::JointPdGainsForProfile(_pdGainProfile, 0);
  const auto thigh_gains =
      rapid_rl::JointPdGainsForProfile(_pdGainProfile, 1);
  const auto calf_gains =
      rapid_rl::JointPdGainsForProfile(_pdGainProfile, 2);
  std::cout << "[LeggedGymRL] PD gain profile: "
            << rapid_rl::PdGainProfileName(_pdGainProfile)
            << ", Kp=[" << abad_gains.kp << ", " << thigh_gains.kp << ", "
            << calf_gains.kp << "], Kd=[" << abad_gains.kd << ", "
            << thigh_gains.kd << ", " << calf_gains.kd << "]" << std::endl;

  if (_publishDebugState && !_stateLCM.good()) {
    std::cout << "[LeggedGymRL] WARNING: failed to initialize debug LCM"
              << std::endl;
  }

  _onnxRunner.reset(new rapid_rl::OnnxPolicyRunner());
  _policyReady = _onnxRunner->load();
  if (!_policyReady) {
    std::cout << "[LeggedGymRL] ERROR: failed to load ONNX Runtime policy: "
              << _onnxRunner->error() << std::endl;
  }
  if (_policyReady) {
    std::cout << "[LeggedGymRL] Loaded ONNX Runtime policy: "
              << _onnxRunner->policyPath() << std::endl;
    std::cout << "[LeggedGymRL] Shapes: input "
              << _onnxRunner->inputShape() << ", action "
              << _onnxRunner->actionShape() << std::endl;
    std::cout << "[LeggedGymRL] ONNX Runtime CPU threads: "
              << rapid_rl::build_config::kOnnxNumThreads << std::endl;
  }
  if (!_flatRunner.load(rapid_rl::build_config::kFlatModelDir)) {
    std::cout << "[DualRL] Flat policy load failed: " << _flatRunner.error() << std::endl;
  } else {
    std::cout << "[DualRL] Loaded flat actor/estimator_5000 from "
              << rapid_rl::build_config::kFlatModelDir << std::endl;
  }
}

template <typename T>
RLControlContext<T>::~RLControlContext() {}

template <typename T>
void RLControlContext<T>::requestEmergencyStop() {
  _emergencyStop = true;
  // Stop this control cycle before RobotRunner reads RC OFF on the next one.
  this->_data->_legController->setEnabled(false);
  request_rc_off();
  // Also leave RL when RC input is disabled (e.g. simulator GUI control).
  this->_data->controlParameters->control_mode = K_PASSIVE;
}

template <typename T>
void RLControlContext<T>::enter(RLController controller) {
  const int64_t now_us = nowUs();
  if (_sessionActive) {
    if (!transferReady(controller)) {
      std::cout << "[DualRL] Invalid FSM handover; requesting OFF" << std::endl;
      requestEmergencyStop();
      return;
    }
    _blendOrigin = _lastTargetQRobot;
    _blendFrom = _switch.active();
    _blendTargetReached = false;
    _switch.activate(controller, now_us);
    _targetErrorCounts.fill(0);
    _targetErrorPeaks.fill(0.f);
    if (controller == RLController::Stairs) {
      Vec4<float> phase = Vec4<float>::Constant(.5f);
      _data->_stateEstimator->setContactPhase(phase);
    }
    holdTarget();
    reportSwitch();
    return;
  }
  _sessionActive = true;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;
  std::cout << "Start legged_gym ONNX Runtime Joint PD" << std::endl;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;

  _stateSequence = 0;
  _lastStatePublishTimeUs = 0;
  _lastPolicyRunTimeUs = 0;
  _lastHistoryTimeUs = _lastStairsHistoryTimeUs = 0;
  _historyTick = 0;
  _flatHistory.reset();
  const auto* rc = this->_data->_desiredStateCommand->rcCommand;
  const bool use_rc = this->_data->controlParameters->use_rc;
  _switch.reset(controller, use_rc ? rc->rl_selection_sequence : 0, now_us);
  _controlTimeUs = now_us;
  _reportedPhase = RLPolicySwitch::Phase::Blocked;
  _blendTargetReached = false;
  _lastPolicyWarningTimeUs = 0;
  _targetErrorCounts.fill(0);
  _targetErrorPeaks.fill(0.0f);
  _lastActionPolicy.setZero();
  _lastTargetQRobot = readRobotQ();
  if (!_lastTargetQRobot.allFinite() || _lastTargetQRobot.norm() < 1e-4f) {
    _lastTargetQRobot = _defaultQRobot;
  }

  for (int leg = 0; leg < 4; ++leg) {
    _preCommands[leg].zero();
  }
  if (_onnxRunner) {
    _onnxRunner->resetObservationHistory();
  }

  _emergencyStop = false;
  if (!controllerReady(_switch.active())) {
    std::cout << "[DualRL] Selected policy is not ready; refusing RL control"
              << std::endl;
    requestEmergencyStop();
  }
  holdTarget();
}

template <typename T>
void RLControlContext<T>::runAtTime(RLController owner, int64_t now_us) {
  if (!_sessionActive || owner != _switch.active()) {
    requestEmergencyStop();
    return;
  }
  _controlTimeUs = now_us;

  if (isOrientationUnsafe()) {
    if (!_emergencyStop) {
      const auto& rpy = this->_data->_stateEstimator->getResult().rpy;
      constexpr float kRadToDeg = 180.0f / static_cast<float>(M_PI);
      std::cout << "[LeggedGymRL] Orientation is unsafe; disabling RL bridge"
                << "; roll=" << rpy[0] * kRadToDeg << " deg"
                << (std::fabs(rpy[0]) > kMaxRollPitchRad ? " [EXCEEDED]" : "")
                << ", pitch=" << rpy[1] * kRadToDeg << " deg"
                << (std::fabs(rpy[1]) > kMaxRollPitchRad ? " [EXCEEDED]" : "")
                << ", limit=+/-" << kMaxRollPitchDeg << " deg";
      if (usesFlatSafety()) {
        const float cos_tilt = this->_data->_stateEstimator->getResult().rBody(2, 2);
        const float tilt = std::acos(std::max(-1.f, std::min(1.f, cos_tilt)));
        std::cout << ", flat_tilt=" << tilt * kRadToDeg << " deg"
                  << ", flat_tilt_limit=" << kMaxFlatTiltDeg << " deg"
                  << (tilt > kMaxFlatTiltRad ? " [EXCEEDED]" : "");
      }
      std::cout << std::endl;
    }
    requestEmergencyStop();
  }

  if (_emergencyStop) {
    this->_data->_legController->setEnabled(false);
    return;
  }

  const bool sample_due = now_us - _lastHistoryTimeUs >= 10000;
  if (sample_due && !updateHistories(now_us)) {
    requestEmergencyStop();
    return;
  }
  _switch.tick(now_us, stationaryFor(_switch.requested()),
               _flatHistory.ready() && _onnxRunner->historyReady());
  // A prepared observation was recorded with zero command this cycle. Keep it
  // until the next policy frame, then resume the newest live RC/nav command.
  if (sample_due && _switch.phase() != RLPolicySwitch::Phase::Preparing &&
      (_switch.active() == RLController::Flat ? _flatHistory.ready() : _onnxRunner->historyReady()) &&
      (owner == RLController::Flat || _stairsObservationDue)) {
    if (owner == RLController::Flat) runFlatPolicy(now_us);
    else runOnnxPolicy(now_us);
    _lastPolicyRunTimeUs = now_us;
    if (!_emergencyStop) _switch.finishBlend(now_us, _blendTargetReached);
  }
  reportSwitch();
  if (_emergencyStop) {
    this->_data->_legController->setEnabled(false);
    return;
  }
  commandTarget(_lastTargetQRobot);
  if (_publishDebugState &&
      now_us - _lastStatePublishTimeUs >= kStatePublishDtUs) {
    publishRobotState(now_us);
    _lastStatePublishTimeUs = now_us;
  }
}

// The RobotRunner already ran the state estimator once. Neither policy runs it
// again; the legacy contact hint is applied for the next estimator cycle only.
template <typename T>
bool RLControlContext<T>::updateHistories(int64_t now_us) {
  rapid_rl::RLStateSample sample;
  const auto& estimate = this->_data->_stateEstimator->getResult();
  sample.time_us = now_us;
  sample.r_body = estimate.rBody;
  sample.omega = estimate.omegaBody;
  sample.q = readRobotQ();
  sample.qd = readRobotQd();
  sample.applied_target = _lastTargetQRobot;
  sample.command = readVelocityCommand();
  for (int leg = 0; leg < 4; ++leg) {
    sample.feet.segment<3>(leg * 3) = this->_data->_quadruped->getHipLocation(leg) +
        this->_data->_legController->datas[leg].p;
  }
  if (!sample.finite()) {
    std::cout << "[DualRL] Non-finite shared robot state; requesting OFF" << std::endl;
    return false;
  }
  if (_lastHistoryTimeUs && now_us - _lastHistoryTimeUs > 30000) {
    _historyTick = 0;
    _onnxRunner->resetObservationHistory();
  }
  _flatHistory.append(sample);
  _stairsObservationDue = (_historyTick++ % 2 == 0);
  _lastHistoryTimeUs = now_us;
  if (_stairsObservationDue) {
    if (_lastStairsHistoryTimeUs && now_us - _lastStairsHistoryTimeUs > 60000)
      _onnxRunner->resetObservationHistory();
    // Preserve HIM's own raw previous action in normal HIM operation. While
    // inactive or handing over, reconstruct from the target actually applied.
    const Vec12f last_action = _switch.active() == RLController::Stairs &&
        _switch.phase() != RLPolicySwitch::Phase::Preparing &&
        _switch.phase() != RLPolicySwitch::Phase::Blending ? _lastActionPolicy :
        rapid_rl::AppliedTargetToHimAction(sample.applied_target);
    std::string error;
    if (!_onnxRunner->observe(sample.omega, readProjectedGravity(), sample.command,
          rapid_rl::RobotToPolicyOrder(sample.q), rapid_rl::RobotToPolicyOrder(sample.qd),
          last_action, &error)) {
      std::cout << "[DualRL] HIM observation failed: " << error << std::endl;
      return false;
    }
    _lastStairsHistoryTimeUs = now_us;
  }
  if (_switch.active() == RLController::Flat) {
    Vec4<float> phase;
    for (int leg = 0; leg < 4; ++leg)
      phase[leg] = sample.applied_target[leg * 3 + 2] - sample.q[leg * 3 + 2] < 0.f;
    this->_data->_stateEstimator->setContactPhase(phase);
  }
  return true;
}

template <typename T>
bool RLControlContext<T>::controllerReady(RLController selected) const {
  return selected == RLController::Flat ? _flatRunner.ready() :
      selected == RLController::Stairs && _policyReady;
}

template <typename T>
bool RLControlContext<T>::stationaryFor(RLController selected) const {
  const auto& estimate = _data->_stateEstimator->getResult();
  const Vec12f qd = readRobotQd();
  return (selected != RLController::Flat || estimate.rBody(2, 2) >= std::cos(kMaxFlatTiltRad)) &&
      estimate.vBody.allFinite() && estimate.omegaBody.allFinite() && qd.allFinite() &&
      estimate.omegaBody.norm() < kMaxHandoverAngularSpeed;
}

template <typename T>
void RLControlContext<T>::request(RLController controller, uint64_t sequence) {
  _switch.request(controller, sequence, nowUs(), controllerReady(controller));
}

template <typename T>
bool RLControlContext<T>::transferReady(RLController controller) const {
  const int64_t now = nowUs();
  return !_emergencyStop && _sessionActive && controllerReady(controller) &&
      _switch.phase() == RLPolicySwitch::Phase::Ready && _switch.requested() == controller &&
      !isOrientationUnsafe() && stationaryFor(controller) &&
      now >= _lastHistoryTimeUs && now - _lastHistoryTimeUs <= 30000 &&
      now >= _lastStairsHistoryTimeUs && now - _lastStairsHistoryTimeUs <= 60000;
}

template <typename T>
void RLControlContext<T>::holdTarget() {
  _controlTimeUs = nowUs();
  if (_emergencyStop) _data->_legController->setEnabled(false);
  else commandTarget(_lastTargetQRobot);
}

template <typename T>
void RLControlContext<T>::leave(bool to_rl) {
  if (to_rl && !_emergencyStop) holdTarget();
  else stop();
}

template <typename T>
void RLControlContext<T>::reportSwitch() {
  if (_reportedPhase == _switch.phase() && _reportedController == _switch.active()) return;
  _reportedPhase = _switch.phase();
  _reportedController = _switch.active();
  std::cout << "[DualRL] active=" << rlControllerName(_switch.active())
            << " requested=" << rlControllerName(_switch.requested())
            << " state=" << RLPolicySwitch::phaseName(_switch.phase()) << std::endl;
}

template <typename T>
void RLControlContext<T>::stop() {
  _sessionActive = false;
  this->_data->_legController->setEnabled(false);
  _flatHistory.reset();
  _onnxRunner->resetObservationHistory();
  // Do not leave the legacy flat contact hint latched after RL exit.
  Vec4<float> phase = Vec4<float>::Constant(.5f);
  this->_data->_stateEstimator->setContactPhase(phase);
}

template <typename T>
void RLControlContext<T>::publishRobotState(int64_t now_us) {
  if (!_stateLCM.good()) {
    return;
  }

  const Vec12f q_policy = rapid_rl::RobotToPolicyOrder(readRobotQ());
  const Vec12f qd_policy = rapid_rl::RobotToPolicyOrder(readRobotQd());
  const Vec3f command = readVelocityCommand();
  const Vec3f projected_gravity = readProjectedGravity();

  rl_robot_state_lcmt state;
  state.timestamp_us = now_us;
  state.sequence = ++_stateSequence;
  state.mode = activeMode();

  for (int i = 0; i < 3; ++i) {
    state.projected_gravity[i] = projected_gravity[i];
    state.command[i] = command[i];
  }

  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    state.q[i] = q_policy[i];
    state.qd[i] = qd_policy[i];
    state.last_action[i] = _lastActionPolicy[i];
  }

  _stateLCM.publish("rl_robot_state", &state);
}

template <typename T>
void RLControlContext<T>::commandTarget(const Vec12f& target_q_robot) {
  Mat3<float> kp = Mat3<float>::Zero();
  Mat3<float> kd = Mat3<float>::Zero();
  // 仅仿真允许 GUI 覆盖 RL 增益；实机始终保留经验证的 real-robot profile，
  // 避免通用 User Control Parameters 通道意外改变硬件关节增益。
  const bool use_simulation_gui_gains =
      _pdGainProfile == rapid_rl::PdGainProfile::kSimulator &&
      this->_data->userParameters != nullptr;
  for (int joint = 0; joint < 3; ++joint) {
    auto gains =
        rapid_rl::JointPdGainsForProfile(_pdGainProfile, joint);
    if (use_simulation_gui_gains) {
      // 每个低层控制回合重读一次，仿真界面修改后无需等待下一次 20 ms 策略推理。
      gains.kp = simulationPdGainOrFallback(
          this->_data->userParameters->rl_kp_joint[joint], gains.kp);
      gains.kd = simulationPdGainOrFallback(
          this->_data->userParameters->rl_kd_joint[joint], gains.kd);
    }
    const float stairs_kp = gains.kp, stairs_kd = gains.kd;
    if (_switch.active() == RLController::Flat) { gains.kp = 17.f; gains.kd = .4f; }
    if (_switch.phase() == RLPolicySwitch::Phase::Blending) {
      const float alpha = _switch.blend(_controlTimeUs);
      const float old_kp = _blendFrom == RLController::Flat ? 17.f : stairs_kp;
      const float old_kd = _blendFrom == RLController::Flat ? .4f : stairs_kd;
      gains.kp = old_kp + alpha * (gains.kp - old_kp);
      gains.kd = old_kd + alpha * (gains.kd - old_kd);
    }
    kp(joint, joint) = gains.kp;
    kd(joint, joint) = gains.kd;
  }

  this->_data->_legController->setEnabled(true);

  for (int leg = 0; leg < 4; ++leg) {
    for (int joint = 0; joint < 3; ++joint) {
      const int idx = leg * 3 + joint;
      this->_data->_legController->commands[leg].qDes[joint] =
          target_q_robot[idx];
      this->_data->_legController->commands[leg].qdDes[joint] = 0.0f;
      this->_data->_legController->commands[leg].tauFeedForward[joint] = 0.0f;
    }
    this->_data->_legController->commands[leg].kpJoint = kp;
    this->_data->_legController->commands[leg].kdJoint = kd;
    _preCommands[leg] = this->_data->_legController->commands[leg];
  }
}

template <typename T>
bool RLControlContext<T>::runOnnxPolicy(int64_t now_us) {
  if (!_onnxRunner || !_onnxRunner->ready()) {
    std::cout << "[LeggedGymRL] ONNX Runtime policy is not ready; disabling RL"
              << std::endl;
    requestEmergencyStop();
    return false;
  }

  Vec12f action_policy;
  Vec12f target_policy;
  float inference_time_ms = 0.0f;
  std::string error;
  if (!_onnxRunner->inferFromHistory(&action_policy, &target_policy,
          &inference_time_ms, &error)) {
    std::cout << "[LeggedGymRL] ONNX Runtime inference failed: " << error
              << std::endl;
    requestEmergencyStop();
    return false;
  }

  rapid_rl::InferenceTimingSummary timing;
  if (_onnxRunner->timingSummary(&timing) &&
      (timing.count == 1 || timing.count == 50 || timing.count % 500 == 0)) {
    std::cout << "[LeggedGymRL] ONNX Runtime timing count=" << timing.count
              << " min/mean/p95/max=" << timing.min_ms << "/"
              << timing.mean_ms << "/" << timing.p95_ms << "/"
              << timing.max_ms << " ms" << std::endl;
  }

  if (shouldHardStopForInference(inference_time_ms)) {
    std::cout << "[LeggedGymRL] ONNX Runtime inference exceeded hard timeout: "
              << inference_time_ms << " ms" << std::endl;
    requestEmergencyStop();
    return false;
  }
  if (inference_time_ms > rapid_rl::build_config::kMaxInferenceMs &&
      now_us - _lastPolicyWarningTimeUs > kPolicyWarningPeriodUs) {
    std::cout << "[LeggedGymRL] WARNING: ONNX Runtime inference took "
              << inference_time_ms << " ms" << std::endl;
    _lastPolicyWarningTimeUs = now_us;
  }

  return acceptPolicyOutput(action_policy, target_policy, true);
}

template <typename T>
bool RLControlContext<T>::runFlatPolicy(int64_t now_us) {
  if (!_flatHistory.ready()) return true;
  Vec12f target_robot;
  float elapsed_ms = 0.f;
  std::string error;
  if (!_flatRunner.infer(_flatHistory.observation(), &target_robot, &elapsed_ms, &error) ||
      shouldHardStopForInference(elapsed_ms)) {
    std::cout << "[DualRL] Flat inference failed/timed out: " << error
              << " elapsed_ms=" << elapsed_ms << std::endl;
    requestEmergencyStop();
    return false;
  }
  if (elapsed_ms > 10.f && now_us - _lastPolicyWarningTimeUs > kPolicyWarningPeriodUs) {
    std::cout << "[DualRL] WARNING: flat inference took " << elapsed_ms << " ms" << std::endl;
    _lastPolicyWarningTimeUs = now_us;
  }
  return acceptPolicyOutput(rapid_rl::AppliedTargetToHimAction(target_robot),
                            rapid_rl::RobotToPolicyOrder(target_robot), true);
}

template <typename T>
bool RLControlContext<T>::acceptPolicyOutput(
    const Vec12f& action_policy, const Vec12f& target_policy,
    bool stop_on_reject) {
  auto reject = [&](const std::string& reason) {
    std::cout << "[LeggedGymRL] Rejecting policy output: " << reason
              << std::endl;
    if (stop_on_reject) {
      requestEmergencyStop();
    }
    return false;
  };

  if (!action_policy.allFinite() || !target_policy.allFinite()) {
    return reject("non-finite action or target");
  }

  Vec12f clamped_target_policy = target_policy;
  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    const float lower = jointLowerLimit(i);
    const float upper = jointUpperLimit(i);
    if (target_policy[i] < lower || target_policy[i] > upper) {
      clamped_target_policy[i] =
          std::max(lower, std::min(upper, target_policy[i]));
      std::cout << "[LeggedGymRL] WARNING: target outside joint limits at policy index "
                << i << ": target=" << target_policy[i]
                << " rad, limits=[" << lower << ", " << upper
                << "] rad, clamped=" << clamped_target_policy[i]
                << " rad; should be disable" << std::endl;
    }
  }

  const Vec12f target_robot =
      rapid_rl::PolicyToRobotOrder(clamped_target_policy);
  const Vec12f current_robot = readRobotQ();
  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    const float error = std::fabs(target_robot[i] - current_robot[i]);
    const bool exceeded = error > kMaxTargetCurrentError;
    if (exceeded) {
      ++_targetErrorCounts[i];
      _targetErrorPeaks[i] = std::max(_targetErrorPeaks[i], error);
    }
    if (_targetErrorCounts[i] > 0 &&
        (!exceeded || _targetErrorCounts[i] % kTargetErrorReportFrames == 0)) {
      std::cout << "[LeggedGymRL] WARNING: target too far from current joint "
                << i << ": status=" << (exceeded ? "ongoing" : "recovered")
                << ", consecutive_frames=" << _targetErrorCounts[i]
                << ", peak_error=" << _targetErrorPeaks[i]
                << " rad, target=" << target_robot[i]
                << " rad, current=" << current_robot[i]
                << " rad, abs_error=" << error
                << " rad, limit=" << kMaxTargetCurrentError
                << " rad; should be disable (warning only)" << std::endl;
    }
    if (!exceeded) {
      _targetErrorCounts[i] = 0;
      _targetErrorPeaks[i] = 0.0f;
    }
  }

  Vec12f proposed_robot = target_robot;
  const bool blending = _switch.phase() == RLPolicySwitch::Phase::Blending;
  if (blending) {
    const float alpha = _switch.blend(_controlTimeUs);
    proposed_robot = _blendOrigin + alpha * (target_robot - _blendOrigin);
  }
  const float limit = blending ? .05f : kMaxTargetDeltaPerPolicyStep;
  _blendTargetReached = (proposed_robot - _lastTargetQRobot).cwiseAbs().maxCoeff() <= limit;
  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    const float delta = proposed_robot[i] - _lastTargetQRobot[i];
    _lastTargetQRobot[i] += std::max(-limit, std::min(limit, delta));
  }

  _lastActionPolicy = action_policy;
  return true;
}

template <typename T>
typename RLControlContext<T>::Vec12f RLControlContext<T>::readRobotQ()
    const {
  Vec12f q;
  for (int leg = 0; leg < 4; ++leg) {
    for (int joint = 0; joint < 3; ++joint) {
      q[leg * 3 + joint] =
          this->_data->_legController->datas[leg].q[joint];
    }
  }
  return q;
}

template <typename T>
typename RLControlContext<T>::Vec12f RLControlContext<T>::readRobotQd()
    const {
  Vec12f qd;
  for (int leg = 0; leg < 4; ++leg) {
    for (int joint = 0; joint < 3; ++joint) {
      qd[leg * 3 + joint] =
          this->_data->_legController->datas[leg].qd[joint];
    }
  }
  return qd;
}

template <typename T>
typename RLControlContext<T>::Vec3f
RLControlContext<T>::readBaseAngularVelocity() const {
  return this->_data->_stateEstimator->getResult().omegaBody;
}

template <typename T>
typename RLControlContext<T>::Vec3f
RLControlContext<T>::readVelocityCommand() const {
  Vec3f command;

  if (this->_data->controlParameters->use_rc) {
    command[0] = static_cast<float>(
        this->_data->_desiredStateCommand->rcCommand->v_des[0]);
    command[1] = static_cast<float>(
        this->_data->_desiredStateCommand->rcCommand->v_des[1]);
    command[2] = static_cast<float>(
        this->_data->_desiredStateCommand->rcCommand->omega_des[2]);
  } else {
    command[0] =
        rapid_rl::kMaxCommandLinearX *
        this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog[1];
    command[1] =
        -rapid_rl::kMaxCommandLinearY *
        this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog[0];
    command[2] =
        -rapid_rl::kMaxCommandYaw *
        this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0];
  }

  if (!command.allFinite()) return command;
  if (_switch.zeroCommand()) return Vec3f::Zero();

  command[0] = std::max(-rapid_rl::kMaxCommandLinearX,
                        std::min(rapid_rl::kMaxCommandLinearX, command[0]));
  command[1] = std::max(-rapid_rl::kMaxCommandLinearY,
                        std::min(rapid_rl::kMaxCommandLinearY, command[1]));
  command[2] = std::max(-rapid_rl::kMaxCommandYaw,
                        std::min(rapid_rl::kMaxCommandYaw, command[2]));

  if (command.norm() < (_switch.active() == RLController::Flat ? .1f : .05f)) {
    command.setZero();
  }
  return command;
}

template <typename T>
typename RLControlContext<T>::Vec3f
RLControlContext<T>::readProjectedGravity() const {
  return this->_data->_stateEstimator->getResult().rBody *
         Vec3f(0.0f, 0.0f, -1.0f);
}

template <typename T>
bool RLControlContext<T>::usesFlatSafety() const {
  return _switch.active() == RLController::Flat ||
      (_switch.phase() == RLPolicySwitch::Phase::Blending && _blendFrom == RLController::Flat);
}

template <typename T>
bool RLControlContext<T>::isOrientationUnsafe() const {
  const auto& estimate = this->_data->_stateEstimator->getResult();
  const auto& rpy = estimate.rpy;
  if (usesFlatSafety() && (!std::isfinite(estimate.rBody(2, 2)) ||
                          estimate.rBody(2, 2) < std::cos(kMaxFlatTiltRad))) return true;
  return !rpy.allFinite() || std::fabs(rpy[0]) > kMaxRollPitchRad ||
         std::fabs(rpy[1]) > kMaxRollPitchRad;
}

template class RLControlContext<float>;
