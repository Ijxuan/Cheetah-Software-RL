/*=================== legged_gym RL Joint PD Bridge ===================*/

#include "FSM_State_RLJointPD.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#ifdef linux
#include <sched.h>
#endif

#include <Configuration.h>
#include "RapidRLBuildConfig.h"
#include "Utilities/utilities.h"
#include "main_helper.h"

namespace {

// 策略以 rapid_rl::kPolicyDt 的周期运行；此处换算为单调时钟的微秒，
// 避免策略推理受系统时间校正影响。
constexpr int64_t kPolicyDtUs =
    static_cast<int64_t>(rapid_rl::kPolicyDt * 1000000.0f);
// 发布只读 RL 调试状态的周期；与策略周期保持一致。
constexpr int64_t kStatePublishDtUs =
    static_cast<int64_t>(rapid_rl::kStatePublishDt * 1000000.0f);
// 异步结果超过该年龄时不再写入关节目标，控制器保持上一帧目标。
constexpr int64_t kPolicyResultTimeoutUs = static_cast<int64_t>(
    rapid_rl::build_config::kPolicyResultTimeoutMs * 1000.0f);
// 连续的策略结果超时次数达到该值时，关节输出失能。
constexpr int kMaxConsecutivePolicyTimeouts =
    rapid_rl::build_config::kPolicyMaxConsecutiveTimeouts;
// 慢推理告警的限频周期，避免控制台被重复日志淹没。
constexpr int64_t kPolicyWarningPeriodUs = 500000;
// 正常运行时每秒打印一次端到端时序汇总；超时则立即打印诊断。
constexpr int64_t kPolicyTimingSummaryPeriodUs = 1000000;
// 相邻两次策略输出对应的关节目标最多变化 0.80 rad，只做限幅而不急停。
constexpr float kMaxTargetDeltaPerPolicyStep = 0.80f;
// 新目标与当前实测关节角相差超过 1.20 rad 时拒绝该目标并急停。
constexpr float kMaxTargetCurrentError = 1.20f;
// 策略关节顺序中 abad（外展）关节的允许目标角范围，单位 rad。
constexpr float kMinAbad = -1.5f;
constexpr float kMaxAbad = 1.5f;
// 策略关节顺序中 hip/thigh（髋/大腿）关节的允许目标角范围，单位 rad。
constexpr float kMinHip = -5.0f;
constexpr float kMaxHip = 5.0f;
// 策略关节顺序中 knee/calf（膝/小腿）关节的允许目标角范围，单位 rad。
constexpr float kMinKnee = -3.0f;
constexpr float kMaxKnee = 3.0f;
// 机身 roll 或 pitch 超过 30 度即急停；yaw 不参与该安全判断。
constexpr float kMaxRollPitchRad = 30.0f * static_cast<float>(M_PI) / 180.0f;

// 返回不受系统时间调整影响的单调微秒时间，供策略节拍和超时判断共用。
int64_t monotonicTimeUs() {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<int64_t>(now.tv_sec) * 1000000LL +
         static_cast<int64_t>(now.tv_nsec) / 1000LL;
}

int currentCpu() {
#ifdef linux
  return sched_getcpu();
#else
  return -1;
#endif
}

float microsecondsToMilliseconds(int64_t microseconds) {
  return microseconds > 0 ? static_cast<float>(microseconds) / 1000.0f
                         : 0.0f;
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
FSM_State_RLJointPD<T>::FSM_State_RLJointPD(
    ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::RL_JOINT_PD,
                   "RL_JOINT_PD"),
      _stateLCM(getLcmUrl(255)) {
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;
  std::cout << "Setup legged_gym LibTorch Joint PD" << std::endl;
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

  if (rapid_rl::build_config::kDebugLcmState && !_stateLCM.good()) {
    std::cout << "[LeggedGymRL] WARNING: failed to initialize debug LCM"
              << std::endl;
  }

  _asyncPolicyRunner.reset(new rapid_rl::AsyncLibtorchPolicyRunner());
  _policyReady = _asyncPolicyRunner->start();
  if (!_policyReady) {
    std::cout << "[LeggedGymRL] ERROR: failed to load LibTorch policy: "
              << _asyncPolicyRunner->error() << std::endl;
    return;
  }
  std::cout << "[LeggedGymRL] Loaded LibTorch policy: "
            << _asyncPolicyRunner->policyPath() << std::endl;
  std::cout << "[LeggedGymRL] Shapes: input "
            << _asyncPolicyRunner->inputShape() << ", action "
            << _asyncPolicyRunner->actionShape() << std::endl;
  std::cout << "[LeggedGymRL] Torch CPU threads: "
            << rapid_rl::build_config::kTorchNumThreads << std::endl;
}

template <typename T>
FSM_State_RLJointPD<T>::~FSM_State_RLJointPD() {}

template <typename T>
void FSM_State_RLJointPD<T>::onEnter() {
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;
  std::cout << "Start legged_gym LibTorch Joint PD" << std::endl;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;

  this->nextStateName = this->stateName;
  this->transitionData.zero();

  _stateSequence = 0;
  _lastStatePublishTimeUs = 0;
  _lastPolicyRequestTimeUs = 0;
  _lastPolicyWarningTimeUs = 0;
  _lastValidPolicyResultTimeUs = monotonicTimeUs();
  _lastPolicyTimeoutCheckTimeUs = _lastValidPolicyResultTimeUs;
  _lastPolicyTimingSummaryTimeUs = _lastValidPolicyResultTimeUs;
  _lastPolicyResultSequence = 0;
  _lastReportedOverwrittenRequests = 0;
  _consecutivePolicyTimeouts = 0;
  _policyTimingWindow = PolicyTimingWindow{};
  _lastPolicyTimingSample = PolicyTimingSample{};
  _policyEpoch = _asyncPolicyRunner ? _asyncPolicyRunner->beginEpoch() : 0;
  _lastActionPolicy.setZero();
  _lastTargetQRobot = readRobotQ();
  if (!_lastTargetQRobot.allFinite() || _lastTargetQRobot.norm() < 1e-4f) {
    _lastTargetQRobot = _defaultQRobot;
  }

  for (int leg = 0; leg < 4; ++leg) {
    _preCommands[leg].zero();
  }

  _emergencyStop = false;
  if (!_policyReady) {
    std::cout << "[LeggedGymRL] LibTorch policy is not ready; refusing RL control"
              << std::endl;
    _emergencyStop = true;
  }
}

template <typename T>
void FSM_State_RLJointPD<T>::run() {
  const int64_t now_us = monotonicTimeUs();

  if (isOrientationUnsafe()) {
    if (!_emergencyStop) {
      std::cout << "[LeggedGymRL] Orientation is unsafe; disabling RL bridge"
                << std::endl;
    }
    _emergencyStop = true;
  }

  if (rapid_rl::build_config::kDebugLcmState &&
      now_us - _lastStatePublishTimeUs >= kStatePublishDtUs) {
    publishRobotState(now_us);
    _lastStatePublishTimeUs = now_us;
  }

  if (_emergencyStop) {
    this->_data->_legController->setEnabled(false);
    return;
  }

  // 轮询不会阻塞控制线程。仅按 50 Hz 策略周期提交新状态快照；已完成的结果会
  // 在下一次 2 ms 控制周期中被采用。
  runAsyncPolicy(now_us);
  if (_emergencyStop) {
    this->_data->_legController->setEnabled(false);
    return;
  }
  commandTarget(_lastTargetQRobot);
}

template <typename T>
FSM_StateName FSM_State_RLJointPD<T>::checkTransition() {
  this->nextStateName = this->stateName;

  switch (static_cast<int>(this->_data->controlParameters->control_mode)) {
    case K_RL_JOINT_PD:
      break;

    case K_JOINT_PD:
      this->nextStateName = FSM_StateName::JOINT_PD;
      break;

    case K_IMPEDANCE_CONTROL:
      this->nextStateName = FSM_StateName::IMPEDANCE_CONTROL;
      this->transitionDuration = 1.0;
      break;

    case K_STAND_UP:
      this->nextStateName = FSM_StateName::STAND_UP;
      this->transitionDuration = 0.0;
      break;

    case K_PASSIVE:
      this->nextStateName = FSM_StateName::PASSIVE;
      this->transitionDuration = 0.0;
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      this->transitionDuration = 0.0;
      break;

    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      this->transitionDuration = 0.0;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      this->transitionDuration = 0.0;
      break;

    default:
      std::cout << "[LeggedGymRL] Bad transition request from " << K_RL_JOINT_PD
                << " to " << this->_data->controlParameters->control_mode
                << std::endl;
  }

  return this->nextStateName;
}

template <typename T>
TransitionData<T> FSM_State_RLJointPD<T>::transition() {
  this->transitionData.done = true;
  return this->transitionData;
}

template <typename T>
void FSM_State_RLJointPD<T>::onExit() {
  if (_asyncPolicyRunner) {
    _asyncPolicyRunner->beginEpoch();
  }
  this->_data->_legController->setEnabled(false);
}

template <typename T>
void FSM_State_RLJointPD<T>::publishRobotState(int64_t now_us) {
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
  state.mode = static_cast<int32_t>(this->_data->controlParameters->control_mode);

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
void FSM_State_RLJointPD<T>::commandTarget(const Vec12f& target_q_robot) {
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
void FSM_State_RLJointPD<T>::recordPolicyTiming(
    const rapid_rl::AsyncPolicyResult& result, int64_t now_us) {
  const float queue_ms = microsecondsToMilliseconds(
      result.worker_started_timestamp_us - result.request_timestamp_us);
  const float delivery_ms = microsecondsToMilliseconds(
      now_us - result.completed_timestamp_us);
  const float total_ms = microsecondsToMilliseconds(
      now_us - result.request_timestamp_us);

  _lastPolicyTimingSample.valid = true;
  _lastPolicyTimingSample.sequence = result.sequence;
  _lastPolicyTimingSample.queue_ms = queue_ms;
  _lastPolicyTimingSample.inference_ms = result.inference_time_ms;
  _lastPolicyTimingSample.delivery_ms = delivery_ms;
  _lastPolicyTimingSample.total_ms = total_ms;
  _lastPolicyTimingSample.overwritten_request_count =
      result.overwritten_request_count;
  _lastPolicyTimingSample.worker_cpu = result.worker_cpu;
  _lastPolicyTimingSample.control_cpu = currentCpu();

  ++_policyTimingWindow.count;
  _policyTimingWindow.queue_sum_ms += queue_ms;
  _policyTimingWindow.inference_sum_ms += result.inference_time_ms;
  _policyTimingWindow.delivery_sum_ms += delivery_ms;
  _policyTimingWindow.total_sum_ms += total_ms;
  _policyTimingWindow.queue_max_ms =
      std::max(_policyTimingWindow.queue_max_ms, queue_ms);
  _policyTimingWindow.inference_max_ms = std::max(
      _policyTimingWindow.inference_max_ms, result.inference_time_ms);
  _policyTimingWindow.delivery_max_ms =
      std::max(_policyTimingWindow.delivery_max_ms, delivery_ms);
  _policyTimingWindow.total_max_ms =
      std::max(_policyTimingWindow.total_max_ms, total_ms);
}

template <typename T>
void FSM_State_RLJointPD<T>::publishPolicyTimingSummary(int64_t now_us) {
  if (!rapid_rl::build_config::kDebugPolicyTiming ||
      now_us - _lastPolicyTimingSummaryTimeUs < kPolicyTimingSummaryPeriodUs ||
      !_asyncPolicyRunner) {
    return;
  }

  rapid_rl::AsyncLibtorchPolicyRunner::Diagnostics diagnostics;
  if (!_asyncPolicyRunner->diagnostics(&diagnostics)) {
    return;
  }
  const uint64_t overwritten_delta =
      diagnostics.overwritten_request_count >= _lastReportedOverwrittenRequests
          ? diagnostics.overwritten_request_count -
                _lastReportedOverwrittenRequests
          : 0;
  std::cout << "[LeggedGymRL] Policy pipeline 1s: samples="
            << _policyTimingWindow.count
            << " submit_seq=" << diagnostics.latest_submitted_sequence
            << " done_seq=" << diagnostics.latest_completed_sequence
            << " inflight_seq=" << diagnostics.in_flight_sequence
            << " worker_busy=" << diagnostics.worker_busy
            << " overwrites=" << diagnostics.overwritten_request_count
            << " (+" << overwritten_delta << ")";
  if (_policyTimingWindow.count > 0) {
    const double count = static_cast<double>(_policyTimingWindow.count);
    std::cout << " queue(avg/max)=" << _policyTimingWindow.queue_sum_ms / count
              << "/" << _policyTimingWindow.queue_max_ms
              << " infer(avg/max)="
              << _policyTimingWindow.inference_sum_ms / count << "/"
              << _policyTimingWindow.inference_max_ms
              << " delivery(avg/max)="
              << _policyTimingWindow.delivery_sum_ms / count << "/"
              << _policyTimingWindow.delivery_max_ms
              << " total(avg/max)="
              << _policyTimingWindow.total_sum_ms / count << "/"
              << _policyTimingWindow.total_max_ms
              << " last_seq=" << _lastPolicyTimingSample.sequence
              << " worker_cpu=" << _lastPolicyTimingSample.worker_cpu
              << " control_cpu=" << _lastPolicyTimingSample.control_cpu;
  } else {
    std::cout << " worker_cpu=" << diagnostics.worker_cpu
              << " control_cpu=" << currentCpu();
  }
  std::cout << std::endl;

  _lastPolicyTimingSummaryTimeUs = now_us;
  _lastReportedOverwrittenRequests = diagnostics.overwritten_request_count;
  _policyTimingWindow = PolicyTimingWindow{};
}

template <typename T>
void FSM_State_RLJointPD<T>::publishPolicyTimeoutDiagnostics(int64_t now_us) {
  if (!rapid_rl::build_config::kDebugPolicyTiming || !_asyncPolicyRunner) {
    return;
  }

  rapid_rl::AsyncLibtorchPolicyRunner::Diagnostics diagnostics;
  if (!_asyncPolicyRunner->diagnostics(&diagnostics)) {
    return;
  }
  std::cout << "[LeggedGymRL] Policy timeout diagnostics: submit_seq="
            << diagnostics.latest_submitted_sequence
            << " done_seq=" << diagnostics.latest_completed_sequence
            << " inflight_seq=" << diagnostics.in_flight_sequence
            << " worker_busy=" << diagnostics.worker_busy
            << " overwrites=" << diagnostics.overwritten_request_count
            << " worker_cpu=" << diagnostics.worker_cpu
            << " control_cpu=" << currentCpu();
  if (_lastPolicyTimingSample.valid) {
    std::cout << " last_seq=" << _lastPolicyTimingSample.sequence
              << " queue/infer/delivery/total="
              << _lastPolicyTimingSample.queue_ms << "/"
              << _lastPolicyTimingSample.inference_ms << "/"
              << _lastPolicyTimingSample.delivery_ms << "/"
              << _lastPolicyTimingSample.total_ms << " ms"
              << " last_worker_cpu=" << _lastPolicyTimingSample.worker_cpu
              << " last_control_cpu=" << _lastPolicyTimingSample.control_cpu;
  }
  std::cout << " result_age="
            << microsecondsToMilliseconds(
                   now_us - _lastValidPolicyResultTimeUs)
            << " ms" << std::endl;
}

template <typename T>
bool FSM_State_RLJointPD<T>::runAsyncPolicy(int64_t now_us) {
  if (!_asyncPolicyRunner || !_asyncPolicyRunner->ready() ||
      _policyEpoch == 0) {
    std::cout << "[LeggedGymRL] LibTorch policy is not ready; disabling RL"
              << std::endl;
    _emergencyStop = true;
    return false;
  }

  if (now_us - _lastPolicyRequestTimeUs >= kPolicyDtUs) {
    rapid_rl::AsyncPolicyInput input;
    input.base_linear_velocity = readBaseLinearVelocity();
    input.base_angular_velocity = readBaseAngularVelocity();
    input.projected_gravity = readProjectedGravity();
    input.command = readVelocityCommand();
    input.q_policy = rapid_rl::RobotToPolicyOrder(readRobotQ());
    input.qd_policy = rapid_rl::RobotToPolicyOrder(readRobotQd());
    input.last_action = _lastActionPolicy;
    input.timestamp_us = now_us;
    if (!_asyncPolicyRunner->submit(_policyEpoch, input)) {
      std::cout << "[LeggedGymRL] Failed to submit LibTorch policy request; "
                   "disabling RL"
                << std::endl;
      _emergencyStop = true;
      return false;
    }
    _lastPolicyRequestTimeUs = now_us;
  }

  rapid_rl::AsyncPolicyResult result;
  if (_asyncPolicyRunner->latestResult(&result) &&
      result.epoch == _policyEpoch &&
      result.sequence > _lastPolicyResultSequence) {
    _lastPolicyResultSequence = result.sequence;
    if (!result.success) {
      std::cout << "[LeggedGymRL] LibTorch inference failed: " << result.error
                << std::endl;
      _emergencyStop = true;
      return false;
    }

    recordPolicyTiming(result, now_us);

    const int64_t result_age_us = now_us - result.request_timestamp_us;
    if (result_age_us > kPolicyResultTimeoutUs) {
      std::cout << "[LeggedGymRL] Discarding stale LibTorch result (age "
                << result_age_us / 1000.0f << " ms); holding previous target"
                << std::endl;
    } else {
      if (result.inference_time_ms > rapid_rl::build_config::kMaxInferenceMs &&
          now_us - _lastPolicyWarningTimeUs > kPolicyWarningPeriodUs) {
        std::cout << "[LeggedGymRL] WARNING: LibTorch inference took "
                  << result.inference_time_ms << " ms" << std::endl;
        _lastPolicyWarningTimeUs = now_us;
      }
      if (!acceptPolicyOutput(result.action, result.target_q, true)) {
        return false;
      }
      _lastValidPolicyResultTimeUs = now_us;
      _lastPolicyTimeoutCheckTimeUs = now_us;
      _consecutivePolicyTimeouts = 0;
    }
  }

  publishPolicyTimingSummary(now_us);

  if (now_us - _lastValidPolicyResultTimeUs >= kPolicyResultTimeoutUs &&
      now_us - _lastPolicyTimeoutCheckTimeUs >= kPolicyDtUs) {
    _lastPolicyTimeoutCheckTimeUs = now_us;
    ++_consecutivePolicyTimeouts;
    std::cout << "[LeggedGymRL] Policy result timeout "
              << _consecutivePolicyTimeouts << "/"
              << kMaxConsecutivePolicyTimeouts
              << "; holding previous target" << std::endl;
    publishPolicyTimeoutDiagnostics(now_us);
    if (_consecutivePolicyTimeouts >= kMaxConsecutivePolicyTimeouts) {
      std::cout << "[LeggedGymRL] Policy timed out consecutively; "
                   "entering emergency stop"
                << std::endl;
      this->_data->emergencyStopRequested = true;
      _emergencyStop = true;
      return false;
    }
  }

  return true;
}

template <typename T>
bool FSM_State_RLJointPD<T>::acceptPolicyOutput(
    const Vec12f& action_policy, const Vec12f& target_policy,
    bool stop_on_reject) {
  auto reject = [&](const std::string& reason) {
    std::cout << "[LeggedGymRL] Rejecting policy output: " << reason
              << std::endl;
    if (stop_on_reject) {
      _emergencyStop = true;
    }
    return false;
  };

  if (!action_policy.allFinite() || !target_policy.allFinite()) {
    return reject("non-finite action or target");
  }

  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    if (target_policy[i] < jointLowerLimit(i) ||
        target_policy[i] > jointUpperLimit(i)) {
      return reject("target outside joint limits at index " +
                    std::to_string(i) + ": " +
                    std::to_string(target_policy[i]));
    }
  }

  const Vec12f target_robot = rapid_rl::PolicyToRobotOrder(target_policy);
  const Vec12f current_robot = readRobotQ();
  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    if (std::fabs(target_robot[i] - current_robot[i]) >
        kMaxTargetCurrentError) {
      return reject("target too far from current joint " + std::to_string(i));
    }
  }

  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    const float delta = target_robot[i] - _lastTargetQRobot[i];
    const float limited_delta =
        std::max(-kMaxTargetDeltaPerPolicyStep,
                 std::min(kMaxTargetDeltaPerPolicyStep, delta));
    _lastTargetQRobot[i] += limited_delta;
  }

  _lastActionPolicy = action_policy;
  return true;
}

template <typename T>
typename FSM_State_RLJointPD<T>::Vec12f FSM_State_RLJointPD<T>::readRobotQ()
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
typename FSM_State_RLJointPD<T>::Vec12f FSM_State_RLJointPD<T>::readRobotQd()
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
typename FSM_State_RLJointPD<T>::Vec3f
FSM_State_RLJointPD<T>::readBaseLinearVelocity() const {
  return this->_data->_stateEstimator->getResult().vBody;
}

template <typename T>
typename FSM_State_RLJointPD<T>::Vec3f
FSM_State_RLJointPD<T>::readBaseAngularVelocity() const {
  return this->_data->_stateEstimator->getResult().omegaBody;
}

template <typename T>
typename FSM_State_RLJointPD<T>::Vec3f
FSM_State_RLJointPD<T>::readVelocityCommand() const {
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
        1.0f * this->_data->_desiredStateCommand->gamepadCommand
                   ->leftStickAnalog[1];
    command[1] =
        -1.0f * this->_data->_desiredStateCommand->gamepadCommand
                    ->leftStickAnalog[0];
    command[2] =
        -1.0f * this->_data->_desiredStateCommand->gamepadCommand
                    ->rightStickAnalog[0];
  }

  command[0] = std::max(-1.0f, std::min(1.0f, command[0]));
  command[1] = std::max(-1.0f, std::min(1.0f, command[1]));
  command[2] = std::max(-1.0f, std::min(1.0f, command[2]));

  if (command.norm() < 0.05f) {
    command.setZero();
  }
  return command;
}

template <typename T>
typename FSM_State_RLJointPD<T>::Vec3f
FSM_State_RLJointPD<T>::readProjectedGravity() const {
  return this->_data->_stateEstimator->getResult().rBody *
         Vec3f(0.0f, 0.0f, -1.0f);
}

template <typename T>
bool FSM_State_RLJointPD<T>::isOrientationUnsafe() const {
  const auto& rpy = this->_data->_stateEstimator->getResult().rpy;
  return std::fabs(rpy[0]) > kMaxRollPitchRad ||
         std::fabs(rpy[1]) > kMaxRollPitchRad;
}

template class FSM_State_RLJointPD<float>;
