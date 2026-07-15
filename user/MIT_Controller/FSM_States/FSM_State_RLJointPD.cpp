/*=================== legged_gym RL Joint PD Bridge ===================*/

#include "FSM_State_RLJointPD.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <Configuration.h>
#include "RapidRLBuildConfig.h"
#include "Utilities/utilities.h"
#include "main_helper.h"

namespace {

constexpr int64_t kPolicyDtUs =
    static_cast<int64_t>(rapid_rl::kPolicyDt * 1000000.0f);
constexpr int64_t kStatePublishDtUs =
    static_cast<int64_t>(rapid_rl::kStatePublishDt * 1000000.0f);
constexpr int64_t kPolicyTimeoutUs = 100000;
constexpr int64_t kPolicyWarningPeriodUs = 500000;
constexpr float kMaxTargetDeltaPerPolicyStep = 0.80f;
constexpr float kMaxTargetCurrentError = 1.20f;
constexpr float kMinAbad = -1.5f;
constexpr float kMaxAbad = 1.5f;
constexpr float kMinHip = -5.0f;
constexpr float kMaxHip = 5.0f;
constexpr float kMinKnee = -3.0f;
constexpr float kMaxKnee = 3.0f;
constexpr float kMaxRollPitchRad = 30.0f * static_cast<float>(M_PI) / 180.0f;

int64_t monotonicTimeUs() {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<int64_t>(now.tv_sec) * 1000000LL +
         static_cast<int64_t>(now.tv_nsec) / 1000LL;
}

bool shouldHardStopForInference(float inference_time_ms) {
  return inference_time_ms * 1000.0f > static_cast<float>(kPolicyTimeoutUs);
}

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

  _libtorchRunner.reset(new rapid_rl::LibtorchPolicyRunner());
  _policyReady = _libtorchRunner->load();
  if (!_policyReady) {
    std::cout << "[LeggedGymRL] ERROR: failed to load LibTorch policy: "
              << _libtorchRunner->error() << std::endl;
    return;
  }
  std::cout << "[LeggedGymRL] Loaded LibTorch policy: "
            << _libtorchRunner->policyPath() << std::endl;
  std::cout << "[LeggedGymRL] Shapes: input "
            << _libtorchRunner->inputShape() << ", action "
            << _libtorchRunner->actionShape() << std::endl;
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
  _lastPolicyRunTimeUs = 0;
  _lastPolicyWarningTimeUs = 0;
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

  if (now_us - _lastPolicyRunTimeUs >= kPolicyDtUs) {
    runLibtorchPolicy(now_us);
    _lastPolicyRunTimeUs = now_us;
  }
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
  for (int joint = 0; joint < 3; ++joint) {
    const auto gains =
        rapid_rl::JointPdGainsForProfile(_pdGainProfile, joint);
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
bool FSM_State_RLJointPD<T>::runLibtorchPolicy(int64_t now_us) {
  if (!_libtorchRunner || !_libtorchRunner->ready()) {
    std::cout << "[LeggedGymRL] LibTorch policy is not ready; disabling RL"
              << std::endl;
    _emergencyStop = true;
    return false;
  }

  const Vec12f q_policy = rapid_rl::RobotToPolicyOrder(readRobotQ());
  const Vec12f qd_policy = rapid_rl::RobotToPolicyOrder(readRobotQd());
  const Vec3f base_linear_velocity = readBaseLinearVelocity();
  const Vec3f base_angular_velocity = readBaseAngularVelocity();
  const Vec3f command = readVelocityCommand();
  const Vec3f projected_gravity = readProjectedGravity();
  const float base_height = readBaseHeight();

  Vec12f action_policy;
  Vec12f target_policy;
  float inference_time_ms = 0.0f;
  std::string error;
  if (!_libtorchRunner->infer(
          base_linear_velocity, base_angular_velocity, projected_gravity,
          command, q_policy, qd_policy, _lastActionPolicy, base_height,
          &action_policy, &target_policy, &inference_time_ms, &error)) {
    std::cout << "[LeggedGymRL] LibTorch inference failed: " << error
              << std::endl;
    _emergencyStop = true;
    return false;
  }

  rapid_rl::InferenceTimingSummary timing;
  if (_libtorchRunner->timingSummary(&timing) &&
      (timing.count == 1 || timing.count == 50 || timing.count % 500 == 0)) {
    std::cout << "[LeggedGymRL] LibTorch timing count=" << timing.count
              << " min/mean/p95/max=" << timing.min_ms << "/"
              << timing.mean_ms << "/" << timing.p95_ms << "/"
              << timing.max_ms << " ms" << std::endl;
  }

  if (shouldHardStopForInference(inference_time_ms)) {
    std::cout << "[LeggedGymRL] LibTorch inference exceeded hard timeout: "
              << inference_time_ms << " ms" << std::endl;
    _emergencyStop = true;
    return false;
  }
  if (inference_time_ms > rapid_rl::build_config::kMaxInferenceMs &&
      now_us - _lastPolicyWarningTimeUs > kPolicyWarningPeriodUs) {
    std::cout << "[LeggedGymRL] WARNING: LibTorch inference took "
              << inference_time_ms << " ms" << std::endl;
    _lastPolicyWarningTimeUs = now_us;
  }

  return acceptPolicyOutput(action_policy, target_policy, true);
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
float FSM_State_RLJointPD<T>::readBaseHeight() const {
  return static_cast<float>(
      this->_data->_stateEstimator->getResult().position[2]);
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
        1.6f * this->_data->_desiredStateCommand->gamepadCommand
                   ->leftStickAnalog[1];
    command[1] =
        -0.6f * this->_data->_desiredStateCommand->gamepadCommand
                    ->leftStickAnalog[0];
    command[2] =
        -1.0f * this->_data->_desiredStateCommand->gamepadCommand
                    ->rightStickAnalog[0];
  }

  command[0] = std::max(-1.6f, std::min(1.6f, command[0]));
  command[1] = std::max(-0.6f, std::min(0.6f, command[1]));
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
