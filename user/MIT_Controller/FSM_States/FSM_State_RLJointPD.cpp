/*====================== Rapid RL Joint PD Bridge ======================*/

#include "FSM_State_RLJointPD.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

#include "Utilities/utilities.h"

namespace {

constexpr int64_t kPolicyDtUs =
    static_cast<int64_t>(rapid_rl::kPolicyDt * 1000000.0f);
constexpr int64_t kPolicyTimeoutUs = 100000;
constexpr int64_t kPolicyWarningPeriodUs = 500000;
constexpr int64_t kMaxStateLag = 8;
constexpr float kMaxTargetDeltaPerPolicyStep = 0.10f;
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

bool finiteArray(const float* values, int count) {
  for (int i = 0; i < count; ++i) {
    if (!std::isfinite(values[i])) {
      return false;
    }
  }
  return true;
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
      _stateLCM(getLcmUrl(255)),
      _policyLCM(getLcmUrl(255)),
      _lcmThreadRunning(false) {
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;
  std::cout << "Setup rapid-locomotion LCM Joint PD bridge" << std::endl;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;

  _lastActionPolicy.setZero();
  _defaultQRobot = rapid_rl::PolicyToRobotOrder(
      rapid_rl::DefaultJointPositionPolicyOrder());
  _lastTargetQRobot = _defaultQRobot;
  std::memset(&_latestPolicyCmd, 0, sizeof(_latestPolicyCmd));

  if (!_stateLCM.good() || !_policyLCM.good()) {
    std::cout << "[RapidRL] ERROR: failed to initialize LCM" << std::endl;
    return;
  }

  _policyLCM.subscribe("rl_policy_cmd",
                       &FSM_State_RLJointPD<T>::handlePolicyLCM, this);
  _lcmThreadRunning.store(true);
  _lcmThread = std::thread(&FSM_State_RLJointPD<T>::lcmThreadLoop, this);
}

template <typename T>
FSM_State_RLJointPD<T>::~FSM_State_RLJointPD() {
  _lcmThreadRunning.store(false);
  if (_lcmThread.joinable()) {
    _lcmThread.join();
  }
}

template <typename T>
void FSM_State_RLJointPD<T>::onEnter() {
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;
  std::cout << "Start rapid-locomotion LCM Joint PD bridge" << std::endl;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;

  this->nextStateName = this->stateName;
  this->transitionData.zero();

  _stateSequence = 0;
  _lastStatePublishTimeUs = 0;
  _lastPolicyWarningTimeUs = 0;
  _lastAcceptedPolicySequence = -1;
  _lastActionPolicy.setZero();
  _lastTargetQRobot = readRobotQ();
  if (!_lastTargetQRobot.allFinite() || _lastTargetQRobot.norm() < 1e-4f) {
    _lastTargetQRobot = _defaultQRobot;
  }

  {
    std::lock_guard<std::mutex> lock(_policyMutex);
    _hasPolicyCmd = false;
    std::memset(&_latestPolicyCmd, 0, sizeof(_latestPolicyCmd));
    _latestPolicyReceiveTimeUs = 0;
  }

  for (int leg = 0; leg < 4; ++leg) {
    _preCommands[leg].zero();
  }

  _emergencyStop = false;
}

template <typename T>
void FSM_State_RLJointPD<T>::run() {
  const int64_t now_us = monotonicTimeUs();

  if (isOrientationUnsafe()) {
    if (!_emergencyStop) {
      std::cout << "[RapidRL] Orientation is unsafe; disabling RL bridge"
                << std::endl;
    }
    _emergencyStop = true;
  }

  if (now_us - _lastStatePublishTimeUs >= kPolicyDtUs) {
    publishRobotState(now_us);
    _lastStatePublishTimeUs = now_us;
  }

  if (_emergencyStop) {
    this->_data->_legController->setEnabled(false);
    return;
  }

  acceptLatestPolicyCommand(now_us);
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
      std::cout << "[RapidRL] Bad transition request from " << K_RL_JOINT_PD
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
void FSM_State_RLJointPD<T>::lcmThreadLoop() {
  while (_lcmThreadRunning.load()) {
    _policyLCM.handleTimeout(50);
  }
}

template <typename T>
void FSM_State_RLJointPD<T>::handlePolicyLCM(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const rl_policy_cmd_lcmt* msg) {
  (void)rbuf;
  (void)chan;
  std::lock_guard<std::mutex> lock(_policyMutex);
  _latestPolicyCmd = *msg;
  _latestPolicyReceiveTimeUs = monotonicTimeUs();
  _hasPolicyCmd = true;
}

template <typename T>
void FSM_State_RLJointPD<T>::publishRobotState(int64_t now_us) {
  if (!_stateLCM.good()) {
    return;
  }

  const Vec12f q_policy = rapid_rl::RobotToPolicyOrder(readRobotQ());
  const Vec12f qd_policy = rapid_rl::RobotToPolicyOrder(readRobotQd());
  const Eigen::Matrix<float, 3, 1> command = readVelocityCommand();
  const Eigen::Matrix<float, 3, 1> projected_gravity =
      this->_data->_stateEstimator->getResult().rBody *
      Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, -1.0f);

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
  kp.diagonal().setConstant(rapid_rl::kKp);
  kd.diagonal().setConstant(rapid_rl::kKd);

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
bool FSM_State_RLJointPD<T>::acceptLatestPolicyCommand(int64_t now_us) {
  rl_policy_cmd_lcmt cmd;
  int64_t receive_time_us = 0;
  {
    std::lock_guard<std::mutex> lock(_policyMutex);
    if (!_hasPolicyCmd) {
      if (now_us - _lastPolicyWarningTimeUs > kPolicyWarningPeriodUs) {
        std::cout << "[RapidRL] Waiting for rl_policy_cmd" << std::endl;
        _lastPolicyWarningTimeUs = now_us;
      }
      return false;
    }
    cmd = _latestPolicyCmd;
    receive_time_us = _latestPolicyReceiveTimeUs;
  }

  if (cmd.status != 1) {
    return false;
  }
  if (cmd.sequence <= _lastAcceptedPolicySequence) {
    return false;
  }
  if (now_us - receive_time_us > kPolicyTimeoutUs) {
    if (now_us - _lastPolicyWarningTimeUs > kPolicyWarningPeriodUs) {
      std::cout << "[RapidRL] Policy command timeout; holding last target"
                << std::endl;
      _lastPolicyWarningTimeUs = now_us;
    }
    return false;
  }
  if (_stateSequence > kMaxStateLag &&
      cmd.state_sequence < _stateSequence - kMaxStateLag) {
    if (now_us - _lastPolicyWarningTimeUs > kPolicyWarningPeriodUs) {
      std::cout << "[RapidRL] Stale policy command for state seq "
                << cmd.state_sequence << ", latest " << _stateSequence
                << std::endl;
      _lastPolicyWarningTimeUs = now_us;
    }
    return false;
  }
  if (!finiteArray(cmd.action, rapid_rl::kActionDim) ||
      !finiteArray(cmd.target_q, rapid_rl::kActionDim)) {
    std::cout << "[RapidRL] Rejecting non-finite policy command" << std::endl;
    return false;
  }

  Vec12f action_policy;
  Vec12f target_policy;
  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    action_policy[i] = cmd.action[i];
    target_policy[i] = cmd.target_q[i];
    if (target_policy[i] < jointLowerLimit(i) ||
        target_policy[i] > jointUpperLimit(i)) {
      std::cout << "[RapidRL] Rejecting target outside joint limits at index "
                << i << ": " << target_policy[i] << std::endl;
      return false;
    }
  }

  Vec12f target_robot = rapid_rl::PolicyToRobotOrder(target_policy);
  const Vec12f current_robot = readRobotQ();
  for (int i = 0; i < rapid_rl::kActionDim; ++i) {
    if (std::fabs(target_robot[i] - current_robot[i]) >
        kMaxTargetCurrentError) {
      std::cout << "[RapidRL] Rejecting target too far from current joint "
                << i << std::endl;
      return false;
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
  _lastAcceptedPolicySequence = cmd.sequence;
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
Eigen::Matrix<float, 3, 1> FSM_State_RLJointPD<T>::readVelocityCommand()
    const {
  Eigen::Matrix<float, 3, 1> command;

  if (this->_data->controlParameters->use_rc) {
    command[0] = static_cast<float>(
        this->_data->_desiredStateCommand->rcCommand->v_des[0]);
    command[1] = static_cast<float>(
        this->_data->_desiredStateCommand->rcCommand->v_des[1]);
    command[2] = static_cast<float>(
        this->_data->_desiredStateCommand->rcCommand->omega_des[2]);
  } else {
    command[0] =
        0.6f * this->_data->_desiredStateCommand->gamepadCommand
                   ->leftStickAnalog[1];
    command[1] =
        -0.6f * this->_data->_desiredStateCommand->gamepadCommand
                    ->leftStickAnalog[0];
    command[2] =
        -1.0f * this->_data->_desiredStateCommand->gamepadCommand
                    ->rightStickAnalog[0];
  }

  command[0] = std::max(-0.6f, std::min(0.6f, command[0]));
  command[1] = std::max(-0.6f, std::min(0.6f, command[1]));
  command[2] = std::max(-1.0f, std::min(1.0f, command[2]));

  if (command.norm() < 0.05f) {
    command.setZero();
  }
  return command;
}

template <typename T>
bool FSM_State_RLJointPD<T>::isOrientationUnsafe() const {
  const auto& rpy = this->_data->_stateEstimator->getResult().rpy;
  return std::fabs(rpy[0]) > kMaxRollPitchRad ||
         std::fabs(rpy[1]) > kMaxRollPitchRad;
}

template class FSM_State_RLJointPD<float>;
