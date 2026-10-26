#pragma once

#include "FSM_State.h"
#include "RLControlContext.h"

// Common lifecycle/protection plumbing. Policy identity is immutable per state.
// ControlFSM, not the shared context, chooses and enters the destination state.
template <typename T>
class FSM_State_RLPolicy : public FSM_State<T> {
 public:
  FSM_State_RLPolicy(ControlFSMData<T>* data,
                    std::shared_ptr<RLControlContext<T>> context,
                    RLController controller, FSM_StateName name, const char* label)
      : FSM_State<T>(data, name, label), context_(std::move(context)), controller_(controller) {}

  void onEnter() override {
    this->nextStateName = this->stateName;
    this->transitionData.zero();
    context_->enter(controller_);
    std::cout << "[FSM] Enter " << this->stateString << std::endl;
  }
  FSM_StateName checkTransition() override {
    this->nextStateName = this->stateName;
    const int mode = static_cast<int>(this->_data->controlParameters->control_mode);
    if (context_->failed()) return this->nextStateName = FSM_StateName::PASSIVE;
    if (mode == K_RL_FLAT || mode == K_RL_STAIRS) {
      const auto requested = mode == K_RL_FLAT ? RLController::Flat : RLController::Stairs;
      const uint64_t sequence = this->_data->controlParameters->use_rc ?
          this->_data->_desiredStateCommand->rcCommand->rl_selection_sequence : 0;
      context_->request(requested, sequence);
      if (requested != controller_ && context_->transferReady(requested))
        this->nextStateName = requested == RLController::Flat ?
            FSM_StateName::RL_FLAT : FSM_StateName::RL_STAIRS;
      return this->nextStateName;
    }
    switch (mode) {
      case K_PASSIVE: this->nextStateName = FSM_StateName::PASSIVE; break;
      case K_RECOVERY_STAND: this->nextStateName = FSM_StateName::RECOVERY_STAND; break;
      case K_STAND_UP: this->nextStateName = FSM_StateName::STAND_UP; break;
      case K_BALANCE_STAND: this->nextStateName = FSM_StateName::BALANCE_STAND; break;
      case K_LOCOMOTION: this->nextStateName = FSM_StateName::LOCOMOTION; break;
      case K_JOINT_PD: this->nextStateName = FSM_StateName::JOINT_PD; break;
      case K_IMPEDANCE_CONTROL: this->nextStateName = FSM_StateName::IMPEDANCE_CONTROL; break;
      default: break;
    }
    return this->nextStateName;
  }
  TransitionData<T> transition() override {
    // Stop/preparation was performed in the old state's run(). Transfer itself
    // is atomic within a ControlFSM iteration, retaining the last applied PD.
    if (isRLState(this->nextStateName)) context_->holdTarget();
    else context_->stop();
    this->transitionData.done = true;
    return this->transitionData;
  }
  void onExit() override {
    context_->leave(isRLState(this->nextStateName));
    std::cout << "[FSM] Exit " << this->stateString << std::endl;
  }

 protected:
  std::shared_ptr<RLControlContext<T>> context_;
  const RLController controller_;
};

template <typename T>
class FSM_State_RLFlat final : public FSM_State_RLPolicy<T> {
 public:
  FSM_State_RLFlat(ControlFSMData<T>* data, std::shared_ptr<RLControlContext<T>> context)
      : FSM_State_RLPolicy<T>(data, std::move(context), RLController::Flat,
                             FSM_StateName::RL_FLAT, "RL_FLAT") {}
  void run() override { this->context_->runFlat(); }
};

template <typename T>
class FSM_State_RLStairs final : public FSM_State_RLPolicy<T> {
 public:
  FSM_State_RLStairs(ControlFSMData<T>* data, std::shared_ptr<RLControlContext<T>> context)
      : FSM_State_RLPolicy<T>(data, std::move(context), RLController::Stairs,
                             FSM_StateName::RL_STAIRS, "RL_STAIRS") {}
  void run() override { this->context_->runStairs(); }
};
