#pragma once

#include <cstdint>

// Policy identity shared by independent RL_FLAT / RL_STAIRS FSM states.
enum class RLController { Flat = 0, Stairs = 1 };

inline bool validRLController(RLController controller) {
  return controller == RLController::Flat || controller == RLController::Stairs;
}

inline const char* rlControllerName(RLController controller) {
  return controller == RLController::Flat ? "flat" : "stairs";
}
