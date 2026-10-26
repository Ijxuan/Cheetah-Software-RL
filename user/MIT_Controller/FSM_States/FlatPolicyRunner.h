#pragma once

#include <cstring>
#include <sstream>
#include "cpuMLP.hpp"
#include "RLPolicyHistory.h"

namespace rapid_rl {
// Legacy local-deploy-nav-integration actor/estimator; no actuator or estimator
// side effects here. All temporal inputs come from real executed robot history.
class FlatPolicyRunner {
 public:
  bool load(const std::string& directory);
  bool infer(const FlatObservation& raw, JointVector* target, float* elapsed_ms,
             std::string* error);
  bool ready() const { return ready_; }
  const std::string& error() const { return error_; }
  static JointVector defaultQ() {
    JointVector q;
    q << 0.f, -.9f, 1.8f, 0.f, -.9f, 1.8f,
         0.f, -.9f, 1.8f, 0.f, -.9f, 1.8f;
    return q;
  }

 private:
  rai::FuncApprox::MLP_fullyconnected<float, 152, 12,
      rai::FuncApprox::ActivationType::leakyrelu> actor_{{512, 256, 64}};
  rai::FuncApprox::MLP_fullyconnected<float, 141, 11,
      rai::FuncApprox::ActivationType::leakyrelu> estimator_{{256, 128}};
  FlatObservation mean_ = FlatObservation::Zero();
  FlatObservation variance_ = FlatObservation::Zero();
  bool ready_ = false;
  std::string error_;
};
}  // namespace rapid_rl
