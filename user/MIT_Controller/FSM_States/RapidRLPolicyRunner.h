#ifndef RAPID_RL_POLICY_RUNNER_H
#define RAPID_RL_POLICY_RUNNER_H

#include <array>
#include <string>

#ifdef USE_LIBTORCH_RL
#include <torch/script.h>
#endif

#include "RapidRLPolicyConfig.h"

namespace rapid_rl {

enum class PolicyBackend {
  LCM,
  LIBTORCH,
};

struct PolicyRuntimeConfig {
  bool valid = false;
  PolicyBackend backend = PolicyBackend::LCM;
  std::string checkpoint_dir;
  bool debug_lcm = true;
  int warmup_iters = 20;
  float max_inference_ms = 20.0f;
  std::string error;
};

std::string PolicyBackendToString(PolicyBackend backend);
PolicyRuntimeConfig LoadPolicyRuntimeConfig(const std::string& config_path);

class LibtorchPolicyRunner {
 public:
  using Vec3f = Eigen::Matrix<float, 3, 1>;
  using Vec12f = Eigen::Matrix<float, kActionDim, 1>;

  bool load(const PolicyRuntimeConfig& config);
  void resetHistory();
  bool infer(const Vec3f& projected_gravity,
             const Vec3f& command,
             const Vec12f& q_policy,
             const Vec12f& qd_policy,
             const Vec12f& last_action,
             Vec12f* action,
             Vec12f* target_q,
             float* inference_time_ms,
             std::string* error);

  bool ready() const { return ready_; }
  const std::string& error() const { return error_; }

 private:
  std::array<float, kObsHistoryDim> obs_history_{};
  bool ready_ = false;
  std::string error_;

#ifdef USE_LIBTORCH_RL
  torch::jit::script::Module adaptation_;
  torch::jit::script::Module body_;
#endif
};

}  // namespace rapid_rl

#endif  // RAPID_RL_POLICY_RUNNER_H
