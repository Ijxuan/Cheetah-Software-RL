#ifndef RAPID_RL_POLICY_RUNNER_H
#define RAPID_RL_POLICY_RUNNER_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#ifdef USE_LIBTORCH_RL
#include <torch/script.h>
#endif

#include "RapidRLPolicyConfig.h"

namespace rapid_rl {

struct InferenceTimingSummary {
  uint64_t count = 0;
  float min_ms = 0.0f;
  float mean_ms = 0.0f;
  float p95_ms = 0.0f;
  float max_ms = 0.0f;
};

class LibtorchPolicyRunner {
 public:
  using Vec3f = Eigen::Matrix<float, 3, 1>;
  using Vec12f = Eigen::Matrix<float, kActionDim, 1>;

  bool load();
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
  const std::string& checkpointDir() const { return checkpoint_dir_; }
  const std::string& latentShape() const { return latent_shape_; }
  const std::string& actionShape() const { return action_shape_; }
  bool timingSummary(InferenceTimingSummary* summary) const;

 private:
  void resetTiming();
  void recordTiming(float inference_time_ms);

  std::array<float, kObsHistoryDim> obs_history_{};
  bool ready_ = false;
  std::string error_;
  std::string checkpoint_dir_;
  std::string latent_shape_ = "[]";
  std::string action_shape_ = "[]";
  uint64_t timing_count_ = 0;
  double timing_sum_ms_ = 0.0;
  float timing_min_ms_ = 0.0f;
  float timing_max_ms_ = 0.0f;
  std::vector<float> timing_samples_;

#ifdef USE_LIBTORCH_RL
  torch::jit::script::Module adaptation_;
  torch::jit::script::Module body_;
  torch::Tensor history_tensor_;
  torch::Tensor actor_input_tensor_;
  torch::Tensor actor_latent_view_;
#endif
};

}  // namespace rapid_rl

#endif  // RAPID_RL_POLICY_RUNNER_H
