#include "RapidRLPolicyRunner.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <sstream>
#include <sys/stat.h>

#include <ATen/Parallel.h>
#include "RapidRLBuildConfig.h"

namespace rapid_rl {
namespace {

constexpr size_t kTimingSampleCapacity = 1000;

std::string joinPath(const std::string& base, const std::string& leaf) {
  if (base.empty()) {
    return leaf;
  }
  if (base[base.size() - 1] == '/') {
    return base + leaf;
  }
  return base + "/" + leaf;
}

bool fileExists(const std::string& path) {
  struct stat buffer;
  return stat(path.c_str(), &buffer) == 0;
}

#ifdef USE_LIBTORCH_RL
bool g_interop_threads_set = false;

std::string tensorShapeString(const std::vector<int64_t>& sizes) {
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < sizes.size(); ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << sizes[i];
  }
  stream << "]";
  return stream.str();
}

bool hasExpectedShape(const torch::Tensor& tensor, int64_t cols) {
  return tensor.dim() == 2 && tensor.size(0) == 1 && tensor.size(1) == cols;
}
#endif

}  // namespace

bool LibtorchPolicyRunner::load() {
  ready_ = false;
  error_.clear();
  checkpoint_dir_ = build_config::kCheckpointDir;
  latent_shape_ = "[]";
  action_shape_ = "[]";
  resetHistory();
  resetTiming();

#ifndef USE_LIBTORCH_RL
  error_ = "mit_ctrl was built without USE_LIBTORCH_RL";
  return false;
#else
  if (checkpoint_dir_.empty()) {
    error_ = "rapid RL checkpoint dir is empty";
    return false;
  }

  const std::string adaptation_path =
      joinPath(checkpoint_dir_, "adaptation_module_latest.jit");
  const std::string body_path = joinPath(checkpoint_dir_, "body_latest.jit");
  if (!fileExists(adaptation_path) || !fileExists(body_path)) {
    error_ = "missing adaptation_module_latest.jit or body_latest.jit under " +
             checkpoint_dir_;
    return false;
  }

  try {
    at::set_num_threads(build_config::kTorchNumThreads);
    if (!g_interop_threads_set) {
      at::set_num_interop_threads(1);
      g_interop_threads_set = true;
    }

    adaptation_ = torch::jit::load(adaptation_path, torch::kCPU);
    body_ = torch::jit::load(body_path, torch::kCPU);
    adaptation_.eval();
    body_.eval();

    torch::InferenceMode inference_mode;
    const auto options =
        torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU);
    history_tensor_ =
        torch::from_blob(obs_history_.data(), {1, kObsHistoryDim}, options);
    actor_input_tensor_ = torch::zeros({1, kActorInputDim}, options);
    actor_latent_view_ =
        actor_input_tensor_.narrow(1, kObsDim, kLatentDim);

    torch::Tensor latent;
    torch::Tensor action;
    const int validation_iters = std::max(1, build_config::kWarmupIters);
    for (int i = 0; i < validation_iters; ++i) {
      latent = adaptation_.forward({history_tensor_}).toTensor();
      if (!hasExpectedShape(latent, kLatentDim)) {
        error_ = "expected latent shape [1, 18], got " +
                 tensorShapeString(latent.sizes().vec());
        return false;
      }
      actor_latent_view_.copy_(latent);
      action = body_.forward({actor_input_tensor_}).toTensor();
      if (!hasExpectedShape(action, kActionDim)) {
        error_ = "expected action shape [1, 12], got " +
                 tensorShapeString(action.sizes().vec());
        return false;
      }
    }

    latent_shape_ = tensorShapeString(latent.sizes().vec());
    action_shape_ = tensorShapeString(action.sizes().vec());
  } catch (const c10::Error& e) {
    error_ = e.what();
    return false;
  } catch (const std::exception& e) {
    error_ = e.what();
    return false;
  }

  ready_ = true;
  return true;
#endif
}

void LibtorchPolicyRunner::resetHistory() {
  obs_history_.fill(0.0f);
}

void LibtorchPolicyRunner::resetTiming() {
  timing_count_ = 0;
  timing_sum_ms_ = 0.0;
  timing_min_ms_ = 0.0f;
  timing_max_ms_ = 0.0f;
  timing_samples_.clear();
  timing_samples_.reserve(kTimingSampleCapacity);
}

void LibtorchPolicyRunner::recordTiming(float inference_time_ms) {
  if (timing_count_ == 0) {
    timing_min_ms_ = inference_time_ms;
    timing_max_ms_ = inference_time_ms;
  } else {
    timing_min_ms_ = std::min(timing_min_ms_, inference_time_ms);
    timing_max_ms_ = std::max(timing_max_ms_, inference_time_ms);
  }
  ++timing_count_;
  timing_sum_ms_ += inference_time_ms;

  if (timing_samples_.size() < kTimingSampleCapacity) {
    timing_samples_.push_back(inference_time_ms);
  }
}

bool LibtorchPolicyRunner::timingSummary(
    InferenceTimingSummary* summary) const {
  if (!summary || timing_count_ == 0) {
    return false;
  }

  summary->count = timing_count_;
  summary->min_ms = timing_min_ms_;
  summary->mean_ms = static_cast<float>(
      timing_sum_ms_ / static_cast<double>(timing_count_));
  summary->max_ms = timing_max_ms_;

  if (timing_samples_.empty()) {
    summary->p95_ms = timing_max_ms_;
    return true;
  }

  std::vector<float> sorted = timing_samples_;
  std::sort(sorted.begin(), sorted.end());
  size_t p95_index = (sorted.size() * 95 + 99) / 100;
  if (p95_index == 0) {
    p95_index = 1;
  }
  summary->p95_ms = sorted[p95_index - 1];
  return true;
}

bool LibtorchPolicyRunner::infer(const Vec3f& projected_gravity,
                                 const Vec3f& command,
                                 const Vec12f& q_policy,
                                 const Vec12f& qd_policy,
                                 const Vec12f& last_action,
                                 Vec12f* action,
                                 Vec12f* target_q,
                                 float* inference_time_ms,
                                 std::string* error) {
  if (error) {
    error->clear();
  }
  if (inference_time_ms) {
    *inference_time_ms = 0.0f;
  }
  if (!action || !target_q) {
    if (error) {
      *error = "null output pointer passed to LibTorch policy";
    }
    return false;
  }
  if (!ready_) {
    if (error) {
      *error = error_.empty() ? "LibTorch policy is not ready" : error_;
    }
    return false;
  }

#ifndef USE_LIBTORCH_RL
  (void)projected_gravity;
  (void)command;
  (void)q_policy;
  (void)qd_policy;
  (void)last_action;
  if (error) {
    *error = "mit_ctrl was built without USE_LIBTORCH_RL";
  }
  return false;
#else
  try {
    const auto start = std::chrono::steady_clock::now();
    const Eigen::Matrix<float, kObsDim, 1> obs = BuildObservationPolicyOrder(
        projected_gravity, command, q_policy, qd_policy, last_action);

    std::copy(obs_history_.begin() + kObsDim, obs_history_.end(),
              obs_history_.begin());
    std::copy(obs.data(), obs.data() + kObsDim,
              obs_history_.end() - kObsDim);

    torch::InferenceMode inference_mode;
    float* actor_input = actor_input_tensor_.data_ptr<float>();
    std::copy(obs.data(), obs.data() + kObsDim, actor_input);

    torch::Tensor latent = adaptation_.forward({history_tensor_}).toTensor();
    if (!hasExpectedShape(latent, kLatentDim)) {
      if (error) {
        *error = "expected latent shape [1, 18], got " +
                 tensorShapeString(latent.sizes().vec());
      }
      return false;
    }
    actor_latent_view_.copy_(latent);

    torch::Tensor action_t = body_.forward({actor_input_tensor_}).toTensor();
    if (!hasExpectedShape(action_t, kActionDim)) {
      if (error) {
        *error = "expected action shape [1, 12], got " +
                 tensorShapeString(action_t.sizes().vec());
      }
      return false;
    }

    action_t = action_t.to(torch::kCPU).to(torch::kFloat32).contiguous();
    const float* action_data = action_t.data_ptr<float>();
    for (int i = 0; i < kActionDim; ++i) {
      (*action)[i] = action_data[i];
    }
    *target_q = ActionToTargetQPolicyOrder(*action);

    const auto end = std::chrono::steady_clock::now();
    const float elapsed_ms =
        std::chrono::duration<float, std::milli>(end - start).count();
    if (inference_time_ms) {
      *inference_time_ms = elapsed_ms;
    }
    recordTiming(elapsed_ms);
  } catch (const c10::Error& e) {
    if (error) {
      *error = e.what();
    }
    return false;
  } catch (const std::exception& e) {
    if (error) {
      *error = e.what();
    }
    return false;
  }
  return true;
#endif
}

}  // namespace rapid_rl
