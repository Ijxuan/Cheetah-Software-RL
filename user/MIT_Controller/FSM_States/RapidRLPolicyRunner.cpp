#include "RapidRLPolicyRunner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <sys/stat.h>

#include <ATen/Parallel.h>
#include "RapidRLBuildConfig.h"

namespace rapid_rl {
namespace {

constexpr size_t kTimingSampleCapacity = 1000;
constexpr const char* kPolicyFileName = "legged_gym_policy_latest.jit";

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

bool isFloat32CpuTensor(const torch::Tensor& tensor) {
  return tensor.scalar_type() == torch::kFloat32 && tensor.device().is_cpu();
}

bool isFiniteTensor(const torch::Tensor& tensor) {
  return torch::isfinite(tensor).all().item<bool>();
}
#endif

}  // namespace

bool LibtorchPolicyRunner::load() {
  ready_ = false;
  error_.clear();
  checkpoint_dir_ = build_config::kCheckpointDir;
  policy_path_.clear();
  input_shape_ = "[]";
  action_shape_ = "[]";
  resetTiming();

#ifndef USE_LIBTORCH_RL
  error_ = "mit_ctrl was built without USE_LIBTORCH_RL";
  return false;
#else
  if (checkpoint_dir_.empty()) {
    error_ = "RL checkpoint dir is empty";
    return false;
  }

  policy_path_ = joinPath(checkpoint_dir_, kPolicyFileName);
  if (!fileExists(policy_path_)) {
    error_ = "missing " + std::string(kPolicyFileName) + " under " +
             checkpoint_dir_;
    return false;
  }

  try {
    at::set_num_threads(build_config::kTorchNumThreads);
    if (!g_interop_threads_set) {
      at::set_num_interop_threads(1);
      g_interop_threads_set = true;
    }

    policy_ = torch::jit::load(policy_path_, torch::kCPU);
    policy_.eval();

    torch::InferenceMode inference_mode;
    const auto options =
        torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU);
    policy_input_tensor_ = torch::zeros({1, kObsDim}, options);
    if (!hasExpectedShape(policy_input_tensor_, kObsDim) ||
        !isFloat32CpuTensor(policy_input_tensor_) ||
        !isFiniteTensor(policy_input_tensor_)) {
      error_ = "expected a finite CPU float32 policy input with shape [1, 48]";
      return false;
    }

    torch::Tensor action;
    const int validation_iters = std::max(1, build_config::kWarmupIters);
    for (int i = 0; i < validation_iters; ++i) {
      action = policy_.forward({policy_input_tensor_}).toTensor();
      if (!hasExpectedShape(action, kActionDim)) {
        error_ = "expected action shape [1, 12], got " +
                 tensorShapeString(action.sizes().vec());
        return false;
      }
      if (!isFloat32CpuTensor(action)) {
        error_ = "expected a CPU float32 action tensor";
        return false;
      }
      if (!isFiniteTensor(action)) {
        error_ = "policy warmup produced non-finite actions";
        return false;
      }
    }

    input_shape_ = tensorShapeString(policy_input_tensor_.sizes().vec());
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

bool LibtorchPolicyRunner::infer(const Vec3f& base_linear_velocity,
                                 const Vec3f& base_angular_velocity,
                                 const Vec3f& projected_gravity,
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
  if (!base_linear_velocity.allFinite() ||
      !base_angular_velocity.allFinite() ||
      !projected_gravity.allFinite() || !command.allFinite() ||
      !q_policy.allFinite() || !qd_policy.allFinite() ||
      !last_action.allFinite()) {
    if (error) {
      *error = "policy input contains non-finite values";
    }
    return false;
  }

#ifndef USE_LIBTORCH_RL
  (void)base_linear_velocity;
  (void)base_angular_velocity;
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
        base_linear_velocity, base_angular_velocity, projected_gravity,
        command, q_policy, qd_policy, last_action);
    if (!obs.allFinite()) {
      if (error) {
        *error = "policy observation contains non-finite values";
      }
      return false;
    }

    torch::InferenceMode inference_mode;
    float* policy_input = policy_input_tensor_.data_ptr<float>();
    std::copy(obs.data(), obs.data() + kObsDim, policy_input);

    torch::Tensor action_t = policy_.forward({policy_input_tensor_}).toTensor();
    if (!hasExpectedShape(action_t, kActionDim)) {
      if (error) {
        *error = "expected action shape [1, 12], got " +
                 tensorShapeString(action_t.sizes().vec());
      }
      return false;
    }
    if (!isFloat32CpuTensor(action_t)) {
      if (error) {
        *error = "expected a CPU float32 action tensor";
      }
      return false;
    }
    if (!isFiniteTensor(action_t)) {
      if (error) {
        *error = "policy produced non-finite actions";
      }
      return false;
    }

    action_t = action_t.contiguous();
    const float* action_data = action_t.data_ptr<float>();
    for (int i = 0; i < kActionDim; ++i) {
      (*action)[i] = action_data[i];
    }
    *action = ClipPolicyAction(*action);
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
