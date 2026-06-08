#include "RapidRLPolicyRunner.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <sstream>
#include <sys/stat.h>

#include <Configuration.h>
#include "ParamHandler/ParamHandler.hpp"

namespace rapid_rl {
namespace {

bool isAbsolutePath(const std::string& path) {
  return !path.empty() && path[0] == '/';
}

std::string joinPath(const std::string& base, const std::string& leaf) {
  if (base.empty()) {
    return leaf;
  }
  if (base[base.size() - 1] == '/') {
    return base + leaf;
  }
  return base + "/" + leaf;
}

std::string resolveRepoPath(const std::string& path) {
  if (isAbsolutePath(path)) {
    return path;
  }
  return joinPath(THIS_COM, path);
}

std::string lowerCopy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

#ifdef USE_LIBTORCH_RL
bool fileExists(const std::string& path) {
  struct stat buffer;
  return stat(path.c_str(), &buffer) == 0;
}

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
#endif

}  // namespace

std::string PolicyBackendToString(PolicyBackend backend) {
  switch (backend) {
    case PolicyBackend::LCM:
      return "lcm";
    case PolicyBackend::LIBTORCH:
      return "libtorch";
  }
  return "unknown";
}

PolicyRuntimeConfig LoadPolicyRuntimeConfig(const std::string& config_path) {
  PolicyRuntimeConfig config;
  ParamHandler handler(config_path);
  if (!handler.fileOpenedSuccessfully()) {
    config.error = "failed to open " + config_path;
    return config;
  }

  std::string backend;
  if (!handler.getString("backend", backend)) {
    config.error = "missing backend in " + config_path;
    return config;
  }

  backend = lowerCopy(backend);
  if (backend == "lcm") {
    config.backend = PolicyBackend::LCM;
  } else if (backend == "libtorch") {
    config.backend = PolicyBackend::LIBTORCH;
  } else {
    config.error = "unsupported backend '" + backend + "'";
    return config;
  }

  std::string checkpoint_dir;
  if (handler.getString("checkpoint_dir", checkpoint_dir)) {
    config.checkpoint_dir = resolveRepoPath(checkpoint_dir);
  }

  int debug_lcm = 1;
  if (handler.getValue("debug_lcm", debug_lcm)) {
    config.debug_lcm = debug_lcm != 0;
  }

  int warmup_iters = config.warmup_iters;
  if (handler.getValue("warmup_iters", warmup_iters)) {
    config.warmup_iters = std::max(0, warmup_iters);
  }

  double max_inference_ms = config.max_inference_ms;
  if (handler.getValue("max_inference_ms", max_inference_ms)) {
    config.max_inference_ms = static_cast<float>(std::max(0.0, max_inference_ms));
  }

  config.valid = true;
  return config;
}

bool LibtorchPolicyRunner::load(const PolicyRuntimeConfig& config) {
  ready_ = false;
  error_.clear();
  resetHistory();

#ifndef USE_LIBTORCH_RL
  (void)config;
  error_ = "mit_ctrl was built without USE_LIBTORCH_RL";
  return false;
#else
  if (config.checkpoint_dir.empty()) {
    error_ = "checkpoint_dir is empty";
    return false;
  }

  const std::string adaptation_path =
      joinPath(config.checkpoint_dir, "adaptation_module_latest.jit");
  const std::string body_path = joinPath(config.checkpoint_dir, "body_latest.jit");
  if (!fileExists(adaptation_path) || !fileExists(body_path)) {
    error_ = "missing adaptation_module_latest.jit or body_latest.jit under " +
             config.checkpoint_dir;
    return false;
  }

  try {
    adaptation_ = torch::jit::load(adaptation_path, torch::kCPU);
    body_ = torch::jit::load(body_path, torch::kCPU);
    adaptation_.eval();
    body_.eval();

    torch::NoGradGuard no_grad;
    const auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU);
    torch::Tensor hist = torch::zeros({1, kObsHistoryDim}, options);
    torch::Tensor obs = torch::zeros({1, kObsDim}, options);
    torch::Tensor latent;
    torch::Tensor action;
    for (int i = 0; i < std::max(1, config.warmup_iters); ++i) {
      latent = adaptation_.forward({hist}).toTensor();
      action = body_.forward({torch::cat({obs, latent}, 1)}).toTensor();
    }

    if (latent.dim() != 2 || latent.size(0) != 1 || latent.size(1) != kLatentDim) {
      error_ = "expected latent shape [1, 18], got " +
               tensorShapeString(latent.sizes().vec());
      return false;
    }
    if (action.dim() != 2 || action.size(0) != 1 || action.size(1) != kActionDim) {
      error_ = "expected action shape [1, 12], got " +
               tensorShapeString(action.sizes().vec());
      return false;
    }
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
  (void)action;
  (void)target_q;
  (void)inference_time_ms;
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

    torch::NoGradGuard no_grad;
    const auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU);
    torch::Tensor hist_t = torch::from_blob(
        obs_history_.data(), {1, kObsHistoryDim}, options);
    torch::Tensor obs_t = torch::from_blob(
        const_cast<float*>(obs.data()), {1, kObsDim}, options);
    torch::Tensor latent = adaptation_.forward({hist_t}).toTensor();
    if (latent.dim() != 2 || latent.size(0) != 1 || latent.size(1) != kLatentDim) {
      if (error) {
        *error = "expected latent shape [1, 18], got " +
                 tensorShapeString(latent.sizes().vec());
      }
      return false;
    }

    torch::Tensor action_t = body_.forward({torch::cat({obs_t, latent}, 1)}).toTensor();
    if (action_t.dim() != 2 || action_t.size(0) != 1 ||
        action_t.size(1) != kActionDim) {
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
    if (inference_time_ms) {
      *inference_time_ms =
          std::chrono::duration<float, std::milli>(end - start).count();
    }
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
