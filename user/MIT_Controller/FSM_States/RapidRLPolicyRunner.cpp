#include "RapidRLPolicyRunner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <sys/stat.h>

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

}  // namespace

bool OnnxPolicyRunner::load() {
  ready_ = false;
  error_.clear();
  checkpoint_dir_ = build_config::kCheckpointDir;
  policy_path_.clear();
  input_shape_ = "[]";
  action_shape_ = "[]";
  resetTiming();
  resetObservationHistory();

  if (checkpoint_dir_.empty()) {
    error_ = "RL checkpoint dir is empty";
    return false;
  }

  policy_path_ = joinPath(checkpoint_dir_, build_config::kCheckpointFile);
  if (!fileExists(policy_path_)) {
    error_ = "missing " + std::string(build_config::kCheckpointFile) + " under " +
             checkpoint_dir_;
    return false;
  }

  try {
    // Use only the default CPU provider, with sequential execution.
    session_.reset();
    policy_input_tensor_ = Ort::Value{nullptr};
    env_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "HIMLoco"));
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(build_config::kOnnxNumThreads);
    options.SetInterOpNumThreads(1);
    options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_.reset(new Ort::Session(*env_, policy_path_.c_str(), options));
    if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1) {
      error_ = "expected exactly one ONNX input and one output";
      return false;
    }
    const auto input_type = session_->GetInputTypeInfo(0);
    const auto output_type = session_->GetOutputTypeInfo(0);
    const auto input_info = input_type.GetTensorTypeAndShapeInfo();
    const auto output_info = output_type.GetTensorTypeAndShapeInfo();
    input_shape_ = tensorShapeString(input_info.GetShape());
    action_shape_ = tensorShapeString(output_info.GetShape());
    if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        input_info.GetShape() != std::vector<int64_t>{1, kObsDim} ||
        output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        output_info.GetShape() != std::vector<int64_t>{1, kActionDim}) {
      error_ = "expected ONNX float32 input [1, 270] and output [1, 12], got " +
               input_shape_ + " -> " + action_shape_;
      return false;
    }
    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
    const int64_t shape[] = {1, kObsDim};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    policy_input_tensor_ = Ort::Value::CreateTensor<float>(
        memory, observation_history_.data(), kObsDim, shape, 2);
    Vec12f action;
    for (int i = 0; i < std::max(1, build_config::kWarmupIters); ++i) {
      if (!runPolicy(&action, &error_)) return false;
    }
  } catch (const std::exception& e) {
    error_ = e.what();
    return false;
  }

  ready_ = true;
  return true;
}

void OnnxPolicyRunner::resetObservationHistory() {
  observation_history_.setZero();
}

void OnnxPolicyRunner::resetTiming() {
  timing_count_ = 0;
  timing_sum_ms_ = 0.0;
  timing_min_ms_ = 0.0f;
  timing_max_ms_ = 0.0f;
  timing_samples_.clear();
  timing_samples_.reserve(kTimingSampleCapacity);
}

void OnnxPolicyRunner::recordTiming(float inference_time_ms) {
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

bool OnnxPolicyRunner::timingSummary(
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

bool OnnxPolicyRunner::infer(const Vec3f& base_angular_velocity,
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
      *error = "null output pointer passed to ONNX Runtime policy";
    }
    return false;
  }
  if (!ready_) {
    if (error) {
      *error = error_.empty() ? "ONNX Runtime policy is not ready" : error_;
    }
    return false;
  }
  if (!base_angular_velocity.allFinite() ||
      !projected_gravity.allFinite() || !command.allFinite() ||
      !q_policy.allFinite() || !qd_policy.allFinite() ||
      !last_action.allFinite()) {
    if (error) {
      *error = "policy input contains non-finite values";
    }
    return false;
  }

  try {
    const auto start = std::chrono::steady_clock::now();
    const Eigen::Matrix<float, kOneStepObsDim, 1> current_obs =
        BuildOneStepObservationPolicyOrder(
            base_angular_velocity, projected_gravity, command, q_policy,
            qd_policy, last_action);
    if (!current_obs.allFinite()) {
      if (error) {
        *error = "policy observation contains non-finite values";
      }
      return false;
    }

    // Training prepends the newest 45-D frame and shifts older frames right:
    // [t, t-1, t-2, t-3, t-4, t-5].
    observation_history_.template tail<kObsDim - kOneStepObsDim>() =
        observation_history_
            .template head<kObsDim - kOneStepObsDim>()
            .eval();
    observation_history_.template head<kOneStepObsDim>() = current_obs;

    if (!runPolicy(action, error)) return false;
    *action = ClipPolicyAction(*action);
    *target_q = ActionToTargetQPolicyOrder(*action);

    const auto end = std::chrono::steady_clock::now();
    const float elapsed_ms =
        std::chrono::duration<float, std::milli>(end - start).count();
    if (inference_time_ms) {
      *inference_time_ms = elapsed_ms;
    }
    recordTiming(elapsed_ms);
  } catch (const std::exception& e) {
    if (error) {
      *error = e.what();
    }
    return false;
  }
  return true;
}

bool OnnxPolicyRunner::runPolicy(Vec12f* action, std::string* error) {
  const char* inputs[] = {input_name_.c_str()};
  const char* outputs[] = {output_name_.c_str()};
  auto values = session_->Run(Ort::RunOptions{nullptr}, inputs,
                              &policy_input_tensor_, 1, outputs, 1);
  const auto info = values[0].GetTensorTypeAndShapeInfo();
  if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      info.GetShape() != std::vector<int64_t>{1, kActionDim}) {
    if (error) *error = "expected ONNX float32 action [1, 12]";
    return false;
  }
  const float* data = values[0].GetTensorData<float>();
  for (int i = 0; i < kActionDim; ++i) (*action)[i] = data[i];
  if (!action->allFinite()) {
    if (error) *error = "ONNX policy produced non-finite actions";
    return false;
  }
  return true;
}

}  // namespace rapid_rl
