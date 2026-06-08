#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <vector>

#include "FSM_States/RapidRLPolicyRunner.h"
#include "RapidRLBuildConfig.h"

namespace {

struct TimingStats {
  float min_ms = 0.0f;
  float mean_ms = 0.0f;
  float p50_ms = 0.0f;
  float p95_ms = 0.0f;
  float max_ms = 0.0f;
};

float percentile(const std::vector<float>& sorted, int percentile_value) {
  if (sorted.empty()) {
    return 0.0f;
  }
  size_t index = (sorted.size() * static_cast<size_t>(percentile_value) + 99) /
                 100;
  if (index == 0) {
    index = 1;
  }
  return sorted[index - 1];
}

TimingStats computeTimingStats(std::vector<float> samples) {
  TimingStats stats;
  if (samples.empty()) {
    return stats;
  }
  std::sort(samples.begin(), samples.end());
  stats.min_ms = samples.front();
  stats.max_ms = samples.back();
  stats.p50_ms = percentile(samples, 50);
  stats.p95_ms = percentile(samples, 95);
  const double sum =
      std::accumulate(samples.begin(), samples.end(), 0.0);
  stats.mean_ms = static_cast<float>(sum / samples.size());
  return stats;
}

}  // namespace

int main(int argc, char** argv) {
  int iterations = 500;
  if (argc > 1) {
    const int requested = std::atoi(argv[1]);
    if (requested > 0) {
      iterations = requested;
    }
  }

  rapid_rl::LibtorchPolicyRunner runner;
  if (!runner.load()) {
    std::cerr << "[RapidRLBenchmark] failed to load policy: "
              << runner.error() << std::endl;
    return 1;
  }

  std::cout << "[RapidRLBenchmark] checkpoint_dir="
            << runner.checkpointDir() << std::endl;
  std::cout << "[RapidRLBenchmark] latent_shape=" << runner.latentShape()
            << " action_shape=" << runner.actionShape() << std::endl;
  std::cout << "[RapidRLBenchmark] torch_threads="
            << rapid_rl::build_config::kTorchNumThreads
            << " iterations=" << iterations << std::endl;

  rapid_rl::LibtorchPolicyRunner::Vec3f projected_gravity;
  projected_gravity << 0.0f, 0.0f, -1.0f;
  rapid_rl::LibtorchPolicyRunner::Vec3f command;
  command.setZero();
  rapid_rl::LibtorchPolicyRunner::Vec12f q_policy =
      rapid_rl::DefaultJointPositionPolicyOrder();
  rapid_rl::LibtorchPolicyRunner::Vec12f qd_policy;
  qd_policy.setZero();
  rapid_rl::LibtorchPolicyRunner::Vec12f last_action;
  last_action.setZero();

  std::vector<float> timings;
  timings.reserve(static_cast<size_t>(iterations));
  for (int i = 0; i < iterations; ++i) {
    rapid_rl::LibtorchPolicyRunner::Vec12f action;
    rapid_rl::LibtorchPolicyRunner::Vec12f target_q;
    float inference_time_ms = 0.0f;
    std::string error;
    if (!runner.infer(projected_gravity, command, q_policy, qd_policy,
                      last_action, &action, &target_q, &inference_time_ms,
                      &error)) {
      std::cerr << "[RapidRLBenchmark] inference failed: " << error
                << std::endl;
      return 2;
    }
    if (!action.allFinite() || !target_q.allFinite()) {
      std::cerr << "[RapidRLBenchmark] non-finite policy output"
                << std::endl;
      return 3;
    }
    last_action = action;
    timings.push_back(inference_time_ms);
  }

  const TimingStats stats = computeTimingStats(timings);
  std::cout << "[RapidRLBenchmark] p50/p95/max=" << stats.p50_ms << "/"
            << stats.p95_ms << "/" << stats.max_ms << " ms"
            << " mean=" << stats.mean_ms << " min=" << stats.min_ms
            << std::endl;
  if (stats.p95_ms > rapid_rl::build_config::kMaxInferenceMs) {
    std::cout << "[RapidRLBenchmark] WARNING: p95 exceeds "
              << rapid_rl::build_config::kMaxInferenceMs << " ms target"
              << std::endl;
  }
  return 0;
}
