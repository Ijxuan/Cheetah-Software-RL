#include <chrono>
#include <iostream>
#include <thread>

#include "FSM_States/RapidRLAsyncPolicyRunner.h"

namespace {

int64_t monotonicTimeUs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

int fail(const std::string& message) {
  std::cerr << "[LeggedGymRLAsyncTest] FAIL: " << message << std::endl;
  return 1;
}

}  // namespace

int main() {
  rapid_rl::AsyncLibtorchPolicyRunner runner;
  if (!runner.start()) {
    return fail("worker did not start: " + runner.error());
  }

  const uint64_t epoch = runner.beginEpoch();
  rapid_rl::AsyncPolicyInput input;
  input.base_linear_velocity.setZero();
  input.base_angular_velocity.setZero();
  input.projected_gravity << 0.0f, 0.0f, -1.0f;
  input.command.setZero();
  input.q_policy = rapid_rl::DefaultJointPositionPolicyOrder();
  input.qd_policy.setZero();
  input.last_action.setZero();
  input.timestamp_us = monotonicTimeUs();
  if (!runner.submit(epoch, input)) {
    return fail("worker rejected a request after successful startup");
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(2);
  rapid_rl::AsyncPolicyResult result;
  while (std::chrono::steady_clock::now() < deadline) {
    if (runner.latestResult(&result) && result.epoch == epoch) {
      if (!result.success) {
        return fail("inference failed: " + result.error);
      }
      if (result.sequence == 0 || !result.action.allFinite() ||
          !result.target_q.allFinite()) {
        return fail("worker returned an invalid policy result");
      }
      std::cout << "[LeggedGymRLAsyncTest] PASS: sequence="
                << result.sequence << " latency=" << result.inference_time_ms
                << " ms" << std::endl;
      runner.stop();
      return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  runner.stop();
  return fail("timed out waiting for asynchronous inference");
}
