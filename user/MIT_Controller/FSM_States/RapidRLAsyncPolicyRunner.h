#ifndef RAPID_RL_ASYNC_POLICY_RUNNER_H
#define RAPID_RL_ASYNC_POLICY_RUNNER_H

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "RapidRLPolicyRunner.h"

namespace rapid_rl {

// 控制线程提交完整状态快照，绝不等待 LibTorch。待处理请求刻意只保留一份：
// 当慢推理正在执行时，下一次只需要推理最新的机器人状态。
struct AsyncPolicyInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LibtorchPolicyRunner::Vec3f base_linear_velocity;
  LibtorchPolicyRunner::Vec3f base_angular_velocity;
  LibtorchPolicyRunner::Vec3f projected_gravity;
  LibtorchPolicyRunner::Vec3f command;
  LibtorchPolicyRunner::Vec12f q_policy;
  LibtorchPolicyRunner::Vec12f qd_policy;
  LibtorchPolicyRunner::Vec12f last_action;
  int64_t timestamp_us = 0;
};

struct AsyncPolicyResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t epoch = 0;
  uint64_t sequence = 0;
  int64_t request_timestamp_us = 0;
  int64_t worker_started_timestamp_us = 0;
  int64_t completed_timestamp_us = 0;
  int worker_cpu = -1;
  uint64_t overwritten_request_count = 0;
  bool success = false;
  float inference_time_ms = 0.0f;
  LibtorchPolicyRunner::Vec12f action;
  LibtorchPolicyRunner::Vec12f target_q;
  std::string error;
};

class AsyncLibtorchPolicyRunner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AsyncLibtorchPolicyRunner() = default;
  ~AsyncLibtorchPolicyRunner();

  AsyncLibtorchPolicyRunner(const AsyncLibtorchPolicyRunner&) = delete;
  AsyncLibtorchPolicyRunner& operator=(const AsyncLibtorchPolicyRunner&) =
      delete;

  // 启动 SCHED_OTHER 工作线程，只在策略加载阶段等待。该函数在机器人控制周期
  // 任务启动前调用，绝不从 run() 调用。
  bool start();
  void stop();

  bool ready() const;
  std::string error() const;
  std::string policyPath() const;
  std::string inputShape() const;
  std::string actionShape() const;

  // 丢弃属于上一次进入 RL 状态的结果；返回的 epoch 必须传给 submit()。
  uint64_t beginEpoch();
  bool submit(uint64_t epoch, const AsyncPolicyInput& input);
  bool latestResult(AsyncPolicyResult* result) const;

  struct Diagnostics {
    uint64_t latest_submitted_sequence = 0;
    uint64_t latest_completed_sequence = 0;
    uint64_t in_flight_sequence = 0;
    uint64_t overwritten_request_count = 0;
    bool worker_busy = false;
    int worker_cpu = -1;
  };
  bool diagnostics(Diagnostics* diagnostics) const;

 private:
  struct Request {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint64_t epoch = 0;
    uint64_t sequence = 0;
    AsyncPolicyInput input;
  };

  void workerLoop();
  bool configureWorkerThread(std::string* error) const;

  mutable std::mutex mutex_;
  std::condition_variable work_cv_;
  std::condition_variable startup_cv_;
  std::thread worker_;
  bool started_ = false;
  bool stop_requested_ = false;
  bool startup_done_ = false;
  bool ready_ = false;
  std::string error_;
  std::string policy_path_;
  std::string input_shape_;
  std::string action_shape_;
  uint64_t active_epoch_ = 0;
  uint64_t next_sequence_ = 0;
  uint64_t latest_submitted_sequence_ = 0;
  uint64_t latest_completed_sequence_ = 0;
  uint64_t in_flight_sequence_ = 0;
  uint64_t overwritten_request_count_ = 0;
  bool worker_busy_ = false;
  int worker_cpu_ = -1;
  bool has_request_ = false;
  Request request_;
  bool has_result_ = false;
  AsyncPolicyResult latest_result_;
  LibtorchPolicyRunner runner_;
};

}  // namespace rapid_rl

#endif  // RAPID_RL_ASYNC_POLICY_RUNNER_H
