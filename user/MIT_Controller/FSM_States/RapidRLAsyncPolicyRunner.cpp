#include "RapidRLAsyncPolicyRunner.h"

#include <cstring>
#include <sstream>
#include <time.h>

#ifdef linux
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#endif

#include "RapidRLBuildConfig.h"

namespace rapid_rl {
namespace {

int64_t monotonicTimeUs() {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<int64_t>(now.tv_sec) * 1000000LL +
         static_cast<int64_t>(now.tv_nsec) / 1000LL;
}

int currentCpu() {
#ifdef linux
  return sched_getcpu();
#else
  return -1;
#endif
}

std::string pthreadError(const char* operation, int error_code) {
  std::ostringstream stream;
  stream << operation << " failed: " << std::strerror(error_code);
  return stream.str();
}

}  // namespace

AsyncLibtorchPolicyRunner::~AsyncLibtorchPolicyRunner() { stop(); }

bool AsyncLibtorchPolicyRunner::start() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (started_) {
    return ready_;
  }

  stop_requested_ = false;
  startup_done_ = false;
  ready_ = false;
  error_.clear();
  policy_path_.clear();
  input_shape_.clear();
  action_shape_.clear();
  started_ = true;
  worker_ = std::thread(&AsyncLibtorchPolicyRunner::workerLoop, this);
  startup_cv_.wait(lock, [this] { return startup_done_; });
  return ready_;
}

void AsyncLibtorchPolicyRunner::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_) {
      return;
    }
    stop_requested_ = true;
    work_cv_.notify_all();
  }
  if (worker_.joinable()) {
    worker_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  started_ = false;
  ready_ = false;
  has_request_ = false;
  has_result_ = false;
}

bool AsyncLibtorchPolicyRunner::ready() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ready_;
}

std::string AsyncLibtorchPolicyRunner::error() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return error_;
}

std::string AsyncLibtorchPolicyRunner::policyPath() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return policy_path_;
}

std::string AsyncLibtorchPolicyRunner::inputShape() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return input_shape_;
}

std::string AsyncLibtorchPolicyRunner::actionShape() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return action_shape_;
}

uint64_t AsyncLibtorchPolicyRunner::beginEpoch() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++active_epoch_;
  has_request_ = false;
  has_result_ = false;
  latest_submitted_sequence_ = 0;
  latest_completed_sequence_ = 0;
  overwritten_request_count_ = 0;
  return active_epoch_;
}

bool AsyncLibtorchPolicyRunner::submit(uint64_t epoch,
                                        const AsyncPolicyInput& input) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ready_ || stop_requested_ || epoch != active_epoch_) {
    return false;
  }
  if (has_request_) {
    ++overwritten_request_count_;
  }
  request_.epoch = epoch;
  request_.sequence = ++next_sequence_;
  request_.input = input;
  latest_submitted_sequence_ = request_.sequence;
  has_request_ = true;
  work_cv_.notify_one();
  return true;
}

bool AsyncLibtorchPolicyRunner::latestResult(AsyncPolicyResult* result) const {
  if (!result) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_result_) {
    return false;
  }
  *result = latest_result_;
  return true;
}

bool AsyncLibtorchPolicyRunner::diagnostics(
    Diagnostics* diagnostics) const {
  if (!diagnostics) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  diagnostics->latest_submitted_sequence = latest_submitted_sequence_;
  diagnostics->latest_completed_sequence = latest_completed_sequence_;
  diagnostics->in_flight_sequence = in_flight_sequence_;
  diagnostics->overwritten_request_count = overwritten_request_count_;
  diagnostics->worker_busy = worker_busy_;
  diagnostics->worker_cpu = worker_cpu_;
  return true;
}

bool AsyncLibtorchPolicyRunner::configureWorkerThread(std::string* error) const {
#ifdef linux
  sched_param params;
  params.sched_priority = 0;
  int result = pthread_setschedparam(pthread_self(), SCHED_OTHER, &params);
  if (result != 0) {
    if (error) {
      *error = pthreadError("无法将 LibTorch 工作线程设为 SCHED_OTHER", result);
    }
    return false;
  }

  const long cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
  const int ethercat_cpu = build_config::kEthercatCpuCore;
  if (cpu_count <= 1 || ethercat_cpu < 0 || ethercat_cpu >= cpu_count) {
    if (error) {
      *error = "无法为 EtherCAT 预留 CPU：CPU 拓扑或配置无效";
    }
    return false;
  }
  cpu_set_t worker_cpus;
  CPU_ZERO(&worker_cpus);
  for (long cpu = 0; cpu < cpu_count && cpu < CPU_SETSIZE; ++cpu) {
    if (cpu != ethercat_cpu) {
      CPU_SET(cpu, &worker_cpus);
    }
  }
  result = pthread_setaffinity_np(pthread_self(), sizeof(worker_cpus),
                                  &worker_cpus);
  if (result != 0) {
    if (error) {
      *error = pthreadError("无法将 EtherCAT CPU 从 LibTorch 工作线程亲和性中排除",
                            result);
    }
    return false;
  }
#else
  (void)error;
#endif
  return true;
}

void AsyncLibtorchPolicyRunner::workerLoop() {
  std::string startup_error;
  const bool worker_ready = configureWorkerThread(&startup_error) && runner_.load();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ready_ = worker_ready;
    if (worker_ready) {
      policy_path_ = runner_.policyPath();
      input_shape_ = runner_.inputShape();
      action_shape_ = runner_.actionShape();
    } else {
      error_ = startup_error.empty() ? runner_.error() : startup_error;
    }
    startup_done_ = true;
    startup_cv_.notify_all();
  }
  if (!worker_ready) {
    return;
  }

  while (true) {
    Request request;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      work_cv_.wait(lock, [this] { return stop_requested_ || has_request_; });
      if (stop_requested_) {
        return;
      }
      request = request_;
      has_request_ = false;
      worker_busy_ = true;
      in_flight_sequence_ = request.sequence;
    }

    AsyncPolicyResult result;
    result.epoch = request.epoch;
    result.sequence = request.sequence;
    result.request_timestamp_us = request.input.timestamp_us;
    result.worker_started_timestamp_us = monotonicTimeUs();
    result.worker_cpu = currentCpu();
    result.success = runner_.infer(
        request.input.base_linear_velocity, request.input.base_angular_velocity,
        request.input.projected_gravity, request.input.command,
        request.input.q_policy, request.input.qd_policy,
        request.input.last_action, &result.action, &result.target_q,
        &result.inference_time_ms, &result.error);
    result.completed_timestamp_us = monotonicTimeUs();

    std::lock_guard<std::mutex> lock(mutex_);
    worker_busy_ = false;
    in_flight_sequence_ = 0;
    if (!stop_requested_ && request.epoch == active_epoch_) {
      result.overwritten_request_count = overwritten_request_count_;
      latest_completed_sequence_ = request.sequence;
      worker_cpu_ = result.worker_cpu;
      latest_result_ = result;
      has_result_ = true;
    }
  }
}

}  // namespace rapid_rl
