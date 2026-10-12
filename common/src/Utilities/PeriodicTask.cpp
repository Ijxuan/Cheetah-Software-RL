/*!
 * @file PeriodicTask.cpp
 * @brief Implementation of a periodic function running in a separate thread.
 * Periodic tasks have a task manager, which measure how long they take to run.
 */
#ifdef linux
 #include <sys/timerfd.h>
 #include <pthread.h>
 #include <sched.h>
#endif

#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include "Utilities/PeriodicTask.h"
#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"


/*!
 * Construct a new task within a TaskManager
 * @param taskManager : Parent task manager
 * @param period : how often to run
 * @param name : name of task
 */
PeriodicTask::PeriodicTask(PeriodicTaskManager* taskManager, float period,
                           std::string name)
    : _period(period), _name(name) {
  taskManager->addTask(this);
}

/*!
 * Begin running task
 */
void PeriodicTask::start() {
  if (_running) {
    printf("[PeriodicTask] Tried to start %s but it was already running!\n",
           _name.c_str());
    return;
  }
  init();
  _running = true;
  _thread = std::thread(&PeriodicTask::loopFunction, this);
}

void PeriodicTask::setThreadRealtimePriority(int fifo_priority) {
  if (_running) {
    throw std::logic_error("Cannot change periodic task priority after start");
  }
  _threadFifoPriority = fifo_priority;
}

void PeriodicTask::setThreadCpuAffinity(int cpu_core) {
  if (_running) {
    throw std::logic_error("Cannot change periodic task affinity after start");
  }
  _threadCpuCore = cpu_core;
}

/*!
 * Stop running task
 */
void PeriodicTask::stop() {
  if (!_running) {
    printf("[PeriodicTask] Tried to stop %s but it wasn't running!\n",
           _name.c_str());
    return;
  }
  _running = false;
  printf("[PeriodicTask] Waiting for %s to stop...\n", _name.c_str());
  _thread.join();
  printf("[PeriodicTask] Done!\n");
  cleanup();
}

/*!
 * If max period is more than 30% over desired period, it is slow
 */
bool PeriodicTask::isSlow() {
  return _maxPeriod > _period * 1.3f || _maxRuntime > _period;
}

/*!
 * Reset max statistics
 */
void PeriodicTask::clearMax() {
  _maxPeriod = 0;
  _maxRuntime = 0;
  _maxMissedPeriods = 0;
}

/*!
 * Print the status of this task in the table format
 */
void PeriodicTask::printStatus() {
  if (!_running) return;
  if (isSlow()) {
        printf_color(PrintColor::Red, "|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f|%llu\n",
                 _name.c_str(), _lastRuntime, _maxRuntime, _period,
           _lastPeriodTime, _maxPeriod,
           static_cast<unsigned long long>(_maxMissedPeriods));
  } else {
    printf("|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f|%llu\n", _name.c_str(),
          _lastRuntime, _maxRuntime, _period, _lastPeriodTime, _maxPeriod,
          static_cast<unsigned long long>(_maxMissedPeriods));
  }
}

/*!
 * Call the task in a timed loop.  Uses a timerfd
 */
void PeriodicTask::loopFunction() {
#ifdef linux
  if (_threadCpuCore >= 0) {
    if (_threadCpuCore >= CPU_SETSIZE) {
      fprintf(stderr,
              "[PeriodicTask] FATAL: %s CPU %d exceeds CPU_SETSIZE\n",
              _name.c_str(), _threadCpuCore);
      std::abort();
    }
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(_threadCpuCore, &cpu_set);
    const int affinity_result =
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set);
    if (affinity_result != 0) {
      fprintf(stderr,
              "[PeriodicTask] FATAL: cannot pin %s to CPU %d: %s\n",
              _name.c_str(), _threadCpuCore, strerror(affinity_result));
      std::abort();
    }
  }

  if (_threadFifoPriority >= 0) {
    sched_param params;
    params.sched_priority = _threadFifoPriority;
    const int scheduler_result =
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
    if (scheduler_result != 0) {
      fprintf(stderr,
              "[PeriodicTask] FATAL: cannot set %s to SCHED_FIFO %d: %s\n",
              _name.c_str(), _threadFifoPriority,
              strerror(scheduler_result));
      std::abort();
    }
  }

  int actual_policy = 0;
  sched_param actual_params;
  actual_params.sched_priority = 0;
  const int verify_result =
      pthread_getschedparam(pthread_self(), &actual_policy, &actual_params);
  const int actual_cpu = sched_getcpu();
  if (verify_result != 0) {
    fprintf(stderr, "[PeriodicTask] FATAL: cannot verify %s scheduler: %s\n",
            _name.c_str(), strerror(verify_result));
    std::abort();
  }
  printf("[PeriodicTask] Scheduling verified: %s policy=%s priority=%d cpu=%d"
         " requested_cpu=%d\n",
         _name.c_str(), actual_policy == SCHED_FIFO ? "SCHED_FIFO" : "OTHER",
         actual_params.sched_priority, actual_cpu, _threadCpuCore);
  fflush(stdout);
  if (_threadFifoPriority >= 0 &&
      (actual_policy != SCHED_FIFO ||
       actual_params.sched_priority != _threadFifoPriority)) {
    fprintf(stderr,
            "[PeriodicTask] FATAL: %s scheduler verification mismatch\n",
            _name.c_str());
    std::abort();
  }
  if (_threadCpuCore >= 0 && actual_cpu != _threadCpuCore) {
    fprintf(stderr,
            "[PeriodicTask] FATAL: %s started on CPU %d instead of CPU %d\n",
            _name.c_str(), actual_cpu, _threadCpuCore);
    std::abort();
  }

  auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
#endif
  int seconds = (int)_period;
  int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));

  Timer t;

#ifdef linux
  itimerspec timerSpec;
  timerSpec.it_interval.tv_sec = seconds;
  timerSpec.it_value.tv_sec = seconds;
  timerSpec.it_value.tv_nsec = nanoseconds;
  timerSpec.it_interval.tv_nsec = nanoseconds;

  timerfd_settime(timerFd, 0, &timerSpec, nullptr);
#endif
  unsigned long long missed = 0;

  printf("[PeriodicTask] Start %s (%d s, %d ns)\n", _name.c_str(), seconds,
         nanoseconds);
  while (_running) {
    _lastPeriodTime = (float)t.getSeconds();
    t.start();
    run();
    _lastRuntime = (float)t.getSeconds();
#ifdef linux
    int m = read(timerFd, &missed, sizeof(missed));
    if (m == sizeof(missed)) {
      const uint64_t missedPeriods = missed > 0 ? missed - 1 : 0;
      _maxMissedPeriods = std::max(
          _maxMissedPeriods, static_cast<uint64_t>(missedPeriods));
    }
#endif
    _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
    _maxRuntime = std::max(_maxRuntime, _lastRuntime);
  }
  printf("[PeriodicTask] %s has stopped!\n", _name.c_str());
}

PeriodicTaskManager::~PeriodicTaskManager() {}

/*!
 * Add a new task to a task manager
 */
void PeriodicTaskManager::addTask(PeriodicTask* task) {
  _tasks.push_back(task);
}

/*!
 * Print the status of all tasks and rest max statistics
 */
void PeriodicTaskManager::printStatus() {
  printf("\n----------------------------TASKS----------------------------\n");
    printf("|%-20s|%-6s|%-6s|%-6s|%-6s|%-6s|%-6s\n", "name", "rt", "rt-max",
      "T-des", "T-act", "T-max", "miss-max");
  printf("-----------------------------------------------------------\n");
  for (auto& task : _tasks) {
    task->printStatus();
    task->clearMax();
  }
  printf("-------------------------------------------------------------\n\n");
}

/*!
 * Print only the slow tasks
 */
void PeriodicTaskManager::printStatusOfSlowTasks() {
  for (auto& task : _tasks) {
    if (task->isSlow()) {
      task->printStatus();
      task->clearMax();
    }
  }
}

/*!
 * Stop all tasks
 */
void PeriodicTaskManager::stopAll() {
  for (auto& task : _tasks) {
    task->stop();
  }
}
