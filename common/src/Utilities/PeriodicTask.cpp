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
#include <cmath>
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

#ifdef linux
  if (_threadCpuCore >= 0) {
    if (_threadCpuCore >= CPU_SETSIZE) {
      _running = false;
      _thread.join();
      cleanup();
      throw std::runtime_error("PeriodicTask 指定的 CPU 核超出 CPU_SETSIZE 范围");
    }
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(_threadCpuCore, &cpu_set);
    const int result = pthread_setaffinity_np(
        _thread.native_handle(), sizeof(cpu_set), &cpu_set);
    if (result != 0) {
      _running = false;
      _thread.join();
      cleanup();
      throw std::runtime_error("无法将周期任务 " + _name + " 绑定到 CPU " +
                               std::to_string(_threadCpuCore));
    }
  }
  if (_threadFifoPriority >= 0) {
    if (_threadFifoPriority < 1 || _threadFifoPriority > 99) {
      _running = false;
      _thread.join();
      cleanup();
      throw std::runtime_error("PeriodicTask 的 SCHED_FIFO 优先级必须为 1-99");
    }
    sched_param params;
    params.sched_priority = _threadFifoPriority;
    const int result = pthread_setschedparam(
        _thread.native_handle(), SCHED_FIFO, &params);
    if (result != 0) {
      _running = false;
      _thread.join();
      cleanup();
      throw std::runtime_error("无法为周期任务设置 SCHED_FIFO：" + _name);
    }
  }
#endif
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
}

/*!
 * Print the status of this task in the table format
 */
void PeriodicTask::printStatus() {
  if (!_running) return;
  if (isSlow()) {
    printf_color(PrintColor::Red, "|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f\n",
                 _name.c_str(), _lastRuntime, _maxRuntime, _period,
                 _lastPeriodTime, _maxPeriod);
  } else {
    printf("|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f\n", _name.c_str(),
           _lastRuntime, _maxRuntime, _period, _lastPeriodTime, _maxPeriod);
  }
}

/*!
 * Call the task in a timed loop.  Uses a timerfd
 */
void PeriodicTask::loopFunction() {
#ifdef linux
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
    (void)m;
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
  printf("|%-20s|%-6s|%-6s|%-6s|%-6s|%-6s\n", "name", "rt", "rt-max", "T-des",
         "T-act", "T-max");
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
