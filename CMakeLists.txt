cmake_minimum_required(VERSION 3.5)
project(CheetahSoftwareRL LANGUAGES C CXX)
# CMake 4 removed compatibility for several bundled third-party projects whose
# declared policy version is 3.5.  Keep this project-level compatibility knob
# here so operators do not need to repeat it on every configure command.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()
enable_testing()
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)
#execute_process(COMMAND ../scripts/make_types.sh)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#set(CMAKE_VERBOSE_MAKEFILE ON)

option(MINI_CHEETAH_BUILD "use compiler flags for mini cheetah computer" OFF)
set(BUILD_TYPE_RELEASE TRUE)

option(NO_SIM "Do not build simulator" OFF)
set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime C/C++ CPU package root")
set(RAPID_RL_CHECKPOINT_DIR "${PROJECT_SOURCE_DIR}/rl-checkpoints/onnx" CACHE PATH "Directory containing HIMLoco ONNX policy")
set(RAPID_RL_CHECKPOINT_FILE "policy.onnx" CACHE STRING "HIMLoco ONNX policy filename")
set(RL_FLAT_MODEL_DIR "${PROJECT_SOURCE_DIR}/actor_model" CACHE PATH "Legacy flat actor_5000/estimator_5000 and normalization directory")
set(RAPID_RL_WARMUP_ITERS "20" CACHE STRING "Warmup iterations before running the rapid RL policy")
set(RAPID_RL_MAX_INFERENCE_MS "20.0" CACHE STRING "Soft warning threshold for one rapid RL inference")
set(RAPID_RL_ONNX_NUM_THREADS "1" CACHE STRING "ONNX Runtime intra-op CPU thread count for rapid RL inference")
option(RAPID_RL_DEBUG_LCM_STATE "Publish read-only rl_robot_state LCM debug state" ON)
set(RAPID_RL_ETHERCAT_CPU_CORE "0" CACHE STRING "Dedicated logical CPU for the EtherCAT task")
set(RAPID_RL_ETHERCAT_RT_PRIORITY "99" CACHE STRING "SCHED_FIFO priority for EtherCAT (1-99)")

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/RapidRLDependencyPaths.cmake")

if(MINI_CHEETAH_BUILD)
  SET (THIS_COM "../" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)
  set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wshadow -Wsign-promo -Werror")
  set(CMAKE_C_FLAGS "-O3  -ggdb -std=gnu99 -I.")
  message("**** Mini-Cheetah build enabled ****")
else(MINI_CHEETAH_BUILD)
  SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)

  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wshadow -Wsign-promo -Werror")
  elseif(APPLE)
    set(CMAKE_CXX_FLAGS "-O3 -march=native -ggdb -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wshadow -Wsign-promo")
    include_directories("/usr/local/include/")   # lcm includes
  endif()

  set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
  message("**** Mini-Cheetah build disabled ****")
endif(MINI_CHEETAH_BUILD)

rapid_rl_resolve_dependency_paths()

if(NOT RAPID_RL_WARMUP_ITERS MATCHES "^[0-9]+$")
  message(FATAL_ERROR "RAPID_RL_WARMUP_ITERS must be a non-negative integer")
endif()
if(NOT RAPID_RL_ONNX_NUM_THREADS MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "RAPID_RL_ONNX_NUM_THREADS must be a positive integer")
endif()
if(NOT RAPID_RL_MAX_INFERENCE_MS MATCHES "^[0-9]+(\\.[0-9]+)?$")
  message(FATAL_ERROR "RAPID_RL_MAX_INFERENCE_MS must be a non-negative number")
endif()
if(NOT RAPID_RL_ETHERCAT_CPU_CORE MATCHES "^[0-9]+$")
  message(FATAL_ERROR "RAPID_RL_ETHERCAT_CPU_CORE must be a non-negative integer")
endif()
if(NOT RAPID_RL_ETHERCAT_RT_PRIORITY MATCHES "^[1-9][0-9]*$" OR
   RAPID_RL_ETHERCAT_RT_PRIORITY GREATER 99)
  message(FATAL_ERROR "RAPID_RL_ETHERCAT_RT_PRIORITY must be an integer from 1 to 99")
endif()

get_filename_component(RAPID_RL_CHECKPOINT_DIR
  "${RAPID_RL_CHECKPOINT_DIR}" ABSOLUTE BASE_DIR "${PROJECT_SOURCE_DIR}")
set(RAPID_RL_CHECKPOINT_DIR "${RAPID_RL_CHECKPOINT_DIR}" CACHE PATH
  "Directory containing HIMLoco ONNX policy" FORCE)
if(NOT EXISTS "${RAPID_RL_CHECKPOINT_DIR}/${RAPID_RL_CHECKPOINT_FILE}")
  message(FATAL_ERROR
    "Missing HIMLoco RL checkpoint ${RAPID_RL_CHECKPOINT_DIR}/${RAPID_RL_CHECKPOINT_FILE}")
endif()

if(RAPID_RL_DEBUG_LCM_STATE)
  set(RAPID_RL_DEBUG_LCM_STATE_VALUE 1)
else()
  set(RAPID_RL_DEBUG_LCM_STATE_VALUE 0)
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/RapidRLBuildConfig.h.cmake
  ${CMAKE_BINARY_DIR}/RapidRLBuildConfig.h @ONLY)

message(STATUS "Rapid RL checkpoint dir: ${RAPID_RL_CHECKPOINT_DIR}")
message(STATUS "Rapid RL ONNX Runtime: ${ONNXRUNTIME_ROOT}")

set(CMAKE_CXX_STANDARD 14)

#find_package(lcm)

add_subdirectory(robot)
add_subdirectory(third-party)
add_subdirectory(common)

if(NO_SIM)

else(NO_SIM)
  add_subdirectory(sim)
endif()

add_subdirectory(user)
add_subdirectory(rc_test)
