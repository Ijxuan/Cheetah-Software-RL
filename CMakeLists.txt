cmake_minimum_required(VERSION 3.5)
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
option(USE_LIBTORCH_RL "Build the MIT controller with the in-process LibTorch RL backend" ON)
set(LIBTORCH_ROOT "" CACHE PATH "LibTorch package root used for RL_JOINT_PD deployment")
option(LIBTORCH_CPU_ONLY "Require a CPU-only LibTorch package for RL_JOINT_PD" ON)
set(LIBTORCH_CXX11_ABI "" CACHE STRING "Explicit _GLIBCXX_USE_CXX11_ABI value for LibTorch if Torch cannot report it")
set(RAPID_RL_CHECKPOINT_DIR "${PROJECT_SOURCE_DIR}/rl-checkpoints" CACHE PATH "Directory containing rapid RL TorchScript checkpoints")
set(RAPID_RL_WARMUP_ITERS "20" CACHE STRING "Warmup iterations before running the rapid RL policy")
set(RAPID_RL_MAX_INFERENCE_MS "20.0" CACHE STRING "Soft warning threshold for one rapid RL inference")
set(RAPID_RL_TORCH_NUM_THREADS "1" CACHE STRING "Torch intra-op CPU thread count for rapid RL inference")
option(RAPID_RL_DEBUG_LCM_STATE "Publish read-only rl_robot_state LCM debug state" ON)

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

if(NOT USE_LIBTORCH_RL)
  message(FATAL_ERROR "RL_JOINT_PD is LibTorch-only; configure with USE_LIBTORCH_RL=ON")
endif()

if(NOT LIBTORCH_ROOT)
  message(FATAL_ERROR "LIBTORCH_ROOT is required for RL_JOINT_PD deployment")
endif()
if(NOT IS_DIRECTORY "${LIBTORCH_ROOT}")
  message(FATAL_ERROR "LIBTORCH_ROOT does not exist: ${LIBTORCH_ROOT}")
endif()

foreach(RAPID_RL_TORCH_REQUIRED_LIB
    libtorch.so
    libtorch_cpu.so
    libc10.so)
  if(NOT EXISTS "${LIBTORCH_ROOT}/lib/${RAPID_RL_TORCH_REQUIRED_LIB}")
    message(FATAL_ERROR
      "LIBTORCH_ROOT is missing ${RAPID_RL_TORCH_REQUIRED_LIB}: ${LIBTORCH_ROOT}/lib")
  endif()
endforeach()

if(LIBTORCH_CPU_ONLY)
  file(GLOB RAPID_RL_CUDA_TORCH_LIBS
    "${LIBTORCH_ROOT}/lib/libtorch_cuda*.so"
    "${LIBTORCH_ROOT}/lib/libc10_cuda.so")
  if(RAPID_RL_CUDA_TORCH_LIBS)
    message(FATAL_ERROR
      "LIBTORCH_CPU_ONLY=ON but CUDA LibTorch libraries were found: ${RAPID_RL_CUDA_TORCH_LIBS}")
  endif()
endif()

if(NOT EXISTS "${LIBTORCH_ROOT}/share/cmake/Torch/TorchConfig.cmake")
  message(FATAL_ERROR
    "LIBTORCH_ROOT is missing share/cmake/Torch/TorchConfig.cmake: ${LIBTORCH_ROOT}")
endif()

set(TORCH_INCLUDE_DIRS
  "${LIBTORCH_ROOT}/include"
  "${LIBTORCH_ROOT}/include/torch/csrc/api/include")
set(TORCH_LIBRARIES
  "${LIBTORCH_ROOT}/lib/libtorch.so"
  "${LIBTORCH_ROOT}/lib/libtorch_cpu.so"
  "${LIBTORCH_ROOT}/lib/libc10.so")
if(NOT LIBTORCH_CPU_ONLY)
  foreach(RAPID_RL_TORCH_OPTIONAL_LIB
      libtorch_cuda_cpp.so
      libtorch_cuda.so
      libtorch_cuda_cu.so
      libc10_cuda.so)
    if(EXISTS "${LIBTORCH_ROOT}/lib/${RAPID_RL_TORCH_OPTIONAL_LIB}")
      list(APPEND TORCH_LIBRARIES
        "${LIBTORCH_ROOT}/lib/${RAPID_RL_TORCH_OPTIONAL_LIB}")
    endif()
  endforeach()
endif()

file(READ "${LIBTORCH_ROOT}/share/cmake/Torch/TorchConfig.cmake"
  RAPID_RL_TORCH_CONFIG_TEXT)
set(TORCH_CXX_FLAGS "")
if(RAPID_RL_TORCH_CONFIG_TEXT MATCHES "_GLIBCXX_USE_CXX11_ABI=([01])")
  set(TORCH_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=${CMAKE_MATCH_1}")
endif()

set(RAPID_RL_INFERRED_CXX11_ABI "")
if(TORCH_CXX_FLAGS MATCHES "_GLIBCXX_USE_CXX11_ABI=([01])")
  set(RAPID_RL_INFERRED_CXX11_ABI "${CMAKE_MATCH_1}")
endif()

if(NOT "${LIBTORCH_CXX11_ABI}" STREQUAL "")
  if(NOT LIBTORCH_CXX11_ABI MATCHES "^[01]$")
    message(FATAL_ERROR "LIBTORCH_CXX11_ABI must be 0 or 1, got '${LIBTORCH_CXX11_ABI}'")
  endif()
  if(NOT "${RAPID_RL_INFERRED_CXX11_ABI}" STREQUAL "" AND
      NOT "${LIBTORCH_CXX11_ABI}" STREQUAL "${RAPID_RL_INFERRED_CXX11_ABI}")
    message(FATAL_ERROR
      "LIBTORCH_CXX11_ABI=${LIBTORCH_CXX11_ABI} conflicts with Torch CMake flags ABI ${RAPID_RL_INFERRED_CXX11_ABI}")
  endif()
  set(RAPID_RL_LIBTORCH_CXX11_ABI "${LIBTORCH_CXX11_ABI}")
elseif(NOT "${RAPID_RL_INFERRED_CXX11_ABI}" STREQUAL "")
  set(RAPID_RL_LIBTORCH_CXX11_ABI "${RAPID_RL_INFERRED_CXX11_ABI}")
else()
  message(FATAL_ERROR
    "Could not infer _GLIBCXX_USE_CXX11_ABI from TORCH_CXX_FLAGS; pass -DLIBTORCH_CXX11_ABI=0 or 1")
endif()

if(NOT RAPID_RL_WARMUP_ITERS MATCHES "^[0-9]+$")
  message(FATAL_ERROR "RAPID_RL_WARMUP_ITERS must be a non-negative integer")
endif()
if(NOT RAPID_RL_TORCH_NUM_THREADS MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "RAPID_RL_TORCH_NUM_THREADS must be a positive integer")
endif()
if(NOT RAPID_RL_MAX_INFERENCE_MS MATCHES "^[0-9]+(\\.[0-9]+)?$")
  message(FATAL_ERROR "RAPID_RL_MAX_INFERENCE_MS must be a non-negative number")
endif()

get_filename_component(RAPID_RL_CHECKPOINT_DIR
  "${RAPID_RL_CHECKPOINT_DIR}" ABSOLUTE BASE_DIR "${PROJECT_SOURCE_DIR}")
set(RAPID_RL_CHECKPOINT_DIR "${RAPID_RL_CHECKPOINT_DIR}" CACHE PATH
  "Directory containing rapid RL TorchScript checkpoints" FORCE)
foreach(RAPID_RL_CHECKPOINT_FILE
    adaptation_module_latest.jit
    body_latest.jit)
  if(NOT EXISTS "${RAPID_RL_CHECKPOINT_DIR}/${RAPID_RL_CHECKPOINT_FILE}")
    message(FATAL_ERROR
      "Missing rapid RL checkpoint ${RAPID_RL_CHECKPOINT_DIR}/${RAPID_RL_CHECKPOINT_FILE}")
  endif()
endforeach()

add_definitions(-DUSE_LIBTORCH_RL=1)
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=${RAPID_RL_LIBTORCH_CXX11_ABI})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

set(RAPID_RL_USE_LIBTORCH_VALUE 1)
if(LIBTORCH_CPU_ONLY)
  set(RAPID_RL_LIBTORCH_CPU_ONLY_VALUE 1)
else()
  set(RAPID_RL_LIBTORCH_CPU_ONLY_VALUE 0)
endif()
if(RAPID_RL_DEBUG_LCM_STATE)
  set(RAPID_RL_DEBUG_LCM_STATE_VALUE 1)
else()
  set(RAPID_RL_DEBUG_LCM_STATE_VALUE 0)
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/RapidRLBuildConfig.h.cmake
  ${CMAKE_BINARY_DIR}/RapidRLBuildConfig.h @ONLY)

message(STATUS "Rapid RL checkpoint dir: ${RAPID_RL_CHECKPOINT_DIR}")
message(STATUS "Rapid RL LibTorch ABI: _GLIBCXX_USE_CXX11_ABI=${RAPID_RL_LIBTORCH_CXX11_ABI}")
message(STATUS "Rapid RL LibTorch CPU-only: ${LIBTORCH_CPU_ONLY}")

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
