include_directories("./")
include_directories("./Controllers")
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/FootstepPlanner")
include_directories("../../third-party/")
include_directories(SYSTEM "../../third-party/qpOASES/include")
include_directories("../../third-party/ParamHandler")

include_directories("../../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

if(USE_LIBTORCH_RL)
  include_directories(SYSTEM ${TORCH_INCLUDE_DIRS})
  set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,${LIBTORCH_ROOT}/lib")
  if(LIBTORCH_MKL_ROOT)
    set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,${LIBTORCH_MKL_ROOT}")
  endif()
endif()

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp" 
"Controllers/BalanceController/*.cpp" 
"Controllers/convexMPC/*.cpp")
list(REMOVE_ITEM sources
  "${CMAKE_CURRENT_SOURCE_DIR}/tools/rapid_rl_policy_benchmark.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/tools/rapid_rl_policy_config_test.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/tools/rapid_rl_async_policy_test.cpp"
  "./tools/rapid_rl_policy_benchmark.cpp"
  "./tools/rapid_rl_policy_config_test.cpp"
  "./tools/rapid_rl_async_policy_test.cpp"
  "tools/rapid_rl_policy_benchmark.cpp"
  "tools/rapid_rl_policy_config_test.cpp"
  "tools/rapid_rl_async_policy_test.cpp")
foreach(MIT_CTRL_SOURCE ${sources})
  if(MIT_CTRL_SOURCE MATCHES "rapid_rl_(policy_(benchmark|config_test)|async_policy_test)\\.cpp$")
    list(REMOVE_ITEM sources "${MIT_CTRL_SOURCE}")
  endif()
endforeach()

add_subdirectory(Controllers/WBC)
add_subdirectory(Controllers/WBC_Ctrl)
add_subdirectory(Controllers/VisionMPC)

add_executable(mit_ctrl ${sources} MIT_Controller.cpp main.cpp)
target_link_libraries(mit_ctrl robot biomimetics)
target_link_libraries(mit_ctrl dynacore_param_handler qpOASES)
target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
target_link_libraries(mit_ctrl WBC_Ctrl)
target_link_libraries(mit_ctrl VisionMPC)
target_link_libraries(mit_ctrl stdc++fs)
if(USE_LIBTORCH_RL)
  target_compile_definitions(mit_ctrl PRIVATE USE_LIBTORCH_RL=1)
  target_link_libraries(mit_ctrl ${TORCH_LIBRARIES})

  add_executable(rapid_rl_policy_benchmark
    tools/rapid_rl_policy_benchmark.cpp
    FSM_States/RapidRLPolicyRunner.cpp)
  target_compile_definitions(rapid_rl_policy_benchmark PRIVATE USE_LIBTORCH_RL=1)
  target_link_libraries(rapid_rl_policy_benchmark ${TORCH_LIBRARIES})

  add_executable(rapid_rl_policy_config_test
    tools/rapid_rl_policy_config_test.cpp)
  add_test(NAME legged_gym_rl_policy_config
    COMMAND rapid_rl_policy_config_test)

  add_executable(rapid_rl_async_policy_test
    tools/rapid_rl_async_policy_test.cpp
    FSM_States/RapidRLPolicyRunner.cpp
    FSM_States/RapidRLAsyncPolicyRunner.cpp)
  target_compile_definitions(rapid_rl_async_policy_test PRIVATE USE_LIBTORCH_RL=1)
  target_link_libraries(rapid_rl_async_policy_test ${TORCH_LIBRARIES})
  add_test(NAME legged_gym_rl_async_policy
    COMMAND rapid_rl_async_policy_test)
endif()
