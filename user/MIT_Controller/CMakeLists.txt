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

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp" 
"Controllers/BalanceController/*.cpp" 
"Controllers/convexMPC/*.cpp")
list(REMOVE_ITEM sources
  "${CMAKE_CURRENT_SOURCE_DIR}/tools/rapid_rl_policy_benchmark.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/tools/rapid_rl_policy_config_test.cpp"
  "./tools/rapid_rl_policy_benchmark.cpp"
  "./tools/rapid_rl_policy_config_test.cpp"
  "tools/rapid_rl_policy_benchmark.cpp"
  "tools/rapid_rl_policy_config_test.cpp")
foreach(MIT_CTRL_SOURCE ${sources})
  if(MIT_CTRL_SOURCE MATCHES "rapid_rl_policy_(benchmark|config_test)\\.cpp$")
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
target_link_libraries(mit_ctrl onnxruntime)

add_executable(rapid_rl_policy_benchmark
  tools/rapid_rl_policy_benchmark.cpp
  FSM_States/RapidRLPolicyRunner.cpp)
target_link_libraries(rapid_rl_policy_benchmark onnxruntime)

add_executable(rapid_rl_policy_config_test
  tools/rapid_rl_policy_config_test.cpp)
add_test(NAME legged_gym_rl_policy_config
  COMMAND rapid_rl_policy_config_test)

add_test(NAME himloco_onnx_policy_inference COMMAND rapid_rl_policy_benchmark 100)

# Recent ONNX Runtime headers require C++17; keep bundled dependencies on C++14.
set_target_properties(mit_ctrl rapid_rl_policy_benchmark PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
