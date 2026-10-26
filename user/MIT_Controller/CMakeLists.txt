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
  if(MIT_CTRL_SOURCE MATCHES "tools/.*\\.cpp$")
    list(REMOVE_ITEM sources "${MIT_CTRL_SOURCE}")
  endif()
endforeach()

add_subdirectory(Controllers/WBC)
add_subdirectory(Controllers/WBC_Ctrl)
add_subdirectory(Controllers/VisionMPC)

list(FILTER sources EXCLUDE REGEX "/main\\.cpp$")
# Production and integration tests link the exact same FSM implementation.
add_library(mit_controller_core STATIC ${sources})
target_link_libraries(mit_controller_core PUBLIC robot biomimetics
  dynacore_param_handler qpOASES Goldfarb_Optimizer osqp WBC_Ctrl VisionMPC
  stdc++fs onnxruntime)
add_executable(mit_ctrl main.cpp)
target_link_libraries(mit_ctrl mit_controller_core)

add_executable(rapid_rl_policy_benchmark
  tools/rapid_rl_policy_benchmark.cpp
  FSM_States/RapidRLPolicyRunner.cpp)
target_link_libraries(rapid_rl_policy_benchmark onnxruntime)

add_executable(rapid_rl_policy_config_test
  tools/rapid_rl_policy_config_test.cpp)
add_test(NAME legged_gym_rl_policy_config
  COMMAND rapid_rl_policy_config_test)

add_test(NAME himloco_onnx_policy_inference COMMAND rapid_rl_policy_benchmark 100)

add_executable(dual_rl_policy_test tools/dual_rl_policy_test.cpp
  FSM_States/FlatPolicyRunner.cpp FSM_States/RapidRLPolicyRunner.cpp)
target_link_libraries(dual_rl_policy_test onnxruntime)
add_test(NAME dual_rl_policy COMMAND dual_rl_policy_test)

add_executable(dual_rl_controller_test tools/dual_rl_controller_test.cpp)
target_link_libraries(dual_rl_controller_test mit_controller_core)
add_test(NAME dual_rl_controller COMMAND dual_rl_controller_test)

# Recent ONNX Runtime headers require C++17; keep bundled dependencies on C++14.
set_target_properties(mit_ctrl mit_controller_core rapid_rl_policy_benchmark dual_rl_policy_test dual_rl_controller_test PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
