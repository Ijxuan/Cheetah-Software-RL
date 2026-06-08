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
  if(LIBTORCH_ROOT)
    set(TORCH_INCLUDE_DIRS
      "${LIBTORCH_ROOT}/include"
      "${LIBTORCH_ROOT}/include/torch/csrc/api/include")
    set(TORCH_LIBRARIES
      "${LIBTORCH_ROOT}/lib/libtorch.so"
      "${LIBTORCH_ROOT}/lib/libtorch_cpu.so"
      "${LIBTORCH_ROOT}/lib/libc10.so")
    if(NOT LIBTORCH_CPU_ONLY)
      foreach(TORCH_OPTIONAL_LIB
          libtorch_cuda_cpp.so
          libtorch_cuda.so
          libtorch_cuda_cu.so
          libc10_cuda.so)
        if(EXISTS "${LIBTORCH_ROOT}/lib/${TORCH_OPTIONAL_LIB}")
          list(APPEND TORCH_LIBRARIES "${LIBTORCH_ROOT}/lib/${TORCH_OPTIONAL_LIB}")
        endif()
      endforeach()
    endif()
    set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,${LIBTORCH_ROOT}/lib")
  else()
    if(NOT TORCH_FOUND)
      find_package(Torch REQUIRED)
    endif()
  endif()
  include_directories(SYSTEM ${TORCH_INCLUDE_DIRS})
endif()

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp" 
"Controllers/BalanceController/*.cpp" 
"Controllers/convexMPC/*.cpp")

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
endif()
