# Keep machine-specific paths out of tracked source.
include("${CMAKE_SOURCE_DIR}/cmake/RapidRLLocalConfig.cmake" OPTIONAL)
function(rapid_rl_resolve_dependency_paths)
  set(candidates "${ONNXRUNTIME_ROOT}" "${RAPID_RL_LOCAL_ONNXRUNTIME_ROOT}"
    "$ENV{ONNXRUNTIME_ROOT}" "${CMAKE_SOURCE_DIR}/.deps/onnxruntime"
    "${CMAKE_SOURCE_DIR}/../.deps/onnxruntime" "/opt/onnxruntime")
  foreach(candidate ${candidates})
    if(EXISTS "${candidate}/include/onnxruntime_cxx_api.h" AND
       EXISTS "${candidate}/lib/libonnxruntime.so")
      get_filename_component(root "${candidate}" ABSOLUTE)
      set(ONNXRUNTIME_ROOT "${root}" CACHE PATH "ONNX Runtime C/C++ CPU package root" FORCE)
      if(NOT TARGET onnxruntime)
        add_library(onnxruntime SHARED IMPORTED GLOBAL)
        set_target_properties(onnxruntime PROPERTIES
          IMPORTED_LOCATION "${root}/lib/libonnxruntime.so"
          INTERFACE_INCLUDE_DIRECTORIES "${root}/include")
      endif()
      return()
    endif()
  endforeach()
  message(FATAL_ERROR "ONNX Runtime C/C++ package not found. Set ONNXRUNTIME_ROOT, or place include/ and lib/ under .deps/onnxruntime.")
endfunction()
