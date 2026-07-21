# Resolve per-machine Rapid RL dependencies without embedding a workspace name
# in normal build commands.  A valid explicit/cache value has priority; stale
# values are skipped so an old CMakeCache.txt can repair itself.

set(RAPID_RL_LOCAL_CONFIG_FILE
    "${CMAKE_SOURCE_DIR}/cmake/RapidRLLocalConfig.cmake")
if(EXISTS "${RAPID_RL_LOCAL_CONFIG_FILE}")
  message(STATUS "Rapid RL: loading local dependency settings from ${RAPID_RL_LOCAL_CONFIG_FILE}")
  include("${RAPID_RL_LOCAL_CONFIG_FILE}")
endif()

function(rapid_rl_valid_libtorch_root candidate out_var)
  if("${candidate}" STREQUAL "")
    set(${out_var} "" PARENT_SCOPE)
    return()
  endif()

  get_filename_component(_rapid_rl_root "${candidate}" ABSOLUTE
    BASE_DIR "${CMAKE_SOURCE_DIR}")
  if(EXISTS "${_rapid_rl_root}/lib/libtorch.so" AND
     EXISTS "${_rapid_rl_root}/lib/libtorch_cpu.so" AND
     EXISTS "${_rapid_rl_root}/lib/libc10.so" AND
     EXISTS "${_rapid_rl_root}/share/cmake/Torch/TorchConfig.cmake")
    set(${out_var} "${_rapid_rl_root}" PARENT_SCOPE)
  else()
    set(${out_var} "" PARENT_SCOPE)
  endif()
endfunction()

function(rapid_rl_valid_mkl_root candidate out_var)
  if("${candidate}" STREQUAL "")
    set(${out_var} "" PARENT_SCOPE)
    return()
  endif()

  get_filename_component(_rapid_rl_root "${candidate}" ABSOLUTE
    BASE_DIR "${CMAKE_SOURCE_DIR}")
  set(_rapid_rl_mkl_valid TRUE)
  foreach(_rapid_rl_mkl_library
      libmkl_intel_lp64.so
      libmkl_gnu_thread.so
      libmkl_core.so)
    if(NOT EXISTS "${_rapid_rl_root}/${_rapid_rl_mkl_library}" AND
       NOT EXISTS "${_rapid_rl_root}/${_rapid_rl_mkl_library}.2")
      set(_rapid_rl_mkl_valid FALSE)
    endif()
  endforeach()

  if(_rapid_rl_mkl_valid)
    set(${out_var} "${_rapid_rl_root}" PARENT_SCOPE)
  else()
    set(${out_var} "" PARENT_SCOPE)
  endif()
endfunction()

function(rapid_rl_resolve_dependency_paths)
  # Keep a valid -D/cache path first.  The remaining candidates use only the
  # source location, so changing the name of the enclosing workspace is safe.
  set(_rapid_rl_libtorch_candidates "")
  if(LIBTORCH_ROOT)
    list(APPEND _rapid_rl_libtorch_candidates "${LIBTORCH_ROOT}")
  endif()
  if(RAPID_RL_LOCAL_LIBTORCH_ROOT)
    list(APPEND _rapid_rl_libtorch_candidates
      "${RAPID_RL_LOCAL_LIBTORCH_ROOT}")
  endif()
  if(NOT "$ENV{LIBTORCH_ROOT}" STREQUAL "")
    list(APPEND _rapid_rl_libtorch_candidates "$ENV{LIBTORCH_ROOT}")
  endif()
  list(APPEND _rapid_rl_libtorch_candidates
    "${CMAKE_SOURCE_DIR}/.deps/libtorch-1.10.1-cpu"
    "${CMAKE_SOURCE_DIR}/../.deps/libtorch-1.10.1-cpu"
    "/opt/libtorch-1.10.1-cpu")

  set(_rapid_rl_resolved_libtorch_root "")
  foreach(_rapid_rl_candidate ${_rapid_rl_libtorch_candidates})
    rapid_rl_valid_libtorch_root("${_rapid_rl_candidate}"
      _rapid_rl_valid_root)
    if(_rapid_rl_valid_root)
      set(_rapid_rl_resolved_libtorch_root "${_rapid_rl_valid_root}")
      break()
    endif()
  endforeach()

  if(NOT _rapid_rl_resolved_libtorch_root)
    message(FATAL_ERROR
      "Could not find a usable CPU LibTorch package. Put it at "
      "${CMAKE_SOURCE_DIR}/.deps/libtorch-1.10.1-cpu (recommended), set "
      "LIBTORCH_ROOT, or create cmake/RapidRLLocalConfig.cmake from the "
      "provided .example file.")
  endif()

  if(NOT "${LIBTORCH_ROOT}" STREQUAL "${_rapid_rl_resolved_libtorch_root}")
    message(STATUS "Rapid RL: using LibTorch at ${_rapid_rl_resolved_libtorch_root}")
  endif()
  set(LIBTORCH_ROOT "${_rapid_rl_resolved_libtorch_root}" CACHE PATH
    "LibTorch package root used for RL_JOINT_PD deployment" FORCE)

  # Some CPU LibTorch packages need external MKL at final link.  An invalid
  # old cache is ignored rather than making every incremental build fail.
  set(_rapid_rl_mkl_candidates "")
  if(LIBTORCH_MKL_ROOT)
    list(APPEND _rapid_rl_mkl_candidates "${LIBTORCH_MKL_ROOT}")
  endif()
  if(RAPID_RL_LOCAL_MKL_ROOT)
    list(APPEND _rapid_rl_mkl_candidates "${RAPID_RL_LOCAL_MKL_ROOT}")
  endif()
  if(NOT "$ENV{LIBTORCH_MKL_ROOT}" STREQUAL "")
    list(APPEND _rapid_rl_mkl_candidates "$ENV{LIBTORCH_MKL_ROOT}")
  endif()
  if(NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
    list(APPEND _rapid_rl_mkl_candidates "$ENV{CONDA_PREFIX}/lib")
  endif()
  list(APPEND _rapid_rl_mkl_candidates
    "${LIBTORCH_ROOT}/lib"
    "${CMAKE_SOURCE_DIR}/.deps/mkl/lib"
    "${CMAKE_SOURCE_DIR}/../.deps/mkl/lib")

  set(_rapid_rl_resolved_mkl_root "")
  foreach(_rapid_rl_candidate ${_rapid_rl_mkl_candidates})
    rapid_rl_valid_mkl_root("${_rapid_rl_candidate}" _rapid_rl_valid_root)
    if(_rapid_rl_valid_root)
      set(_rapid_rl_resolved_mkl_root "${_rapid_rl_valid_root}")
      break()
    endif()
  endforeach()

  if(_rapid_rl_resolved_mkl_root)
    if(NOT "${LIBTORCH_MKL_ROOT}" STREQUAL "${_rapid_rl_resolved_mkl_root}")
      message(STATUS "Rapid RL: using MKL at ${_rapid_rl_resolved_mkl_root}")
    endif()
    set(LIBTORCH_MKL_ROOT "${_rapid_rl_resolved_mkl_root}" CACHE PATH
      "Optional directory containing Intel MKL shared libraries required by some CPU LibTorch packages" FORCE)
  elseif(LIBTORCH_MKL_ROOT)
    message(WARNING
      "Rapid RL: ignoring unavailable LIBTORCH_MKL_ROOT='${LIBTORCH_MKL_ROOT}'. "
      "If linking reports mkl_* symbols, set RAPID_RL_LOCAL_MKL_ROOT in "
      "cmake/RapidRLLocalConfig.cmake.")
    set(LIBTORCH_MKL_ROOT "" CACHE PATH
      "Optional directory containing Intel MKL shared libraries required by some CPU LibTorch packages" FORCE)
  endif()
endfunction()
