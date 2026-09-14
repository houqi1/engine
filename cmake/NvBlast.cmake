# Pins NVIDIA Blast Low Level + ExtStress from NVIDIA-Omniverse/PhysX.
# Source is sparse-cloned into the build tree; it is not vendored in git.

set(VE_NVBLAST_GIT_URL "https://github.com/NVIDIA-Omniverse/PhysX.git")
set(VE_NVBLAST_GIT_SHA "7ef568f5b557a6dad9023ecebc43cb809270e035")
set(VE_NVBLAST_VERSION "5.0.6")
set(VE_NVBLAST_SRC "${CMAKE_BINARY_DIR}/_deps/nvblast-src")
set(VE_BLAST_ROOT "${VE_NVBLAST_SRC}/blast")

function(ve_ensure_nvblast)
  find_package(Git REQUIRED)
  set(_have_src FALSE)
  if(EXISTS "${VE_BLAST_ROOT}/include/lowlevel/NvBlast.h" AND EXISTS "${VE_NVBLAST_SRC}/.git")
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
      WORKING_DIRECTORY "${VE_NVBLAST_SRC}"
      OUTPUT_VARIABLE _have_sha
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _rev_ok
    )
    if(_rev_ok EQUAL 0 AND _have_sha STREQUAL VE_NVBLAST_GIT_SHA)
      set(_have_src TRUE)
    endif()
  endif()

  if(_have_src)
    return()
  endif()

  if(EXISTS "${VE_NVBLAST_SRC}")
    file(REMOVE_RECURSE "${VE_NVBLAST_SRC}")
  endif()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/_deps")

  message(STATUS "Fetching NvBlast ${VE_NVBLAST_VERSION} at ${VE_NVBLAST_GIT_SHA}")
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" clone --filter=blob:none --sparse --no-checkout
            "${VE_NVBLAST_GIT_URL}" "${VE_NVBLAST_SRC}"
    RESULT_VARIABLE _ok
  )
  if(NOT _ok EQUAL 0)
    message(FATAL_ERROR "git clone of NVIDIA-Omniverse/PhysX failed")
  endif()
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" sparse-checkout init --cone
    WORKING_DIRECTORY "${VE_NVBLAST_SRC}"
    RESULT_VARIABLE _ok
  )
  if(NOT _ok EQUAL 0)
    message(FATAL_ERROR "git sparse-checkout init failed")
  endif()
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" sparse-checkout set blast
    WORKING_DIRECTORY "${VE_NVBLAST_SRC}"
    RESULT_VARIABLE _ok
  )
  if(NOT _ok EQUAL 0)
    message(FATAL_ERROR "git sparse-checkout set blast failed")
  endif()
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" fetch --depth 1 origin "${VE_NVBLAST_GIT_SHA}"
    WORKING_DIRECTORY "${VE_NVBLAST_SRC}"
    RESULT_VARIABLE _ok
  )
  if(NOT _ok EQUAL 0)
    message(FATAL_ERROR "git fetch of ${VE_NVBLAST_GIT_SHA} failed")
  endif()
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" checkout --detach FETCH_HEAD
    WORKING_DIRECTORY "${VE_NVBLAST_SRC}"
    RESULT_VARIABLE _ok
  )
  if(NOT _ok EQUAL 0)
    message(FATAL_ERROR "git checkout of NvBlast SHA failed")
  endif()
  if(NOT EXISTS "${VE_BLAST_ROOT}/include/lowlevel/NvBlast.h")
    message(FATAL_ERROR "NvBlast headers missing after sparse checkout")
  endif()
endfunction()

ve_ensure_nvblast()

function(ve_apply_p1_extstress)
  set(_py "${CMAKE_SOURCE_DIR}/third_party/nvblast/patches/apply_p1_extstress.py")
  set(_h "${VE_BLAST_ROOT}/include/extensions/stress/NvBlastExtStressSolver.h")
  file(READ "${_h}" _hc)
  if(_hc MATCHES "VE_P1_COPYBOND_LINEAR")
    return()
  endif()
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(NOT Python3_Interpreter_FOUND)
    find_program(Python3_EXECUTABLE NAMES python python3 py)
  endif()
  if(NOT Python3_EXECUTABLE)
    message(FATAL_ERROR "Python is required to apply P1 NvBlast ExtStress adapters")
  endif()
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${_py}" "${VE_BLAST_ROOT}"
    RESULT_VARIABLE _ok
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err
  )
  if(NOT _ok EQUAL 0)
    execute_process(
      COMMAND py -3 "${_py}" "${VE_BLAST_ROOT}"
      RESULT_VARIABLE _ok
      OUTPUT_VARIABLE _out
      ERROR_VARIABLE _err
    )
  endif()
  if(NOT _ok EQUAL 0)
    message(FATAL_ERROR "P1 ExtStress patch failed: ${_out} ${_err}")
  endif()
  message(STATUS "Applied P1 ExtStress adapters")
endfunction()

ve_apply_p1_extstress()

function(ve_apply_p3_extstress)
  set(_py "${CMAKE_SOURCE_DIR}/third_party/nvblast/patches/apply_p3_extstress.py")
  set(_h "${VE_BLAST_ROOT}/include/extensions/stress/NvBlastExtStressSolver.h")
  file(READ "${_h}" _hc)
  if(_hc MATCHES "VE_P3_EXTSTRESS")
    return()
  endif()
  if(NOT Python3_EXECUTABLE)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(NOT Python3_Interpreter_FOUND)
      find_program(Python3_EXECUTABLE NAMES python python3 py)
    endif()
  endif()
  if(NOT Python3_EXECUTABLE)
    message(FATAL_ERROR "Python is required to apply P3 NvBlast ExtStress adapters")
  endif()
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${_py}" "${VE_BLAST_ROOT}"
    RESULT_VARIABLE _ok
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err
  )
  if(NOT _ok EQUAL 0)
    execute_process(
      COMMAND py -3 "${_py}" "${VE_BLAST_ROOT}"
      RESULT_VARIABLE _ok
      OUTPUT_VARIABLE _out
      ERROR_VARIABLE _err
    )
  endif()
  if(NOT _ok EQUAL 0)
    message(FATAL_ERROR "P3 ExtStress patch failed: ${_out} ${_err}")
  endif()
  message(STATUS "Applied P3 ExtStress addLoad")
endfunction()

ve_apply_p3_extstress()

set(VE_NVBLAST_LL_SOURCES
  "${VE_BLAST_ROOT}/source/sdk/common/NvBlastAssert.cpp"
  "${VE_BLAST_ROOT}/source/sdk/common/NvBlastAtomic.cpp"
  "${VE_BLAST_ROOT}/source/sdk/common/NvBlastTime.cpp"
  "${VE_BLAST_ROOT}/source/sdk/common/NvBlastTimers.cpp"
  "${VE_BLAST_ROOT}/source/sdk/lowlevel/NvBlastActor.cpp"
  "${VE_BLAST_ROOT}/source/sdk/lowlevel/NvBlastActorSerializationBlock.cpp"
  "${VE_BLAST_ROOT}/source/sdk/lowlevel/NvBlastAsset.cpp"
  "${VE_BLAST_ROOT}/source/sdk/lowlevel/NvBlastAssetHelper.cpp"
  "${VE_BLAST_ROOT}/source/sdk/lowlevel/NvBlastFamily.cpp"
  "${VE_BLAST_ROOT}/source/sdk/lowlevel/NvBlastFamilyGraph.cpp"
)

set(VE_NVBLAST_GLOBALS_SOURCES
  "${VE_BLAST_ROOT}/source/sdk/globals/NvBlastGlobals.cpp"
  "${VE_BLAST_ROOT}/source/sdk/globals/NvBlastInternalProfiler.cpp"
)

set(VE_NVBLAST_STRESS_SOURCES
  "${VE_BLAST_ROOT}/source/sdk/extensions/stress/NvBlastExtStressSolver.cpp"
  "${VE_BLAST_ROOT}/source/shared/stress_solver/stress.cpp"
)

add_library(nvblast STATIC
  ${VE_NVBLAST_LL_SOURCES}
  ${VE_NVBLAST_GLOBALS_SOURCES}
  ${VE_NVBLAST_STRESS_SOURCES}
)
add_library(engine::nvblast ALIAS nvblast)

target_include_directories(nvblast
  PUBLIC
    "${VE_BLAST_ROOT}/include/lowlevel"
    "${VE_BLAST_ROOT}/include/extensions/stress"
    "${VE_BLAST_ROOT}/include/globals"
    "${VE_BLAST_ROOT}/include/shared/NvFoundation"
  PRIVATE
    "${CMAKE_SOURCE_DIR}/cmake/nvblast_msvc_compat"
    "${VE_BLAST_ROOT}/include"
    "${VE_BLAST_ROOT}/source/sdk/common"
    "${VE_BLAST_ROOT}/source/sdk/lowlevel"
    "${VE_BLAST_ROOT}/source/sdk/globals"
    "${VE_BLAST_ROOT}/source/sdk/extensions/stress"
    "${VE_BLAST_ROOT}/source/shared/NsFoundation/include"
    "${VE_BLAST_ROOT}/source/shared/stress_solver"
)

target_compile_features(nvblast PUBLIC cxx_std_17)

target_compile_definitions(nvblast
  PUBLIC
    VE_NVBLAST_SHA="${VE_NVBLAST_GIT_SHA}"
    VE_NVBLAST_VERSION="${VE_NVBLAST_VERSION}"
  PRIVATE
    $<$<CONFIG:Debug>:NV_DEBUG=1>
    $<$<CONFIG:Debug>:NV_CHECKED=1>
    $<$<NOT:$<CONFIG:Debug>>:NV_DEBUG=0>
    $<$<NOT:$<CONFIG:Debug>>:NV_CHECKED=0>
)

if(MSVC)
  target_compile_options(nvblast PRIVATE
    /bigobj
    /wd4100
    /wd4127
    /wd4189
    /wd4244
    /wd4267
    /wd4324
    /wd4456
    /wd4505
    /wd4996
  )
  set_source_files_properties(
    "${VE_BLAST_ROOT}/source/sdk/extensions/stress/NvBlastExtStressSolver.cpp"
    PROPERTIES COMPILE_OPTIONS "/arch:AVX2;/FInvblast_cxx_math.h"
  )
  set_source_files_properties(
    "${VE_BLAST_ROOT}/source/shared/stress_solver/stress.cpp"
    PROPERTIES COMPILE_OPTIONS "/arch:AVX2"
  )
else()
  set_source_files_properties(${VE_NVBLAST_STRESS_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-mavx2;-mfma"
  )
endif()
