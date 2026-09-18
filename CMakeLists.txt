cmake_minimum_required(VERSION 3.20)
project(vulkan_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(VE_ENABLE_VALIDATION "Enable Vulkan validation layers in Debug builds" ON)

include(FetchContent)

FetchContent_Declare(
  vk_bootstrap
  GIT_REPOSITORY https://github.com/charles-lunarg/vk-bootstrap.git
  GIT_TAG        v1.3.302
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(vk_bootstrap)

FetchContent_Declare(
  vma
  GIT_REPOSITORY https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator.git
  GIT_TAG        v3.2.1
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(vma)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.91.8
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

find_package(Vulkan REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(glm REQUIRED)

# Shared engine modules linked into both demos.
set(VE_COMMON_SOURCES
  src/core/Window.cpp
  src/core/Camera.cpp
  src/gfx/GfxDevice.cpp
  src/gfx/PipelineBuilder.cpp
  src/gfx/Mesh.cpp
  src/gfx/Texture.cpp
  src/gfx/ShIrradiance.cpp
  src/gfx/IblBake.cpp
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)

set(VE_GRASS_SOURCES
  src/main.cpp
  src/scene/Scene.cpp
  src/scene/GrassSystem.cpp
  src/render/Renderer.cpp
)

set(VE_VOXEL_SOURCES
  src/main_voxel.cpp
  src/scene/VoxelScene.cpp
  src/scene/VoxelSceneFracture.cpp
  src/render/VoxelRenderer.cpp
  src/voxel/MeshVoxelizer.cpp
  src/voxel/VoxelConnectivity.cpp
  src/physics/Classify.cpp
  src/physics/VoxelCollide.cpp
  src/physics/Solver.cpp
  src/physics/PhysicsWorld.cpp
)

function(ve_configure_target target_name)
  target_include_directories(${target_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    ${vma_SOURCE_DIR}/include
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
  )

  target_link_libraries(${target_name} PRIVATE
    Vulkan::Vulkan
    glfw
    glm::glm
    vk-bootstrap::vk-bootstrap
  )

  target_compile_definitions(${target_name} PRIVATE
    GLFW_INCLUDE_VULKAN
    GLM_FORCE_RADIANS
    GLM_FORCE_DEPTH_ZERO_TO_ONE
    IMGUI_IMPL_VULKAN_HAS_DYNAMIC_RENDERING
    VE_SHADER_DIR="${VE_SHADER_OUTPUT_DIR}"
    VE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets"
    $<$<AND:$<BOOL:${VE_ENABLE_VALIDATION}>,$<CONFIG:Debug>>:VE_ENABLE_VALIDATION=1>
  )

  if(MSVC)
    target_link_options(${target_name} PRIVATE "/SUBSYSTEM:WINDOWS" "/ENTRY:mainCRTStartup")
  endif()

  if(APPLE)
    target_compile_definitions(${target_name} PRIVATE VE_PLATFORM_MACOS=1)
    target_compile_options(${target_name} PRIVATE -Wno-nullability-completeness)
    # Needed to force CAMetalLayer.displaySyncEnabled = NO under MoltenVK.
    target_link_libraries(${target_name} PRIVATE "-framework Cocoa" "-framework QuartzCore")
  endif()
endfunction()

# Objective-C++ helpers live in Window.cpp on macOS.
if(APPLE)
  set_source_files_properties(src/core/Window.cpp PROPERTIES COMPILE_FLAGS "-x objective-c++")
endif()

# Shader output dir must exist before target compile defs reference it.
set(VE_SHADER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(VE_SHADER_OUTPUT_DIR ${CMAKE_BINARY_DIR}/shaders)
file(MAKE_DIRECTORY ${VE_SHADER_OUTPUT_DIR})

add_executable(vulkan_engine ${VE_COMMON_SOURCES} ${VE_GRASS_SOURCES})
ve_configure_target(vulkan_engine)

add_executable(vulkan_engine_voxel ${VE_COMMON_SOURCES} ${VE_VOXEL_SOURCES})
ve_configure_target(vulkan_engine_voxel)

add_executable(voxel_connectivity_tests
  tests/voxel_connectivity_tests.cpp
  src/voxel/VoxelConnectivity.cpp
)
target_include_directories(voxel_connectivity_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_features(voxel_connectivity_tests PRIVATE cxx_std_20)

include(cmake/NvBlast.cmake)

add_library(ve_structure_core STATIC
  src/blast/BlastMemory.cpp
  src/blast/VoxelGraph.cpp
  src/blast/ContactLoads.cpp
  src/blast/ImpactDamage.cpp
  src/blast/OccupancySampler.cpp
  src/blast/StructureWorld.cpp
)
target_include_directories(ve_structure_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(ve_structure_core PUBLIC nvblast glm::glm)
target_compile_features(ve_structure_core PUBLIC cxx_std_20)
target_compile_definitions(ve_structure_core PUBLIC
  GLM_FORCE_RADIANS
  GLM_FORCE_DEPTH_ZERO_TO_ONE
)
if(MSVC)
  target_compile_definitions(ve_structure_core PUBLIC NOMINMAX)
endif()

target_link_libraries(vulkan_engine_voxel PRIVATE ve_structure_core)

function(ve_blast_test target_name)
  add_executable(${target_name} ${ARGN})
  target_link_libraries(${target_name} PRIVATE ve_structure_core)
  target_compile_features(${target_name} PRIVATE cxx_std_20)
endfunction()

ve_blast_test(blast_p0_tests tests/blast_p0_tests.cpp)
ve_blast_test(blast_p1_tests
  tests/blast_p1_tests.cpp
  src/blast/CylinderGraph.cpp
)
ve_blast_test(blast_p2_tests
  tests/blast_p2_tests.cpp
  src/blast/CylinderGraph.cpp
  src/blast/ActorBodies.cpp
)
ve_blast_test(blast_p3_tests tests/blast_p3_tests.cpp)
ve_blast_test(blast_p4_tests tests/blast_p4_tests.cpp)
ve_blast_test(blast_graph_regression_tests tests/blast_graph_regression_tests.cpp)
ve_blast_test(blast_e0_tests tests/blast_e0_tests.cpp)
ve_blast_test(blast_e1_tests tests/blast_e1_tests.cpp)
ve_blast_test(blast_frame_tests tests/blast_frame_tests.cpp)
ve_blast_test(blast_viewer_alignment_tests tests/blast_viewer_alignment_tests.cpp)
ve_blast_test(blast_e2_tests tests/blast_e2_tests.cpp)
ve_blast_test(blast_e2_perf tests/blast_e2_perf.cpp)
ve_blast_test(blast_probe_export_tests tests/blast_probe_export_tests.cpp)
ve_blast_test(blast_e3_tests
  tests/blast_e3_tests.cpp
  src/physics/Solver.cpp
)

# ---- Shader compilation ----
find_program(GLSLC glslc HINTS /opt/homebrew/bin ENV VULKAN_SDK PATH_SUFFIXES bin REQUIRED)

set(VE_SHADER_SOURCES
  mesh.vert
  mesh.frag
  shadow.vert
  shadow.frag
  fullscreen.vert
  tonemap.frag
  grass.vert
  grass.frag
  grass_shadow.vert
  grass_shadow.frag
  sky.frag
  ibl_equirect_to_cube.frag
  ibl_prefilter.frag
  ibl_brdf.frag
  ibl_cube.vert
  voxel.vert
  voxel.frag
  voxel_dda.comp
  voxel_dda_coarse.comp
  voxelize_surface.comp
  proxy_box.vert
  visibility.frag
  voxel_shade.frag
)

set(VE_SPV_FILES "")
foreach(SHADER ${VE_SHADER_SOURCES})
  set(SRC ${VE_SHADER_SOURCE_DIR}/${SHADER})
  set(SPV ${VE_SHADER_OUTPUT_DIR}/${SHADER}.spv)
  if(SHADER STREQUAL "voxel_dda.comp")
    # Debug symbols in this kernel bloat VGPR allocation; keep -O only.
    add_custom_command(
      OUTPUT ${SPV}
      COMMAND ${GLSLC} -O -I ${VE_SHADER_SOURCE_DIR} ${SRC} -o ${SPV}
      DEPENDS ${SRC} ${VE_SHADER_SOURCE_DIR}/dir_mask_lut.glsl ${VE_SHADER_SOURCE_DIR}/voxel_beam.glsl ${VE_SHADER_SOURCE_DIR}/voxel_ao.glsl
      COMMENT "Compiling shader ${SHADER}"
      VERBATIM
    )
  elseif(SHADER STREQUAL "voxel_dda_coarse.comp")
    add_custom_command(
      OUTPUT ${SPV}
      COMMAND ${GLSLC} -O -g -I ${VE_SHADER_SOURCE_DIR} ${SRC} -o ${SPV}
      DEPENDS ${SRC} ${VE_SHADER_SOURCE_DIR}/dir_mask_lut.glsl
      COMMENT "Compiling shader ${SHADER}"
      VERBATIM
    )
  elseif(SHADER STREQUAL "proxy_box.vert" OR SHADER STREQUAL "visibility.frag" OR SHADER STREQUAL "voxel_shade.frag")
    add_custom_command(
      OUTPUT ${SPV}
      COMMAND ${GLSLC} -O -g -I ${VE_SHADER_SOURCE_DIR} ${SRC} -o ${SPV}
      DEPENDS ${SRC}
        ${VE_SHADER_SOURCE_DIR}/voxel_types.glsl
        ${VE_SHADER_SOURCE_DIR}/voxel_trace.glsl
        ${VE_SHADER_SOURCE_DIR}/dir_mask_lut.glsl
        ${VE_SHADER_SOURCE_DIR}/voxel_ao.glsl
      COMMENT "Compiling shader ${SHADER}"
      VERBATIM
    )
  else()
    add_custom_command(
      OUTPUT ${SPV}
      COMMAND ${GLSLC} -O -g ${SRC} -o ${SPV}
      DEPENDS ${SRC}
      COMMENT "Compiling shader ${SHADER}"
      VERBATIM
    )
  endif()
  list(APPEND VE_SPV_FILES ${SPV})
endforeach()

set(VE_INDEXED_SPV ${VE_SHADER_OUTPUT_DIR}/voxel_dda_indexed.comp.spv)
add_custom_command(
  OUTPUT ${VE_INDEXED_SPV}
  COMMAND ${GLSLC} -O -DVE_NONUNIFORM_BRICKS=1 --target-env=vulkan1.2 -I ${VE_SHADER_SOURCE_DIR} ${VE_SHADER_SOURCE_DIR}/voxel_dda.comp -o ${VE_INDEXED_SPV}
  DEPENDS ${VE_SHADER_SOURCE_DIR}/voxel_dda.comp ${VE_SHADER_SOURCE_DIR}/dir_mask_lut.glsl ${VE_SHADER_SOURCE_DIR}/voxel_beam.glsl ${VE_SHADER_SOURCE_DIR}/voxel_ao.glsl
  COMMENT "Compiling shader voxel_dda_indexed.comp"
  VERBATIM
)
list(APPEND VE_SPV_FILES ${VE_INDEXED_SPV})

set(VE_SLIM_SPV ${VE_SHADER_OUTPUT_DIR}/voxel_dda_slim.comp.spv)
add_custom_command(
  OUTPUT ${VE_SLIM_SPV}
  COMMAND ${GLSLC} -O -DENABLE_SHADE=1 -I ${VE_SHADER_SOURCE_DIR} ${VE_SHADER_SOURCE_DIR}/voxel_dda_coarse.comp -o ${VE_SLIM_SPV}
  DEPENDS ${VE_SHADER_SOURCE_DIR}/voxel_dda_coarse.comp ${VE_SHADER_SOURCE_DIR}/dir_mask_lut.glsl
  COMMENT "Compiling shader voxel_dda_slim.comp"
  VERBATIM
)
list(APPEND VE_SPV_FILES ${VE_SLIM_SPV})

add_custom_target(ve_shaders ALL DEPENDS ${VE_SPV_FILES})
add_dependencies(vulkan_engine ve_shaders)
add_dependencies(vulkan_engine_voxel ve_shaders)

if(CMAKE_EXPORT_COMPILE_COMMANDS AND EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")
  add_custom_command(TARGET vulkan_engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_BINARY_DIR}/compile_commands.json
      ${CMAKE_SOURCE_DIR}/compile_commands.json
    COMMENT "Updating compile_commands.json"
  )
endif()
