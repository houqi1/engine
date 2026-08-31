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
  src/render/VoxelRenderer.cpp
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
  endif()
endfunction()

# Shader output dir must exist before target compile defs reference it.
set(VE_SHADER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(VE_SHADER_OUTPUT_DIR ${CMAKE_BINARY_DIR}/shaders)
file(MAKE_DIRECTORY ${VE_SHADER_OUTPUT_DIR})

add_executable(vulkan_engine ${VE_COMMON_SOURCES} ${VE_GRASS_SOURCES})
ve_configure_target(vulkan_engine)

add_executable(vulkan_engine_voxel ${VE_COMMON_SOURCES} ${VE_VOXEL_SOURCES})
ve_configure_target(vulkan_engine_voxel)

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
)

set(VE_SPV_FILES "")
foreach(SHADER ${VE_SHADER_SOURCES})
  set(SRC ${VE_SHADER_SOURCE_DIR}/${SHADER})
  set(SPV ${VE_SHADER_OUTPUT_DIR}/${SHADER}.spv)
  add_custom_command(
    OUTPUT ${SPV}
    COMMAND ${GLSLC} -O -g ${SRC} -o ${SPV}
    DEPENDS ${SRC}
    COMMENT "Compiling shader ${SHADER}"
    VERBATIM
  )
  list(APPEND VE_SPV_FILES ${SPV})
endforeach()

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
