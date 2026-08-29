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

set(VE_SOURCES
  src/main.cpp
  src/core/Window.cpp
  src/core/Camera.cpp
  src/gfx/GfxDevice.cpp
  src/gfx/PipelineBuilder.cpp
  src/gfx/Mesh.cpp
  src/gfx/Texture.cpp
  src/gfx/ShIrradiance.cpp
  src/scene/Scene.cpp
  src/scene/GrassSystem.cpp
  src/render/Renderer.cpp
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)

add_executable(vulkan_engine ${VE_SOURCES})

# Windows: use GUI subsystem so closing a console window does not kill the app.
if(MSVC)
  target_link_options(vulkan_engine PRIVATE "/SUBSYSTEM:WINDOWS" "/ENTRY:mainCRTStartup")
endif()

target_include_directories(vulkan_engine PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
  ${vma_SOURCE_DIR}/include
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(vulkan_engine PRIVATE
  Vulkan::Vulkan
  glfw
  glm::glm
  vk-bootstrap::vk-bootstrap
)

target_compile_definitions(vulkan_engine PRIVATE
  GLFW_INCLUDE_VULKAN
  GLM_FORCE_RADIANS
  GLM_FORCE_DEPTH_ZERO_TO_ONE
  IMGUI_IMPL_VULKAN_HAS_DYNAMIC_RENDERING
  $<$<AND:$<BOOL:${VE_ENABLE_VALIDATION}>,$<CONFIG:Debug>>:VE_ENABLE_VALIDATION=1>
)

if(APPLE)
  target_compile_definitions(vulkan_engine PRIVATE VE_PLATFORM_MACOS=1)
  target_compile_options(vulkan_engine PRIVATE -Wno-nullability-completeness)
endif()

# ---- Shader compilation ----
find_program(GLSLC glslc HINTS /opt/homebrew/bin ENV VULKAN_SDK PATH_SUFFIXES bin REQUIRED)
set(VE_SHADER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(VE_SHADER_OUTPUT_DIR ${CMAKE_BINARY_DIR}/shaders)
file(MAKE_DIRECTORY ${VE_SHADER_OUTPUT_DIR})

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

target_compile_definitions(vulkan_engine PRIVATE
  VE_SHADER_DIR="${VE_SHADER_OUTPUT_DIR}"
  VE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets"
)

if(CMAKE_EXPORT_COMPILE_COMMANDS AND EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")
  add_custom_command(TARGET vulkan_engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_BINARY_DIR}/compile_commands.json
      ${CMAKE_SOURCE_DIR}/compile_commands.json
    COMMENT "Updating compile_commands.json"
  )
endif()
