# Vulkan Engine

Modern Vulkan 1.3 forward renderer on macOS (MoltenVK) / desktop.

## Features (Phases 1–5)

- Vulkan 1.3 + Dynamic Rendering + Synchronization 2
- VMA memory allocation, vk-bootstrap device setup
- Procedural meshes (cube / plane / sphere) + textures
- Reverse-Z camera, orbit controls
- Cook-Torrance PBR (directional light)
- Shadow map + 3×3 PCF
- HDR color target + ACES tonemap
- Dear ImGui debug panel
- Frequency-based descriptor sets (frame / material)

## Prerequisites (macOS)

```bash
brew install cmake glfw glm vulkan-headers vulkan-loader molten-vk vulkan-validationlayers shaderc pkg-config
```

## Build

```bash
cd /Users/Shared/vulkan-engine
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=/opt/homebrew
cmake --build build -j
```

## Run

```bash
./scripts/run.sh
```

## Controls

| Input | Action |
|-------|--------|
| WASD | Orbit camera |
| Q / E | Zoom |
| Right-drag | Orbit |
| Esc | Quit |

## Layout

```
src/core/     Window, Camera
src/gfx/      GfxDevice, VMA helpers, Mesh, Texture, PipelineBuilder
src/scene/    Scene + materials + objects
src/render/   Renderer (shadow → HDR PBR → tonemap → ImGui)
shaders/      GLSL → SPIR-V (built by CMake)
```
