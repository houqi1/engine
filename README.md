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
- Instanced procedural grass (Phase 1: CPU instances + VS wind + shadow)

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
| WASD | Move camera |
| Q / E | Down / Up |
| Right-drag | Look |
| Esc | Quit |

ImGui panel: grass density, wind, colors, enable/disable.

## Demos (two executables, shared engine code)

| Target | Entry | Description |
|--------|-------|-------------|
| `vulkan_engine` | `src/main.cpp` | Grass / PBR demo |
| `vulkan_engine_voxel` | `src/main_voxel.cpp` | Voxel sandbox (dense-grid Amanatides–Woo DDA) |

Shared modules: `src/core`, `src/gfx`, ImGui backends.  
Grass-only: `Scene`, `GrassSystem`, `Renderer`.  
Voxel-only: `VoxelScene`, `VoxelRenderer`.

Windows runners: `run_engine.bat`, `run_engine_voxel.bat`.

The voxel Windows runners use Release. Reproducible 1440p GPU benchmarks,
quality checks, optimization findings, and the remaining sustained-performance
limits are documented in [Voxel Performance](docs/voxel-performance.md).

## Layout

```
src/core/     Window, Camera
src/gfx/      GfxDevice, VMA helpers, Mesh, Texture, PipelineBuilder
src/scene/    Grass Scene + VoxelScene
src/render/   Renderer (grass) + VoxelRenderer
shaders/      GLSL → SPIR-V (built by CMake)
```
