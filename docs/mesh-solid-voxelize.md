# Mesh Import → Surface Voxelize (GPU, with Color)

Living design note for turning a triangle mesh into an editable `VoxelObject` using **GPU surface occupancy only** (hollow shell) plus **sampled mesh color on those surface cells**.

Status: **implemented (v1)** — GPU compute surface voxelization to coarse cells (`brickPage = INVALID`), 256-color palette. **No interior fill.**

Related shipped code:

- `src/scene/VoxelScene.h` / `.cpp` — `VoxelObject`, `CoarseCell { material, brickPage }`, global brick pool, `rebuildVoxels` builds ground + spinner
- `shaders/voxel_dda.comp` — per-object DDA; `INVALID` brick = full cube; albedo currently unified `(0.62, 0.64, 0.68)`
- `src/gfx/GfxDevice` — compute pipelines, SSBO, `immediateSubmit` / staging already used for voxel uploads
- `docs/nested-brick-voxels.md` — coarse → sparse 8³ → 2×2×2 (import MVP does **not** allocate bricks)

---

## 1. Goal

Keep:

- Existing DDA, rigid `VoxelObject`, edit brushes, unified-albedo fallback
- Sparse bricks: occupied coarse cells stay `brickPage = INVALID` (cheap)
- **Voxelization is import-time only** — not in the per-frame loop

Add:

- Load a triangle mesh from disk (MVP: Wavefront OBJ) on **CPU**
- **GPU surface voxelization**: mark only cells that intersect a triangle (**hollow shell**)
- Carry **mesh color** onto those surface voxels (UV/albedo or material Kd)
- Download / pack into `CoarseCell`, then DDA as today
- Spawn / replace an imported object without rebuilding the whole scene from scratch

Non-goals (v1):

- **Interior / solid fill** (flood, parity, SDF) — later optional
- CPU triangle–AABB occupancy (replaced by GPU compute)
- CUDA / extra vendor APIs
- Hardware conservative raster **as a requirement** (optional later)
- glTF / FBX (convert offline if needed)
- Auto-repair of non-manifold / open meshes
- Per-micro / per-fine color (would require allocating bricks)
- Real-time per-frame re-voxelize

Open meshes are fine for v1: there is no flood that can leak.

---

## 2. Why surface-only, and why GPU compute

v1 is **A. surface voxelization**: a voxel is occupied iff its cube overlaps a triangle. The result is a **one-cell-thick (plus conservative fattening) shell**. Looking from outside it reads as the mesh; looking through a hole or carving shows **empty interior**.

Solid fill (border flood / parity) is explicitly **out of v1**. It can be a later pass that consumes the same `occ[]`.

**Occupancy runs on the GPU**, not the CPU:

| Choice | Why |
|--------|-----|
| **Compute shaders** (not graphics raster) | Voxel demo is already compute; no GS / conservative-raster extension; same SAT test as CPU, just parallel |
| **Import-time dispatch** | One `vkQueueSubmit` + fence; DDA frame loop unchanged |
| Raster + `VK_EXT_conservative_rasterization` | Optional later; not required for v1 |

CPU still: parse OBJ, build triangle SSBO, upload textures, wait for GPU, pack `VoxelObject`, palette quantize.

---

## 3. Pipeline overview

```
CPU: OBJ (+ MTL, map_Kd)
    → tinyobj triangulate
    → fit isotropic grid (N³, padding)
    → upload Triangle SSBO + albedo texture(s)

GPU (one import submit, not per-frame):
    1. voxelize_surface.comp — triangle–voxel SAT, atomicOr occ, write color
    2. (optional) voxelize_pack.comp — occ → CoarseCell

CPU:
    → readback occ + packed RGB
    → quantize palette, fill VoxelObject (only occ==1 cells)
    → packObjectPool / upload / DDA
```

Per-frame after that: **only DDA**. No triangle tests.

`sampleColor == false`: occupied cells get `material = 1`, unified/fallback albedo.

**Not in this pipeline:** `voxelize_flood`, interior color BFS.

---

## 4. GPU occupancy (surface only)

### 4.1 Grid fit (CPU)

1. Vertex AABB `bmin`, `bmax`.
2. `maxEdge = max(bmax - bmin)`.
3. `inner = N - 2 * padding` (`inner >= 1`).
4. `voxelSize = maxEdge / inner` (isotropic).
5. Expand `bmin` by `padding * voxelSize`.
6. Push constants / UBO: `bmin`, `invVoxelSize`, `N`.

World placement of the `VoxelObject` is a CPU knob (sit on ground vs hover).

### 4.2 GPU buffers (import scratch, destroyed after fence)

| Buffer / image | Contents |
|----------------|----------|
| `Triangle[]` | `p0,p1,p2`, `uv0,uv1,uv2`, vertex/material RGB, `texIndex` |
| `occ[]` | `N³` uint: `0` air / `1` surface |
| `seedRgb[]` | `N³` packed `0xAARRGGBB`; `A=0` means no color |
| `coarseOut[]` | optional `CoarseCell` staging |
| albedo sampler | `map_Kd`; 1×1 white if missing |

For `N <= 64`, one uint per cell is fine (`64³ × 4 ≈ 1 MB`).

### 4.3 `voxelize_surface.comp` (the only occupancy pass)

Dispatch **one workgroup per triangle** (or persistent threads + triangle id):

1. Load tri, compute voxel AABB of the three vertices, expand 1 cell if `conservative`.
2. Clamp range to `[0, N)`.
3. Loop voxels in that AABB (if a tri covers a huge volume, later tile-bin; v1 brute-force AABB).
4. For each voxel, **triangle–AABB SAT** (Akenine-Möller) in the shader:
   - Box X/Y/Z axes vs triangle
   - Triangle plane vs box
   - Nine edge × box-axis cross tests
5. On hit: `atomicMax(occ[idx], 1)`.
6. If coloring: barycentric at voxel center / closest point on triangle → UV → `textureLod` → pack RGB. **First-wins** on `seedRgb` (`atomicCompSwap` if alpha is 0).

Conservative **index AABB +1** replaces hardware conservative raster.

No second pass. `occ==0` stays air, including the entire interior of a closed mesh.

### 4.4 What the shell looks like

```
........##########........
........##......##........
........##......##........
........##########........
        ↑ hollow interior (air)
```

Carving a surface cell opens a hole to the inside. That is intended for v1.

### 4.5 Why not graphics raster in v1

Raster voxelization needs a graphics pipeline, dominant-axis swizzle, viewport `N×N`, conservative raster or dilated triangles, UAV image, possibly three axis passes.

The voxel demo has **no mesh raster path**. Compute SAT is one algorithm, one queue.

Later optional: `VK_EXT_conservative_rasterization` writing the same `occ[]`.

---

## 5. Color (surface cells only)

### 5.1 Sources (in the surface shader)

1. UV + albedo texture × Kd / baseColorFactor  
2. Else interpolated vertex color  
3. Else constant Kd  
4. Else unified gray `(0.62, 0.64, 0.68)`

CPU loads images with `stb_image`. Missing tex → 1×1 white.

### 5.2 No interior propagate

Only cells with `occ==1` have color. Interior air has none. Palette quantize runs over occupied cells only.

### 5.3 Palette (CPU after readback)

- `material = 0` air  
- `material = 1..255` palette index for surface cells  
- Ground / spinner: no import-palette flag  

DDA:

```
if (object uses import palette)
    albedo = palette[matId];
else
    albedo = vec3(0.62, 0.64, 0.68);
```

**Not v1:** extra RGB on `CoarseCell`.

### 5.4 Resolution

Color = coarse `N³` blocks on the shell, not texture size. Finer color = later surface bricks.

---

## 6. Runtime cost

| Phase | Thread | When |
|-------|--------|------|
| OBJ parse, upload tris/tex | CPU | Import click |
| Surface SAT + color seed | **GPU compute** | Import click (one submit, wait fence) |
| Readback, palette, `VoxelObject` | CPU | Import click |
| DDA of imported object | GPU | **Every frame**, same as spinner |

Import work **does not** stay in `VoxelRenderer::draw`. Extra per-frame cost is **one more DDA object** (a sparse shell, usually cheaper than a solid fill of the same AABB).

---

## 7. Engine integration

### 7.1 New files

| Path | Role |
|------|------|
| `src/voxel/MeshVoxelizer.h/.cpp` | CPU load, GPU submit, readback, palette, `VoxelObject` fill |
| `shaders/voxelize_surface.comp` | SAT occupancy + color |
| `shaders/voxelize_pack.comp` | optional occ → `CoarseCell` |
| `third_party/tiny_obj_loader.h` | Parse |
| `assets/meshes/cube.obj` | Golden |
| `CMakeLists.txt` | `VE_VOXEL_SOURCES` + `ve_shaders` |

Lazy `VoxelizeGpu` helper on first import; destroy in `VoxelScene::cleanup`. No per-frame pipeline rebuild.

Sync: `immediateSubmit` or fence; wait before readback.

### 7.2 API

```cpp
struct MeshVoxelizeConfig {
  int gridN = 48;            // 8..64
  int padding = 1;
  uint32_t fallbackMaterial = 1;
  bool conservative = true;  // expand tri voxel AABB by 1
  bool sampleColor = true;
};

struct MeshVoxelizeResult { /* ok, n, voxelSize, bmin, material[], palette, error */ };

MeshVoxelizeResult voxelizeObjSurface(GfxDevice& gfx, const std::string& path,
                                      const MeshVoxelizeConfig& cfg);
```

`VoxelScene`:

```cpp
bool importSurfaceMesh(GfxDevice& gfx, const std::string& path, const MeshVoxelizeConfig& cfg);
void removeImportedMesh(GfxDevice& gfx);
```

Imported slot: `objects_[2]`, replace on re-import. `rebuildVoxels` re-runs last path.

Write rule: `occ==1` → `material = palette index or 1`, `brickPage = INVALID`; else air.

### 7.3 Object count (P0)

DDA must loop `objectCount`, not `2`.

### 7.4 Placement / UI

Identity rotation, sit-on-ground default. ImGui: path, N, padding, **Import Surface OBJ**, Remove, status.

---

## 8. Shader / DDA (per-frame)

- Flag `kFlagImportPalette`  
- `vec4 importPalette[256]`  
- Debug modes unchanged  
- Voxelize shaders **never** bound in `draw()`

---

## 9. Editing after import

Coarse brushes work on the shell. Micro/fine on `INVALID` bricks allocate occupancy-only pages without extra texture detail. Documented until a later brick-color pass.

Carving through the shell exposes empty interior — expected.

---

## 10. Implementation slices

### P0 — Object-count compatibility

`objects_.size() >= 3`. Demo unchanged.

### P1 — GPU surface occupancy

Triangle SSBO + `voxelize_surface.comp`. Occupied count for `cube.obj` looks like a shell (not a filled cube).

### P2 — Pack into the scene

`importSurfaceMesh`, third object visible, hollow when carved / viewed from a hole.

### P3 — ImGui

Path, N, padding, errors.

### P4 — Color + palette

Surface sample, CPU quantize, DDA palette.

### P5 (later, not v1)

- **Solid fill** (exterior flood) as an optional checkbox  
- Surface 8³ bricks + micro colors  
- glTF; conservative raster; async import  

---

## 11. Tests / goldens

| Case | Expect |
|------|--------|
| Closed cube OBJ | Hollow box (12 walls of coarse cells); interior air |
| Sphere OBJ | Hollow shell |
| Colored / textured OBJ | Blocky colors on the shell |
| Open plane | A voxelized sheet; no “leak” warning needed |
| Import off | Ground + spinner only |
| Grid Size slider | Reimport last path |
| Frame loop | Voxelize compute **not** dispatched |

---

## 12. Risks

| Risk | Mitigation |
|------|------------|
| Huge triangle AABB | Clamp loops; later tile binning |
| Thin walls missed | `conservative` AABB +1 + SAT |
| Shell thicker than 1 cell | Expected with conservative expand |
| Race on seed color | First-wins atomics |
| Readback stall | Import only |
| Hardcoded 2 objects | P0 |

---

## 13. Defaults

| Knob | Default |
|------|---------|
| `gridN` | 48 |
| `padding` | 1 |
| `conservative` | true |
| `sampleColor` | true |
| Palette | 256 |
| Fill interior | **off (not implemented)** |
| Max imported objects | 1 (replace) |

---

## 14. Open points

1. Placement: snap AABB bottom to ground vs hover.  
2. Palette per object vs global.  
3. Brush vs palette indices.  
4. How fat the conservative shell should be (AABB +1 vs SAT-only).  
5. Linear vs sRGB texel fetch.  
6. Keep scratch GPU buffers between imports.  
7. When to add optional solid fill (P5) vs never.

---

## 15. References

- Schwarz & Seidel, *Fast Parallel Surface and Solid Voxelization on GPUs* (2010) — surface vs solid; v1 is surface only  
- Akenine-Möller, *Fast 3D Triangle-Box Overlap Testing*  
- NVIDIA, [Basics of GPU Voxelization](https://developer.nvidia.com/content/basics-gpu-voxelization)  
- [tinyobjloader](https://github.com/tinyobjloader/tinyobjloader)  
- In-engine: `docs/nested-brick-voxels.md`
