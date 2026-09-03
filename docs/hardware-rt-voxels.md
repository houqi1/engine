# Hardware Ray Tracing for Voxels (AABB BLAS + DDA, with Software Fallback)

Living design note. **Not triangle extraction. Not RTX-as-magic. Not Mac-required.**

Status: **design only** — current shipped path is occupancy hull + compute Amanatides–Woo DDA. Hardware RT is an **optional desktop backend** that replaces empty-space skipping, not shading or voxel storage.

Related shipped code:

- `src/gfx/GfxDevice.cpp` — Vulkan 1.3 + dynamic rendering + sync2; **no** AS / ray-query extensions
- `src/render/VoxelRenderer.cpp` — hull raster (`tMin/tMax/tBack`) then `vkCmdDispatch` of `voxel_dda.comp`
- `shaders/voxel_dda.comp` — object loop + coarse DDA + nested `8³` / `2³`; MoltenVK-safe unrolled `grids[]` / brick slabs
- `src/scene/VoxelScene.cpp` — rigid `VoxelObject`s, sparse brick pool, `fillOccMip`, `rebuildOccupancyHull`, dirty-page flush
- `docs/nested-brick-voxels.md` — brick pages; HW RT BLAS listed as out of scope there
- `docs/object-order-ess.md` — current software empty-space skip (occupancy hull)

Related work (external): Tuxedo Labs next engine — sparse `8³` chunks, chunk AABBs in BLAS, intersection-shader DDA.

---

## 1. Goal

Keep:

- Nested sparse bricks (`coarse → 8³ → 2×2×2`), object-local grids, rigid `objectToWorld`
- Compute as the voxel kernel (albedo / normal / AO / sky / render modes)
- CPU pick / brush / dirty-page flush
- **macOS (MoltenVK) as a first-class target** — the demo must build and run there without ray-tracing extensions

Change (desktop, capability-gated):

- Optional hardware BVH to skip empty space **between** objects and unoccupied macros/bricks
- Replace the per-pixel object loop (and, when RT is on, the hull draw) with one TLAS `rayQuery`
- Inside a hit AABB, reuse the existing DDA (`traceMicro` / fine bits)

Why: hardware RT cores traverse a BVH of boxes; they do not walk `8³` occupancy bits. The win is skipping air and scaling to many objects / secondary rays (shadows). Primary visibility at current scene size (world `48³` + spinner) may only tie the hull path.

---

## 2. Non-goals

| Out | Why |
|-----|-----|
| Marching cubes / triangle BLAS of voxels | Fights editable nested bricks; rebuild on every stroke |
| One AABB per micro/fine voxel | BLAS explode; edit becomes unusable |
| Putting the existing hull **triangles** in a BLAS | Reimplements today’s raster hull; interior empty still DDA |
| `VK_KHR_ray_tracing_pipeline` + SBT as v1 | Far from current compute `imageStore`; Ray Query is enough |
| Metal RT on Apple GPUs | Different API; this engine is Vulkan/MoltenVK |
| Requiring buffer device address / AS to **create the device** | Would fail MoltenVK / iGPU device selection |
| Hardware rays for vertex AO | Current `neighborSolid` is voxel topology, not hemisphere GI |
| Dropping the hull path once RT ships | Hull **is** the Mac renderer |

---

## 3. Current frame (software path — also the Mac path)

```
CPU:  edit → dirty brick pages → fillOccMip → rebuildOccupancyHull (4³ macros as boxes)
GPU:  raster hull → tMin / tMax / tBack
      compute DDA: for each VoxelObject, transform ray, coarse → 8³ → 2³
      blit + ImGui
```

This is already software raycasting plus a cheap object-order skip. Hardware RT replaces **hull draw + object loop + coarse empty walks**, not materials, brick SSBO, or AO.

| Today | Hardware RT analogue |
|-------|----------------------|
| `rebuildOccupancyHull` boxes | BLAS AABB primitives |
| `objectToWorld` / spinner spin | TLAS instance transform |
| `tMinMap` raster | closest `rayQuery` hit |
| `traverseObject` coarse DDA | intersection inside the hit AABB |
| `traceMicro` / `getFineVoxel` | **unchanged** |
| `neighborSolid` AO | still reads `grids[]` + brick pool |

`GfxDevice` prefers an NVIDIA RTX **name** when scoring GPUs. That is device selection only. No RT extension is enabled.

---

## 4. What “hardware RT” means here

Not: extract a triangle mesh and `vkCmdTraceRays`.

Yes: **AABB custom primitives in a BLAS**, instances in a **TLAS**, then **the same DDA** inside the box that the RT core reported.

```
Ray
 └─ TLAS (world: which VoxelObject, what transform)
       ├─ instance world    → BLAS_world    (AABBs of occupied macros / bricks)
       └─ instance spinner  → BLAS_spinner
              └─ hit AABB → existing nested DDA in that box
```

### BLAS / TLAS

- **BLAS (bottom-level):** BVH over one object’s AABBs, in object-local (or voxel-scaled) space. Expensive to rebuild if the AABB *set* changes.
- **TLAS (top-level):** BVH over instances `{BLAS, objectToWorld, mask, customIndex}`. Moving/rotating a rigid object updates the instance, not the geometry tree.

Same BLAS can be instanced many times. Spinner rotation = TLAS-only. Digging a brick interior (AABB still present) = no BLAS rebuild.

Vulkan APIs:

| | Ray Query (compute) | Ray Tracing Pipeline |
|--|---------------------|----------------------|
| Distance from this repo | Near: keep `imageStore` | Far: SBT, `.rgen/.rint/.rchit` |
| v1 choice | **This** | Later, if any-hit / multi-bounce |

Required (all **optional** at device create):

- `VK_KHR_acceleration_structure`
- `VK_KHR_deferred_host_operations`
- `VK_KHR_ray_query`
- `bufferDeviceAddress` feature

Do **not** enable `VK_KHR_ray_tracing_pipeline` for v1.

---

## 5. Geometry policy (the only real design fork)

Boxes too coarse → RT barely skips. Boxes too fine → BLAS rebuild kills editing.

### Do not

- Per-fine/micro AABB
- Marching cubes
- Triangle hull in BLAS as the RT strategy

### Do: `VK_GEOMETRY_TYPE_AABBS_KHR`

**Phase RT-1 (ship first):** one AABB per **occupied occupancy-mip `4³` macro** — the same cells `fillOccMip` / `rebuildOccupancyHull` already emit.

- World `N=48` → `12³ = 1728` possible macros; occupied count is hundreds
- Hit a macro box, then DDA coarse → micro → fine **inside that box**
- Benefit: drop object loop + hull draw; BVH skips empty macros
- Interior of a macro may still empty-walk, same class as today’s post-hull DDA

**Phase RT-2:** finer leaves

- Allocated brick → one coarse AABB (payload includes `brickPage`)
- Solid coarse with `brickPage == INVALID` (virtual full `8³`, e.g. ground) → **greedy-merge** into a few large AABBs, never one box per cell

A 48×1×48 solid slab must become **one plate AABB**, with DDA inside. Tens of thousands of adjacent solid AABBs lose to current coarse DDA.

### Coordinates

Prefer world-unit AABBs matching hull `emitBox`:

```
min = cell * voxelSize
max = (cell + extent) * voxelSize
```

TLAS instance transform = existing `GpuVoxelObject.objectToWorld`. Avoid a second unit system.

### Sidecar (AS does not store brickPage)

```cpp
struct AabbMeta {
  uint32_t objectIndex;
  uint16_t cx, cy, cz;   // macro or coarse origin
  uint16_t extent;       // 1 = coarse, 4 = mip, or merged size
  uint32_t brickPage;    // optional; shader may also texelFetch grids[]
};
```

GPU AABB buffer: tightly packed `VkAabbPositionsKHR` (16-byte aligned).  
`AabbMeta` SSBO indexed by `rayQueryGetIntersectionPrimitiveIndexEXT` plus instance `customIndex` (meta base or `objectIndex`).

---

## 6. Fallback is first-class (macOS / MoltenVK)

MoltenVK does **not** expose `VK_KHR_ray_query` / `VK_KHR_acceleration_structure`. Apple Metal RT on later M-series is a different API and is **out of scope**.

The software path (hull + DDA) **is** the Mac renderer, not a temporary stub.

### Rules

1. **Capability probe, not only `#ifdef APPLE`.** Windows iGPU and older AMD need the same fallback. `VE_PLATFORM_MACOS` may omit RT translation units; it is not the only gate.
2. **Shared voxel truth, two hit backends.** Bricks, 3D grids, transforms, AO, edits, CPU pick are common. Only “first hit along the camera ray” forks.
3. **Zero cost when RT is absent.** No AS allocations, no BDA-required device, no ray-query shader as the only SPIR-V.
4. **Do not delete `rebuildOccupancyHull`.** Mac and “RT off” both need the hull mesh. Hardware path may skip the **draw**, not the occupancy mip.

### Runtime fork

```
selectDevice():
  required: Vulkan 1.3 + dynamicRendering + sync2     // enough to boot on Mac
  optional: accelerationStructure + rayQuery + BDA

useHwRt = gfx.rayQuerySupported() && userWantsRt      // Mac ⇒ always false

if (useHwRt)  dispatch voxel_dda_rq.comp, skip hull draw
else          recordHullPass + dispatch voxel_dda.comp
```

Do not toggle per frame unless the user flips an ImGui switch **and** the device actually supports RT. On Mac, show `Hardware RT: unavailable (MoltenVK)` — not a dead checkbox.

### Device create must not hard-require RT

| Feature | Required to boot | Optional for RT |
|---------|------------------|-----------------|
| Vulkan 1.3, dynamic rendering, sync2 | yes | |
| `bufferDeviceAddress` | **no** | yes |
| `accelerationStructure` / `rayQuery` | **no** | yes |
| VMA `BUFFER_DEVICE_ADDRESS` bit | only if `rayQuerySupported` | |

Making BDA required can make MoltenVK fail `selectDevice` and kill the voxel demo on Mac.

### Compile split

`GL_EXT_ray_query` must not live in the **only** copy of `voxel_dda.comp`.

Same source, two SPIR-V:

```
glslc shaders/voxel_dda.comp
    → voxel_dda.comp.spv              # Mac / fallback, no RT extensions

glslc -DVE_RAY_QUERY=1 shaders/voxel_dda.comp
    → voxel_dda_rq.comp.spv           # load only if probe succeeded
```

```glsl
#ifdef VE_RAY_QUERY
#extension GL_EXT_ray_query : require
layout(set = 0, binding = 11) uniform accelerationStructureEXT tlas;
#endif

ObjectHit findHit(vec3 origin, vec3 dir) {
#ifdef VE_RAY_QUERY
    return findHitRayQuery(origin, dir);
#else
    return findHitDda(origin, dir);   // tMin + object loop
#endif
}
```

`traceMicro`, AO, sky, and MoltenVK unrolls (`grids[0]/[1]`, `brickSlabs[0..7]`) stay shared.

CMake: `if(APPLE)` need not compile `voxel_dda_rq.comp.spv` at all.

C++: AS helpers in `src/gfx/AccelStruct.cpp`, either `#if !defined(VE_PLATFORM_MACOS)` or runtime no-op if `!rayQuerySupported`. Load `vkCreateAccelerationStructureKHR` via `vkGetDeviceProcAddr` **after** a successful probe — Mac must not link as if those entry points exist.

### Scene ownership

`VoxelScene` keeps: coarse 3D textures, brick pool, occMip, hull mesh, `objectToWorld`.

AABB lists and BLAS are a **desktop cache**, not scene truth:

```
edit → dirtyPages flush + fillOccMip + rebuildOccupancyHull
         ├─ always: hull mesh (Mac)
         └─ only useHwRt: same occMip → AABB list → rebuild BLAS
```

RT-1 AABBs and hull macros must come from the **same** occupancy scan.

CPU pick (`pickObject`) stays software DDA on every platform. Editing must not depend on a TLAS.

### Visual / feature parity

Fallback is not “Mac low quality.” Same scene, same camera:

- Primary visibility: voxel faces, spinner motion, sky — match (1 texel silhouette slop OK)
- Edit: brush, import mesh, page alloc/free — same
- ImGui: render modes, AO, `maxSteps` on both; `Show hull tMin` is software-only
- RT-only extras (shadow rays in RT-3) **do not exist on Mac**. Do not bind shadows to `rayQuery`. Software path keeps current lighting, or a later shadow map, independently.

Layering:

```
Shared:          data + shading + editing
Software:        hull + DDA          ← all of Mac, desktop fallback
Hardware:        TLAS + rayQuery     ← optional acceleration
Hardware-only:   shadow rays, etc.   ← explicitly desktop extra
```

---

## 7. Device / memory (desktop RT path)

Once `rayQuerySupported`:

- Feature pNext: `Vulkan12Features.bufferDeviceAddress`, `accelerationStructure`, `rayQuery`
- VMA: `VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT`
- Buffer usage on AABB / instance / AS / scratch:
  - `VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR`
  - `VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR`
  - `VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT`
- `vkGetBufferDeviceAddress`
- New descriptor: `VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR` (e.g. binding 11)

Suggested modules (do not dump this into `VoxelScene.cpp`):

```
src/gfx/AccelStruct.cpp/.h      // create, scratch, build, destroy
src/scene/VoxelAsBuilder.cpp    // VoxelObject / occMip / bricks → AABB + meta
```

`VoxelScene`: `worldBlasDirty_` / `spinnerBlasDirty_`; after `uploadObjectTransforms`, write TLAS instances.

`VoxelRenderer`: build AS when dirty; bind TLAS; skip `recordHullPass` if `useHwRt`.

---

## 8. Shader (Ray Query)

Do **not** treat the AABB surface as the voxel hit. `rayQueryProceedEXT` yields a **candidate box**; run DDA inside, then `rayQueryConfirmIntersectionEXT` with the voxel `t`.

Sketch:

```glsl
rayQueryEXT q;
rayQueryInitializeEXT(q, tlas,
    gl_RayFlagsOpaqueEXT | gl_RayFlagsSkipTrianglesEXT,
    0xFF, origin, tMin, dir, tMax);

while (rayQueryProceedEXT(q)) {
    uint prim = rayQueryGetIntersectionPrimitiveIndexEXT(q, false);
    uint inst = rayQueryGetIntersectionInstanceCustomIndexEXT(q, false);
    AabbMeta m = metas[metaBase(inst) + prim];
    ObjectHit h = traverseAabb(objects[m.objectIndex], m, origin, dir);
    if (h.hit)
        rayQueryConfirmIntersectionEXT(q);
}

if (committed generated hit) shade(best);  // existing normal / AO / sky
else imageStore(..., sampleSky(dir));
```

`traverseAabb` is `traverseObject` with the start interval clipped to the one macro/brick. `traceMicro` unchanged.

Camera inside the volume: AABBs still test; `cameraInsideWorldAabb` hull special cases can go away **on the RT path only**. Keep them on software.

`MODE_STEPS` still counts DDA steps inside boxes, not BVH nodes.

AO: keep `neighborSolid`. Do not fire hardware AO rays in v1.

Shadows (RT-3 only): second `rayQuery` toward `lightDir` with terminate-on-first-hit. This is the first place HW RT is likely faster than “DDA every object again.” Mac never takes this branch.

---

## 9. Frame timeline (when `useHwRt`)

```
CPU:
  handleEditInput → dirty pages / maybe blasDirty
  uploadObjectTransforms                 // every frame (spinner)
  if worldOccMipDirty: fillOccMip + rebuild AABB list
  write TLAS instance buffer             // ~2 instances

GPU:
  if blasDirty:
      barrier
      vkCmdBuildAccelerationStructuresKHR(BLAS)
      barrier
      vkCmdBuildAccelerationStructuresKHR(TLAS)
  else:
      rebuild or UPDATE TLAS (instance matrices only)
  AS → COMPUTE barrier
  dispatch ray-query compute
  blit, imgui
```

`kFramesInFlight = 2`: do not rebuild a TLAS/BLAS that a previous frame is still tracing.

- Instance buffer: per-frame (tiny)
- World BLAS: wait the in-flight fence (or idle) on occupancy change; brush rate, not per pixel
- Spinner BLAS: **static**; TLAS update only

Build flags: `PREFER_FAST_TRACE`. Occupancy add/remove → **rebuild** BLAS (simpler than UPDATE). Compaction optional at this AABB count.

Typical sizes: hundreds–thousands of AABBs; memory is negligible vs the brick pool (up to 8×1024 pages).

---

## 10. Edit mapping

| Change | Existing | Extra if `useHwRt` |
|--------|----------|--------------------|
| Flip bits inside a live brick | `dirtyPages_` SSBO flush | **BLAS unchanged** |
| `freeBrickPage` / alloc / solid↔mixed that changes AABB set | coarse + pages | **rebuild that object’s BLAS** |
| Solid unallocated coarse (virtual full 8³) | no page | RT-1: covered if mip occupied; RT-2: merge |
| Spinner rotation | `uploadObjectTransforms` | **TLAS instance only** |
| `rebuildVoxels` / mesh import | full upload | full BLAS rebuild for that object |
| `rebuildOccupancyHull` | remesh boxes | `rebuildObjectBlas` from the same mip |

RT-1 dirty rule: rebuild BLAS only when a `4³` macro flips occupied. That is the same dirty timing as hull today. Rebuild on mouse-up or every N strokes, never per pixel.

---

## 11. Phased delivery

**RT-0 — infrastructure, no image change**

- Optional extension probe; Mac ⇒ false
- Two shader variants; Apple CMake may skip the RQ SPIR-V
- Empty AccelStruct module; software path bit-identical
- Device still boots if probe fails (no throw)

**RT-1 — primary visibility**

- occMip `4³` AABBs → per-object BLAS + TLAS
- `findHitRayQuery` replaces object loop + `tMin` on that pipeline
- Skip hull **draw** when `useHwRt`
- Accept: still vs software (silhouette slop OK); spinner OK; camera-in-ground OK

**RT-1b — edits**

- Interior brick bits: no rebuild
- Macro occupancy change: rebuild world BLAS
- ImGui: AABB count, BLAS bytes, last build ms, vs page-flush stats

**RT-2 — finer AABBs**

- Brick boxes + greedy merge of virtual-solid coarse
- DDA starts at one coarse (or merged plate), not a `4³` macro

**RT-3 — shadow ray (desktop extra)**

- Second `rayQuery`; software/Mac lighting unchanged

**Do not** in this note: RT pipeline + SBT, triangle voxels, per-voxel AABB, RT AO, Mac software-emulated AS, Metal RT.

---

## 12. Repo-specific risks

| Risk | Mitigation |
|------|------------|
| Virtual full brick (`brickPage == INVALID` ⇒ solid) | Those coarse cells **must** emit AABBs or the ground vanishes; RT-2 merges them |
| `grids[2]` is `usampler3D` | Intersection keeps `texelFetch`; do not revert to SSBO for RT |
| MoltenVK nonuniform indexing | Software shader keeps unrolls; RQ shader is desktop-only and may use `nonuniformEXT` if needed |
| `hullDilate_` | Raster conservative extra; BLAS should use **true** occupancy (dilation = extra empty boxes) |
| CPU pick | Always software DDA |
| Validation: scratch align, BDA usage, rebuild while in flight | Dual-buffer or fence; get this right before async builds |
| BDA as required feature | **Forbidden** — kills Mac device select |
| Deleting hull after RT-1 | **Forbidden** — hull is Mac |
| Shadow coupled to rayQuery | Forbidden for v1 parity; RT-3 is extra |

---

## 13. Expected performance

At current scale (one `48³` world + spinner):

- **Primary visibility (RT-1):** likely **tie or slightly slower** than hull+DDA (BVH + custom intersection overhead; hull already skips most air)
- **Many objects / sparse imports:** TLAS wins; software still `for objectCount`
- **Shadow rays (RT-3):** first clear win vs a second full DDA
- **Editing:** free if AABB set stable; a hitch if occupancy rebuilds every stroke

Hardware RT is not “enable RTX, get free FPS.” It is the scalable skip for many objects, sparse bricks, and extra rays — the same idea as Tuxedo Labs’ chunk AABB + intersection DDA.

---

## 14. Implementation order (fallback before RT)

1. Extract `findHit` in `voxel_dda.comp`; hull path remains default.
2. Optional RT probe on `GfxDevice`; Mac is always false.
3. Second `VE_RAY_QUERY` SPIR-V + AccelStruct; probe/load failure **silent fallback**, never `throw`.
4. Golden scene on Windows with RT off must match Mac (same code path). Then enable RT-1 on a probing GPU and compare stills.

---

## 15. Mapping onto this repo (checklist)

```
GfxDevice
  required features: unchanged
  optional: AS + rayQuery + BDA
  rayQuerySupported() / createBuffer device address

CMake
  voxel_dda.comp.spv always
  voxel_dda_rq.comp.spv if not APPLE (or always, but Mac never loads it)

VoxelScene
  unchanged storage
  occMip + hull always
  blasDirty only if useHwRt

VoxelAsBuilder (new, desktop)
  occMip → VkAabbPositionsKHR + AabbMeta

AccelStruct (new, desktop)
  BLAS per VoxelObject, one TLAS

VoxelRenderer
  useHwRt ? skip hull draw, bind TLAS, RQ pipeline
          : today’s recordHullPass + DDA pipeline

shaders/voxel_dda.comp
  #ifdef VE_RAY_QUERY findHitRayQuery
  #else findHitDda
  shared: traverse brick, AO, sky, MoltenVK unrolls
```

Identity regression: disable spinner, software path on Windows equals Mac. Ground without brick pages still looks like virtual full occupancy on **both** backends.
)
