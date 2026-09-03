# Coarse Grid as Sampled 3D Texture (N=48, then N=64)

Living design note. **Do not implement Phase 2 until Phase 1 is accepted.**

Status: **Phase 1 implemented; user measured no meaningful FPS change. Phase 2 not started.**

- **Phase 1 (in tree):** world `gridSize = 48`, GPU coarse-cell reads are a sampled 3D texture (`R32G32_UINT`, no mips). Visual/edit path unchanged.
- **Accept result:** overhead did not drop; still GPU-bound. Coarse-cell fetch layout is **not** the limiter at 48³ (~108 KiB fits in cache). Do **not** treat 3D textures as the next FPS project.
- **Phase 2 (N=64): stopped.** Do not grow the grid to spend the padding until a real bottleneck is fixed.

Related shipped code:

- `src/gfx/GfxDevice.cpp` — `createImage` is hardcoded `VK_IMAGE_TYPE_2D` / `VK_IMAGE_VIEW_TYPE_2D`
- `src/scene/VoxelScene.h` / `.cpp` — CPU `VoxelObject.cells` (`CoarseCell {material, brickPage}`), packed SSBO `voxelBuffer_`, `voxelOffset`
- `src/render/VoxelRenderer.cpp` — set 0 binding 1 = `STORAGE_BUFFER` (`VoxelGrid`)
- `shaders/voxel_dda.comp` — `readCell()` indexes `cells[voxelOffset + x + yN + zN²]`
- `docs/nested-brick-voxels.md` — coarse → sparse 8³ → 2×2×2 (unchanged by this work)
- `docs/mesh-solid-voxelize.md` — import-time surface voxelize (stays SSBO; not this work)

---

## 1. Goal

Keep:

- Three-level DDA: **coarse N³ → brick 8³ → fine 2×2×2**
- CPU `cells[]` as source of truth (pick, brush, stamp, occMip rebuild)
- Brick pool SSBO, occupancy-mip SSBO (`S = 4`), object UBO, import palette
- `voxelSize = 0.35` m
- World object + spinner 8³ (two grids)
- Unified world stamp (imported mesh writes into the world grid)
- No texture mip chain, no storage-image `imageLoad`

Change:

- GPU coarse fetch only: SSBO `cells[]` → sampled `usampler3D` + `texelFetch`
- Binding 1: storage buffer → combined image sampler array of count 2

Why: when the camera is **outside and far**, an 8×8 compute group hits many unique coarse cells. Linear SSBO layout `x + yN + zN²` makes those loads uncoalesced (L1 / 64-byte cache lines, ~8 cells per line). A 3D texture puts the same 8-byte cells on the hardware texture cache, which is built for 3D spatial locality. This is **not** an MMU page-table problem and **not** a brick-page problem.

Non-goals (this document):

- Screen-space LOD / coarsening far cells (would change visual grain if done badly)
- Morton reordering of the SSBO (alternative; not chosen)
- Occupancy or bricks as 3D textures
- Import `voxelize_surface.comp` occupancy SSBO
- Filling driver **pitch / tiling padding** of a 48³ image as extra voxels (impossible; see §4)
- Changing edit granularity

---

## 2. What stays as fine as today

The 3D texture **replaces only the coarse GPU read**. Traversal after `readCell` is identical:

```
world ray → object local
  DDA coarse [0, N)          ← texelFetch instead of cells[i]
    air          → skip
    solid + brickPage INVALID → hit as one cube; reconstruct 8³ / 2×2×2 for fine AO
    solid + valid brickPage   → DDA 8³ in BrickPool SSBO
      mixed micro             → DDA 2×2×2 in the same page
```

| Data | Storage after Phase 1 | Grain |
|------|------------------------|--------|
| Coarse cell | CPU `cells[]` + GPU 3D tex `R32G32_UINT` | N³, still 0.35 m |
| Brick occupancy | BrickPool SSBO | 8³ inside a coarse cell |
| Fine occupancy | same brick page | 2×2×2 inside a micro |
| Occupancy mip | SSBO binding 7 | 4³ skip, not a texture mip |
| Import occupancy | `voxelize_surface.comp` SSBO | import-time only |

A texel is one coarse cell. It is **not** a bigger Minecraft-style block. Nested DDA does not flatten.

---

## 3. Gate: 48 first, 64 only after accept

| Phase | Logical `gridSize` | `voxelSize` | World AABB | When |
|-------|--------------------|-------------|------------|------|
| **1** | **48** (current default) | 0.35 m | 16.8 m | Implement now |
| **2** | **64** | 0.35 m (unchanged) | 22.4 m | **Only after Phase 1 is accepted** |

Phase 1 must not change `gridSize_`, `importGridN_`, DDA bounds, or stamp size. The only user-visible intent is: **same scene, faster far-outside coarse fetches** (and no visual/edit regression).

Phase 2 is a **logical** N change, not “use the 48³ image’s hidden padding.” See §4.

---

## 4. Memory, alignment, and why unused pad is not voxels

### 4.1 Payload (addressable texels)

`CoarseCell` is 8 bytes (`uint material` + `uint brickPage`). Format `VK_FORMAT_R32G32_UINT` is 1:1.

| Grid | Cells | Payload |
|------|-------|---------|
| World 48³ | 110 592 | **110 592 B ≈ 108 KiB** |
| Spinner 8³ | 512 | 4 096 B |
| World 64³ (Phase 2) | 262 144 | **256 KiB** |

CPU `cells[]` stays. GPU SSBO of the same payload goes away; GPU 3D image payload is the same size. This path is **not** a memory save.

### 4.2 Driver alignment / tiling

`vkCreateImage` extent is the **logical** size (`48×48×48`). The allocator may round the **physical** allocation (row pitch, tile, Metal texture alignment). Those extra bytes:

- Occupy VRAM
- Are **not** addressable as `x = 48 … 63`
- Are **not** extra voxels DDA can step into
- Cannot be “filled with real voxels” without changing `extent` and `gridSize`

Vulkan does **not** round a 48³ image up to a 64³ image that the shader can index. Alignment waste is padding around the 48³ texel box, not a free 64³ grid.

### 4.3 “If 48 is aligned to 64, why not store voxels there?”

48 is **not** aligned to 64 in the API. Some backends pad internally; that pad is not a valid `texelFetch` coordinate. To turn that VRAM into voxels:

1. Set logical `N = 64`
2. Create a **64³** image
3. Run DDA / pick / occMip / stamp on `[0, 64)`

That is Phase 2, gated on Phase 1 accept. Keep `voxelSize = 0.35` unless a later decision shrinks the cell (finer grain, same AABB) — **not** in this plan.

### 4.4 “Width”

For a 3D image, `VkExtent3D` is:

- `width`  → X (matches `p.x`)
- `height` → Y (`p.y`)
- `depth`  → Z (`p.z`)

Today `createImage` is 2D: `depth = 1`, `imageType = 2D`. Phase 1 adds a 3D path; 2D callers stay 2D.

---

## 5. GPU layout

### 5.1 Images

Two sampled 3D images, **no mipmaps** (`mipLevels = 1`):

| Slot | Object | Extent | Index in `grids[]` |
|------|--------|--------|--------------------|
| World | `objects_[0]` | `N×N×N` (Phase 1: 48) | 0 |
| Spinner | `objects_[1]` | `8×8×8` | 1 |

Reuse `GpuVoxelObject.voxelOffset` as **texture index** (0 or 1). Struct size stays 176 bytes. When the spinner is disabled it is omitted from `objectsGpu_`; both images still exist and stay bound.

Dummy: if a slot is unused, bind a 1×1×1 `R32G32_UINT` image (air) so the descriptor array is always valid.

### 5.2 Image create

```
VK_IMAGE_TYPE_3D
VK_IMAGE_VIEW_TYPE_3D
VK_FORMAT_R32G32_UINT
mipLevels = 1
arrayLayers = 1
tiling = OPTIMAL
usage = SAMPLED | TRANSFER_DST     // not STORAGE
aspect = COLOR
```

**Sampled**, not storage. `imageLoad` on a storage image often bypasses the texture cache; that would miss the point.

Query `vkGetPhysicalDeviceFormatProperties` for `R32G32_UINT` optimal: `SAMPLED_IMAGE` + `TRANSFER_DST`. Fail loudly if missing (MoltenVK on Apple Silicon is expected to support it). No linear-filter requirement (`texelFetch` does not filter).

### 5.3 Sampler

One sampler, shared by both images:

- `mag/min = NEAREST`
- `mipmapMode = NEAREST`
- `addressMode U/V/W = CLAMP_TO_EDGE`
- `anisotropyEnable = FALSE`
- `mipLevels = 1`, lod bias 0

`texelFetch` ignores filtering, but CIS still needs a sampler. Do not use the existing sky sampler (linear + anisotropy).

Existing `GfxDevice::createSampler` hardcodes `mipmapMode = LINEAR`. Either add a mipmap-mode argument or create this sampler locally in `VoxelScene` / `VoxelRenderer`.

### 5.4 Upload

CPU layout already matches a tightly packed 3D copy: index `x + yN + zN²`, texel = `CoarseCell` = 8 bytes.

`vkCmdCopyBufferToImage`:

- `bufferRowLength = 0`, `bufferImageHeight = 0` (tight)
- `imageExtent = {N, N, N}`
- whole image, mip 0, layer 0

Phase 1 uploads the **whole** object grid on create / rebuild / edit (~108 KiB world). Do **not** use current `uploadToImage` mip blit (2D `srcOffsets.z = 1`). New `uploadToImage3D` with **no blit, no mips**.

After copy, layout `SHADER_READ_ONLY_OPTIMAL`. Barrier dst stage must include **compute** shader read (today’s 2D upload barriers to fragment only — wrong for DDA).

Do not transition from `UNDEFINED` on a **partial** update (destroys the rest of the image). Phase 1 whole-grid upload may start from `UNDEFINED`.

`immediateSubmit` is fine at this size.

### 5.5 Shader

Replace binding 1:

```glsl
layout(set = 0, binding = 1) uniform usampler3D grids[2];

CoarseCell readCell(GpuVoxelObject o, ivec3 p) {
    uvec2 packed;
    // Unroll: MoltenVK has been unreliable with nonuniform array indexing
    // (same reason brick slabs are unrolled).
    if (o.voxelOffset == 0u) {
        packed = texelFetch(grids[0], p, 0).xy;
    } else {
        packed = texelFetch(grids[1], p, 0).xy;
    }
    CoarseCell c;
    c.material = packed.x;
    c.brickPage = packed.y;
    return c;
}
```

`insideGrid` still uses `o.gridSize`. Do not fetch out of bounds (DDA already clamps / tests).

Remove `layout(... ) readonly buffer VoxelGrid { CoarseCell cells[]; }`.

AO (`evalVoxelAo`) and occupancy tests keep calling `readCell` — they automatically go through the texture.

### 5.6 Descriptors (`VoxelRenderer`)

| Binding | Today | Phase 1 |
|---------|-------|---------|
| 0 | UBO | unchanged |
| **1** | **SSBO VoxelGrid count=1** | **CIS `usampler3D` count=2** |
| 2 | storage image out | unchanged |
| 3 | brick slabs SSBO[8] | unchanged |
| 4 | sky CIS | unchanged |
| 5 | object SSBO | unchanged (`voxelOffset` = tex index) |
| 6 | palette SSBO | unchanged |
| 7 | occMip SSBO | unchanged |

Pool: add CIS count (`kFramesInFlight * (1 sky + 2 grids)`). Remove one storage-buffer slot that was the coarse grid.

`updateDescriptors` binds world view + spinner view + nearest sampler. Refresh when either 3D image/view changes (replace `boundVoxelBuffer_`).

`draw` early-out: require 3D image views, not `voxelBuffer()`.

---

## 6. CPU / scene

Keep `VoxelObject.cells`. Pick, brush, `fillOccMip`, stamp, import pack **do not** switch to GPU readback.

`packObjectPool` today concatenates cells into `voxelsCpu_` and sets `voxelOffset`. After Phase 1:

- `voxelOffset` = `0` for world, `1` for spinner (not a byte/cell offset)
- `voxelsCpu_` / `voxelBuffer_` SSBO can be **removed** from the DDA path
- `flushObject` uploads that object’s `cells` into its 3D image (whole grid)
- `rebuildVoxels` / `uploadWorldAndObjects` create-or-resize images then upload both

`occMip` pack/upload unchanged.

`ensureGpuBuffers` no longer sizes `voxelBuffer_` for DDA. Brick slabs, objects, palette, occMip stay.

`cleanup` destroys both 3D images, dummy 1³, and the nearest sampler.

---

## 7. GfxDevice API (do not break 2D)

Add; do **not** change `createImage` to 3D (every HDR / depth / blit path is 2D).

```cpp
AllocatedImage createImage3D(VkExtent3D extent, VkFormat format, VkImageUsageFlags usage,
                             VkImageAspectFlags aspect, bool dedicated = false);
void uploadToImage3D(AllocatedImage& image, const void* data, VkDeviceSize size,
                     VkExtent3D extent);
```

`createImage3D`:

- `imageType = VK_IMAGE_TYPE_3D`
- `viewType = VK_IMAGE_VIEW_TYPE_3D`
- `mipLevels = 1` (assert / ignore requests for more)
- `extent.depth >= 1`

`uploadToImage3D`:

- reject `generateMips`
- `vkCmdCopyBufferToImage` with 3D extent
- barrier to compute + fragment shader read

Optional: store `imageType` on `AllocatedImage` if destroy/upload need to distinguish; otherwise infer `extent.depth > 1 && view is 3D`.

---

## 8. Phase 1 execution (N=48 only)

Implement in this order. **Do not bump `gridSize_`.**

### P1.1 — `GfxDevice` 3D

1. `createImage3D` + `uploadToImage3D` as §7.
2. Format-support check for `R32G32_UINT` sampled + transfer dst (can live in `VoxelScene::init` or first create).
3. Existing 2D `createImage` / cubemap / HDR unchanged.
4. Compile.

### P1.2 — Scene images

1. `VoxelScene`: `AllocatedImage grid3D_[2]`, dummy 1³, `VkSampler gridSampler_`.
2. Create world `48³` and spinner `8³` after `buildGroundObject` / `buildSpinnerObject`.
3. `fillGpuObjectRecords`: `g.voxelOffset = objectIndex` (0 / 1), still skip disabled objects in the GPU list.
4. Upload `o.cells.data()` via `uploadToImage3D`.
5. `flushObject` / rebuild / import stamp: re-upload the dirty object’s image.
6. Keep SSBO `voxelBuffer_` **temporarily** so the engine still runs until P1.3–P1.4 switch (or switch in one step if preferred; see P1.4).

### P1.3 — Descriptors

1. Binding 1 → CIS count 2.
2. Pool sizes.
3. `updateDescriptors` writes two `VkDescriptorImageInfo` (sampler + view + `SHADER_READ_ONLY_OPTIMAL`).
4. Track bound image views instead of `boundVoxelBuffer_`.
5. `resize()` still only recreates the 2D output image; 3D grids are scene-lifetime.

### P1.4 — Shader

1. `usampler3D grids[2]`, unrolled `texelFetch` in `readCell`.
2. Rebuild `voxel_dda.comp.spv`.
3. Drop DDA use of `voxelBuffer_`. Destroy / stop allocating the coarse SSBO once the shader no longer reads it.
4. `voxelize_surface.comp` **untouched**.

### P1.5 — Verify Phase 1 (user accept)

Same hut / ground / spinner as today (`N = 48`, `voxelSize = 0.35`).

| Check | Expect |
|-------|--------|
| Shaded / albedo / normal | Same grain as SSBO path; no 4× blockier far walls |
| Nested on | Carve still 8³ then 2×2×2; INVALID coarse still fine AO reconstruct |
| Nested off | Coarse cubes only |
| Pick / place / delete | Same as today; first carve still `ensureBrickPage` |
| Spinner on/off | Second 8³ image; disable → `objectCount = 1` |
| Import stamp | Still writes world 48³; 3D image re-uploaded |
| Steps mode | Coarse step count same order of magnitude (cache, not step budget) |
| **Far outside, looking at the hut** | GPU compute ms **down** vs SSBO (the bug this is for) |
| **Close to a wall (in or out)** | No large regression (working set already small) |
| Memory | Payload ~108 KiB + driver pad; not a 64³ grid |
| Mips | `mipLevels == 1` on both images |

If far-outside does **not** improve, stop. Do not start Phase 2. Debug: confirm sampled (not storage), `texelFetch` (not `texture`), unrolled index, compute barrier, no accidental mip.

**Phase 1 measured (user):** no meaningful overhead change, still GPU-bound. Hypothesis that unique coarse SSBO addresses in an 8×8 were the limiter is **rejected** at N=48. Whole coarse table is ~108 KiB; Apple Silicon unified cache already holds it. Changing fetch layout cannot help. **Do not start Phase 2.**

---

## 9. Phase 2 execution (N=64) — gated

Only after §8 accept. This makes the **logical** grid 64 so VRAM is voxels, not pitch.

### What changes

1. `gridSize_ = 64` (and UI / rebuild clamp already allows 64).
2. `importGridN_ = 64` default (stamp still fits the world).
3. Recreate world 3D image `64×64×64`. Spinner stays 8³.
4. `buildGroundObject`, occMip (`ceil(64/4)=16` per axis), pick bounds, DDA `o.gridSize`, `maxSteps_ = n * 3` (= 192).
5. `voxelSize` **stays 0.35** → world AABB 16.8 m → 22.4 m. Ground still 2 coarse cells thick.
6. Payload 256 KiB. Upload still whole-grid.

### What does not change

- 3-level DDA, brick SSBO, occMip SSBO, no texture mips
- Spinner 8³
- Edit grain (still coarse / 8³ / 2×2×2)
- Format, sampler, bindings

### Phase 2 accept

| Check | Expect |
|-------|--------|
| World is larger (same 0.35 m cells) | Hut stamp still sits on the ground grid; extra cells are air / ground |
| Nested edit | Unchanged fineness |
| Far-outside FPS | At least as good as Phase 1; more empty coarse steps possible (`maxSteps` 192) |
| Memory | ~256 KiB payload + tiling; no unused “48-in-64” hole in the **logical** grid |

If the user later wants **finer** cells in the same AABB, that is a different change (`voxelSize` down, `N = 64`). Not this phase.

---

## 10. File list

| Path | Phase 1 | Phase 2 |
|------|---------|---------|
| `src/gfx/GfxDevice.h/.cpp` | `createImage3D`, `uploadToImage3D` | none |
| `src/gfx/GpuTypes.h` | optional `imageType` on `AllocatedImage` | none |
| `src/scene/VoxelScene.h/.cpp` | two 3D images, sampler, upload/flush, `voxelOffset` as tex index, drop coarse SSBO | `gridSize_` / `importGridN_` 64, recreate 64³ |
| `src/render/VoxelRenderer.h/.cpp` | binding 1 CIS×2, pool, bound views | none |
| `shaders/voxel_dda.comp` | `usampler3D` + unrolled `texelFetch` | none (`gridSize` comes from UBO) |
| `shaders/voxelize_surface.comp` | **no change** | no change |
| `docs/coarse-cell-3d-texture.md` | this file | mark Phase 2 implemented |

No new shader for voxelize. No mipmap generate.

---

## 11. Risks

| Risk | Mitigation |
|------|------------|
| MoltenVK `R32G32_UINT` 3D sampled missing | Check format features at init; fail with a clear error |
| `imageLoad` / storage image used by mistake | Usage bits = SAMPLED only; shader is `usampler3D` |
| Nonuniform `grids[i]` | Unroll 0/1 like brick slabs |
| Upload barrier only fragment | `uploadToImage3D` dst includes compute |
| `createImage` flipped to 3D | New function; 2D path untouched |
| Partial upload from `UNDEFINED` | Phase 1 whole-grid only |
| Visual LOD / blockier far | **No mips, no LOD**; `texelFetch` lod 0 |
| N=64 before 48 is proven | Hard gate: do not change `gridSize_` in Phase 1 |
| Treating pitch as voxels | Impossible; Phase 2 grows logical N instead |
| Import / pick broken | CPU `cells[]` unchanged; only GPU read path moves |
| 8×8 still slow after 3D tex | Stop; do not do 64. Next options (not this doc): Morton SSBO, half-res primary, disable far fine AO |

---

## 12. Defaults

| Knob | Phase 1 | Phase 2 (after accept) |
|------|---------|------------------------|
| World `gridSize` | **48** | **64** |
| `voxelSize` | 0.35 m | 0.35 m |
| Spinner N | 8 | 8 |
| Texture mips | **none** | none |
| Image type | sampled 3D | sampled 3D |
| Format | `R32G32_UINT` | same |
| Filter | nearest / texelFetch | same |
| Brick / fine | SSBO nested | same |
| occMip | SSBO S=4 | same, 16³ bits |
| Coarse SSBO | removed from DDA | still gone |

---

## 13. Open points (do not block Phase 1)

1. Dirty-box `VkBufferImageCopy` instead of whole 48³ on every brush — later; 108 KiB is cheap.
2. `AllocatedImage.imageType` field vs inferring 3D from extent.
3. Keep a debug SSBO mirror of `cells[]` — default no.
4. Phase 2: grow AABB vs shrink `voxelSize` — **this plan grows AABB**. Shrink only if asked after 64.
5. More than two objects: grow `grids[]` and unroll, or bindless later. Current engine: world + spinner.

---

## 14. References (why texture cache, not LOD)

- Aila & Laine — SIMT divergence + cache; primary rays that fan out in world space thrash linear buffers
- Hardware 3D texture cache vs SSBO coalescing (Vulkan sampled image vs storage buffer)
- GigaVoxels / ESVO screen-footprint LOD — **not used here** (would change far grain; user rejected edit-grain change; visual LOD is a different project)
- VoxelRT MultiDDA / BrickMap — bricks stay; this work only relocates the **coarse** table

In-engine: `docs/nested-brick-voxels.md`, `docs/mesh-solid-voxelize.md`.
