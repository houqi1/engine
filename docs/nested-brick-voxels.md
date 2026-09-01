# Nested Sparse Bricks (Empty Bricks Unallocated)

Living design note for making voxel detail finer **without** a dense `N³ × 8³` blow-up.

Status: **Phase A + 2×2×2** — sparse 8³ brick slabs plus optional 512-byte 2×2×2 tables per brick. Unallocated brick/table = virtual full occupancy.

Related shipped code:

- `src/scene/VoxelScene.h` / `.cpp` — `VoxelObject`, coarse `CoarseCell` + global `BrickPool`, rigid transforms, dirty-page flush
- `shaders/voxel_dda.comp` — local Amanatides–Woo DDA; inner `8³` only when `brickPage != INVALID`

---

## 1. Goal

Keep:

- Axis-aligned grids **in object-local space**
- Rigid `objectToWorld` / `worldToObject` (rotating, editable objects)
- Compute DDA raycasting (no primary mesh extraction)

Change:

- Stop reserving an `8³` micro brick for **every** coarse cell
- Allocate brick pages **only where mixed / edited detail exists**
- Optionally add a second nested level (`coarse → brick → micro`) or grow brick size (`8 → 16`) **only on allocated pages**

Finer look comes from hierarchy or brick extent. Memory scales with **occupied/edited volume**, not with `N³`.

---

## 2. Current engine (baseline)

```
VoxelObject
  position, rotation (rigid)
  dense coarse: N³ uints  (material + HAS_MICRO bit)
  dense micro:  N³ × 16 uints  (512 occupancy bits = 8³ per coarse cell)
```

Traversal (`voxel_dda.comp`):

1. Transform world ray into object local space
2. DDA on coarse `[0, N)³`
3. If cell solid and `HAS_MICRO`: DDA inside that cell’s `8³`
4. Shade with world-space normals; AO from local neighbors

Problems for finer detail:

- Micro resolution is **fixed at 8**
- Micro storage is **dense sidecar**: `microOffset + coarseIndex * 16`
- Raising 8 → 16 multiplies micro memory by 8 for **all** cells, including empty and fully solid ones

---

## 3. Target structure

### 3.1 Two-level sparse (minimum viable)

```
Level 0 — coarse grid  N³
  per cell: material, flags, brickPageId  (INVALID = no brick)

Level 1 — brick pool
  page: B³ occupancy (and later material)
  B = 8 (first) or 16 (optional)
```

Cell kinds:

| Coarse cell | `brickPageId` | Meaning |
|-------------|---------------|---------|
| Air | INVALID | Skip; no page |
| Solid, no interior detail | INVALID | Hit as one cube (cheap) |
| Mixed / carved / micro-detail | valid page | Enter brick DDA |

**Empty bricks are never allocated.** Fully solid cells need not allocate a brick either (collapse).

### 3.2 Three-level (shipped: 8³ then 2×2×2)

```
coarse  (e.g. 0.35 m)
  └─ brick page  8³ occupancy + fineTableId
        └─ fine table  512×uint8  (one 2×2×2 per micro) only if mixed
```

Linear fineness vs coarse = 16. Fine table is 512 bytes vs another 8³ (64 bytes occupancy but 512 cells — we store 8 bits/micro). `INVALID` fineTableId = virtual `0xFF` for every occupied micro.

### 3.3 Variable brick size (optional)

Store `B` in the brick header (`8` or `16`). Nearby / hero objects use 16; filler uses 8. DDA step size is read from the header. More shader branching — not phase 1.

---

## 4. Memory layout (CPU / GPU)

Replace dense `micro[]` stride with a **page pool**.

```
SSBO VoxelGrid     — coarse[] per object (still linear N³, or later sparse)
SSBO BrickPool     — fixed-size pages, e.g. 16 uints for 8³ bits
SSBO ObjectBuffer  — existing GpuVoxelObject (matrices, offsets, flags)
```

Coarse cell (sketch):

```
uint packedMaterial;     // low bits material, flags
uint brickPage;          // 0xFFFFFFFF = none
```

Brick page (8³ occupancy, matches current packing `y*64 + z*8 + x`):

```
uint bits[16];           // 512 bits
uint microPage;          // INVALID, or page in a second pool (level 3)
uint flags;              // mixed / dirty / lod
```

Allocator (CPU):

- `freeList` of page indices
- `allocPage()` / `freePage()`
- Edit: `flush` only dirty pages via `uploadToBuffer(..., dstOffset)` (already exists on `GfxDevice`)

Object still uses `voxelOffset` into the coarse pool. Brick pages can be:

- **Global pool** (simple, one descriptor), or
- Per-object brick range (`brickBase + localPage`) if we want isolation

**Decision (open):** global pool vs per-object brick SSBO. Default proposal: **one global BrickPool** + `brickPage` as global index.

---

## 5. Traversal (DDA, unchanged kernel)

Same Amanatides–Woo step as today; extra **enter/leave** conditions:

```
ray → worldToObject → local
DDA coarse [0, N)

hit solid coarse:
  if brickPage == INVALID:
      treat as full cube (current nested-off / no-micro path)
  else:
      map ray into brick local [0, B)
      DDA brick
      if (level-3) and cell has microPage:
          DDA micro [0, M)
```

Normals stay face-axis in the **hit level**, then `N_w = R * N_l`.

AO stays neighbor samples in the hit level; neighbors may sit in another page or only exist as coarse solid — **must treat INVALID brick + solid coarse as fully occupied** for AO.

Step budget: keep `maxSteps` per level (coarse `ubo.maxSteps`, brick 32 as today, micro 32).

---

## 6. Editing

Pick already returns `objectIndex` + local `cell` / `micro`. Extend:

1. Hit coarse, no brick, **fine** brush:
   - `allocPage`, initialize page **filled solid** from coarse material
   - set `brickPage`, carve in brick space
2. Hit existing brick: mutate page bits
3. After edit:
   - brick all empty → `freePage`, coarse = air
   - brick all solid and no holes (optional collapse) → `freePage`, coarse = solid
4. `flushObject` becomes `flushCoarseRange` + `flushDirtyPages`

Rotating objects: still local grids; holes ride the rigid transform (already true for spinner).

---

## 7. Why not SVDAG for this step

| Nested bricks | SVDAG / HashDAG / Aokana |
|---------------|---------------------------|
| Page table + DDA, fits this repo | Pointer DAG, different traversal |
| Edit = alloc/mutate page | Edit = rebuild root path / hash nodes |
| Compresses **emptiness / unrefined cells** | Also merges **identical subtrees** worldwide |
| Incremental on `VoxelObject` | New subsystem |

Use DAG later for huge static / distant LOD. Nested bricks are the **editable, local** path.

---

## 8. Related work (implementations)

Projects that actually use **bricks / unallocated empty chunks**, not just papers:

| Project | What they do | Link |
|---------|----------------|------|
| **Tuxedo Labs next engine** (post-Teardown) | Sparse **8³ chunks** per volume, occupancy bitmap, empty not stored; chunk AABBs in BLAS + intersection shader DDA. Chose 8³ over 4³ (faster render, fewer chunks, unified physics). | [Voxagon 2024](https://blog.voxagon.se/2024/12/29/year-summary.html), [GPC talk](https://www.youtube.com/watch?v=IM1Dr98f3xU) |
| **Teardown (current)** | Dense per-object volume + mip skip 2³/4³ — closer to **today’s** engine than sparse pages | [Frame teardown](https://juandiegomontoya.github.io/teardown_breakdown.html) |
| **BrickMap** | Superchunk 16³ of **8³ bricks**, linear GPU pool + 12-bit indices, stream **surface bricks only**, LOD 8/2/1 | [stijnherfst/BrickMap](https://github.com/stijnherfst/BrickMap) |
| **VoxelRT** | `MultiDDA`: top grid → 8³ bricks; `eXtendedBrickMap`: 3-level + occupancy bitmasks. Benchmarks: flat DDA ~31 vs MultiDDA ~148 Mrays/s (Sponza) | [dubiousconst282/VoxelRT](https://github.com/dubiousconst282/VoxelRT) |
| **VoxelHex** | Sparse tree **leaves are n³ bricks**; brick empty / **solid (one descriptor)** / mixed (full matrix) | [Ministry-of-Voxel-Affairs/VoxelHex](https://github.com/Ministry-of-Voxel-Affairs/VoxelHex) |
| **GigaVoxels** | N³-tree + mipmapped brick **pool**, ray-guided LRU | [gigavoxels.inria.fr](https://gigavoxels.inria.fr/) |

Cubiquity / Aokana are **micro-voxel via SVDAG**, listed for contrast, not as the brick-pool blueprint.

---

## 9. Mapping onto this repo

```
Today:
  VoxelObject → dense coarse N³ → reserved 8³ per cell

Phase A (sparse, still B=8):
  coarse.brickPage → BrickPool page | INVALID
  shader: enter traceBrick only if page valid
  solid ground: no pages (or keep pages until edit path is proven)

Phase B (finer):
  B=16 on allocated pages only
  — or —
  third level micro pages (preferred)

Keep:
  GpuVoxelObject transforms
  object loop + closest world t
  CPU pick in local space
  per-object edit
```

Identity regression: disable spinner, ground without bricks should match current nested-off / solid-micro visual for the slab.

---

## 10. Phased delivery

### Phase A — Sparse 8³ pages (no extra resolution)

1. Brick pool + `INVALID` page id on coarse
2. Shader enter inner DDA only if page allocated
3. First fine edit `alloc` + fill solid + carve
4. Empty brick `free`
5. Flush dirty pages only
6. ImGui: page count, pool bytes

**Accept:** same 8³ look; micro/brick SSBO much smaller on sparse objects (spinner shell).

### Phase B — Finer

- **B1:** allocated pages are 16³  
- **B2 (preferred):** keep 8³ bricks, add optional micro pages  

### Phase C — Quality / skip

- Brick occupancy mip / 4³ bitmask (VoxelRT-style skip)
- Collapse solid bricks
- AO across page boundaries hardened

Out of scope here: SVDAG, clipmaps, hardware RT BLAS (Teardown-next).

---

## 11. Risks

| Risk | Mitigation |
|------|------------|
| Page id / packing mismatch CPU vs GPU | Shared constants; `INVALID = 0xFFFFFFFF`; golden cube |
| AO leaks through “solid coarse, no brick” | Neighbor solid if coarse solid OR brick bit |
| Allocator fragmentation | Free list; optional compact on rebuild |
| Inner DDA step explosion | Cap steps per level |
| Ground still dense N³ | Accept for Phase A; coarse can stay dense |

---

## 12. Open questions (to refine next)

Track decisions here as we iterate.

1. **Brick size B:** start 8 only, or allow 16 in the header from day one?
2. **Solid collapse:** skip in Phase A (always allocate on first micro edit) or collapse immediately?
3. **Ground:** never allocate bricks for the slab until carved, or keep current filled-micro look?
4. **Pool ownership:** one global `BrickPool` vs per-`VoxelObject` brick buffer?
5. **Materials inside bricks:** occupancy bits only (current micro) vs per-voxel material in the page?
6. **Third level:** commit to B2 (8+8) vs B1 (16) before writing shader?
7. **Coarse density:** keep dense `N³` coarse table, or sparse coarse later?

**Locked for Phase A (implemented):**

- `B = 8`, occupancy-only pages, **global BrickPool**, ground **unallocated until carved**, no collapse, no 16, no third level.

---

## 14. GPU brick-pool residency (complete scheme)

Phase A shipped a correct **CPU** page table but a broken **GPU** growth path: `ensureGpuBuffers` destroys the SSBO and `flushDirtyPages` only uploads new pages, so previously allocated spinner bricks vanish. This section is the production residency design. It is what BrickMap, GigaVoxels-style atlases, HashDAG-style virtual tables, and typical Vulkan growable buffers actually do.

### 14.1 Non-goals (on this engine, especially MoltenVK)

| Idea | Why not primary |
|------|-----------------|
| `VK_BUFFER_CREATE_SPARSE_BINDING_BIT` + `vkQueueBindSparse` | Virtual address stays stable (HashDAG-like), but MoltenVK/Metal sparse-buffer support is weak and the bind/sync model is heavy. |
| One GPU `vkAllocateMemory` per brick | BrickMap paper path; fragment VRAM and CPU; they **replaced** it with a linear pool. |
| Recreate SSBO and upload only dirty pages | **Current bug.** Dirty-only is valid **only** while the `VkBuffer` identity is unchanged. |
| Full SVDAG / HashDAG as the brick store | Solves compression, not this pool bug; different traversal. |

### 14.2 Invariants

1. **Page indices are stable** for the lifetime of a live allocation (`freeList` reuse is allowed only after `freeBrickPage`).
2. **GPU capacity ≥ CPU `brickPool_.size()`** at the start of any frame that traces.
3. After a **new** `VkBuffer` is bound, its contents equal the full CPU pool (or a GPU copy of the previous buffer plus the new tail).
4. **Do not destroy** a `VkBuffer` that any in-flight frame still references. Retire it until that frame’s fence signals (same rule as [dynamic graphics buffers](https://vec3.ca/posts/dynamic-graphics-buffers) and Khronos “update descriptors at a safe point”).
5. Dirty-page uploads are an **optimization on a live buffer**, never a substitute for (3).

### 14.3 Capacity policy (CPU + GPU together)

```
kMinPages     = 1024          // or 4096; spinner shell is O(10²) pages
kPageBytes    = 16 * 4        // 8³ occupancy
kMaxPages     = 1u << 20      // hard cap; fail alloc or evict (Phase C)

capacityPages = max(kMinPages, nextPow2(usedPages))

on alloc if used == capacity:
    newCap = min(kMaxPages, max(capacity * 2, used + 1))
    resize CPU vector
    grow GPU buffer (14.4)
```

- **Reserve `kMinPages` at `rebuildVoxels`** so the first ground carve does not grow GPU memory.
- **Double**, never `+1` page (BrickMap: 256→512→1024…; SmartGI atlas: double + extra copy pass).
- Optional: never shrink GPU capacity during play (high-water mark). CPU `freeList` reuses holes.

### 14.4 Grow algorithm (the missing piece)

```
growBrickPoolGpu(newBytes):
    wait or defer until no frame is recording against oldBuf
    newBuf = createBuffer(newBytes)          // DEVICE_LOCAL + TRANSFER_DST|SRC|STORAGE

    if oldBuf exists and oldBytes > 0:
        // Preferred: GPU copy (no extra PCIe of the old working set)
        vkCmdCopyBuffer(oldBuf, newBuf, {0, 0, oldBytes})
        barrier: TRANSFER_WRITE → SHADER_READ (compute)
    else:
        uploadToBuffer(newBuf, brickPool_.data(), cpuBytes)

    bind newBuf in descriptors (all frames, at fence-safe point)
    retire oldBuf → destroy after FiF drain
    gpuCapacityBytes = newBytes
```

If the renderer has no easy GPU copy in the edit path, **CPU full upload of `brickPool_` into `newBuf`** is acceptable (pool is small: 1024 pages ≈ 64 KiB). Either way: **new buffer must be fully defined** before the next DDA dispatch.

After grow, `dirtyPages_` may be cleared if the full CPU image was uploaded; if only GPU-copied the old range, still upload dirty pages that landed in the **new tail**.

### 14.5 Steady-state updates (no grow)

Keep `flushDirtyPages` as specified in Phase A:

- Copy edited `CoarseCell` range into the pooled coarse SSBO (already done).
- Upload each dirty brick page at `page * kPageBytes`.
- Batch copies + one barrier if many pages (Vulkan best practice).

Do **not** recreate the buffer. Do **not** upload the whole pool every edit.

### 14.6 Descriptor / FiF

Current renderer already rebinds when `brickPoolBuffer().buffer` changes. Complete rules:

- Per-frame descriptor sets: update **after** `wait fence` for that frame index, then bind the **current** pool buffer.
- After grow: mark all frames’ descriptors dirty (or update all sets immediately if no command buffer is recording).
- `uploadToBuffer` today uses `immediateSubmit` + wait; that is safe vs compute if it runs before `beginFrame`, but **destroying the old buffer in the same call** races in-flight frames. Retire queue:

```
struct RetiredBuffer { AllocatedBuffer buf; uint64_t lastFrameUsed; };
// destroy when frameIndex has wrapped past lastFrameUsed + kFramesInFlight
```

### 14.7 Allocator completeness

| Event | CPU | GPU |
|-------|-----|-----|
| `alloc` with free slot | pop `freeList`, fill page, dirty | dirty upload |
| `alloc` at capacity | double CPU+GPU (14.4), then fill | copy old + dirty new |
| `free` | zero page, push `freeList`, dirty | dirty upload (optional; INVALID coarse is enough for tracing) |
| `rebuildVoxels` | clear lists, reserve min, build objects | create/reuse high-water buffer, **full** upload |
| compact (optional) | pack live pages, rewrite `brickPage` ids | full upload; only if fragmentation hurts |

Do **not** compact on every edit (page ids would all change). Compact only on rebuild or a rare defrag.

### 14.8 Endgame layers (not required to fix the bug)

These are the “most complete” industry stacks; adopt only if the sandbox becomes a large world:

1. **Fixed GPU working set + eviction** (GigaVoxels / BrickMap streaming): cap pages in VRAM; miss → request buffer; CPU streams surface bricks. Shader must handle `UNLOADED` (fallback to coarse solid / LOD).
2. **3D brick atlas** (GVDB / volume brick cache): slot = texel brick; mapping atlas; hardware filter. Overkill for 8³ bitmasks.
3. **OS virtual memory over a huge table** (HashDAG): pre-size a table for all future edits, commit physical pages on demand. Best if page ids must never move and the table is huge; not needed for a few thousand 64-byte pages.
4. **LOD mips in the pool** (BrickMap 8³/2³/1³, VoxelHex): when a page is evicted, keep a cheap mip so the object does not vanish.

For this repo after the bugfix: **14.3 + 14.4 + 14.5 + 14.6** is the complete *correct* pool. 14.8 is the complete *scalable* pool.

### 14.9 Chosen grow strategy: append-only slabs (implemented)

`pageId = slabIndex * 1024 + local`. Each slab is a fixed GPU SSBO. A full slab never moves. A new slab is created only when 1024 pages are exhausted. Shader `brickSlabs[8]`; unused slots bind a dummy buffer. Dirty-page uploads stay within one slab.

### 14.10 Implementation order (remaining)

1. ~~Append-only slabs (no copy of old GPU data).~~
2. (Later) eviction / UNLOADED / mip fallback if page count becomes a real budget.

### 14.10 One-line

**Sparse allocation on the CPU; a reserved, geometrically growing, fully initialized SSBO on the GPU; dirty uploads only when the buffer object stays the same; never drop live pages on resize.**

---

## 13. One-line summary

**Nested sparse bricks = same local DDA and rigid objects, but `8³` (and later extra levels) live in a page pool; air and uniform solids do not rent a page.**
