# Object-Order Empty Space Skipping (Occupancy Hull → tMin/tMax → Compute DDA)

Living design note. **Not a KD-tree. Not nested AABBs. Not world-AABB-only.**

Status: **v1 implemented (P0–P5)** — occupancy hull raster → `tMin/tMax/tBack` → compute DDA starts at `tMin`.

Related shipped code:

- `src/render/VoxelRenderer.cpp` — full-screen `vkCmdDispatch` of `voxel_dda.comp` (8×8)
- `shaders/voxel_dda.comp` — `traverseObject`: AABB `tEnter/tExit`, then coarse DDA + 4³ `occMip` + nested 8³ / 2×2×2
- `src/scene/VoxelScene.cpp` — unified world 48³ + spinner 8³; `fillOccMip` already builds 4³ occupancy bits
- `src/gfx/PipelineBuilder` — dynamic rendering (mesh demo); voxel demo has **no** graphics pass today
- `docs/coarse-cell-3d-texture.md` — Phase 1 3D tex did not move FPS; layout is not the limiter

---

## 1. Goal

Keep:

- Three-level DDA (coarse → 8³ → 2×2×2), CPU `cells[]`, pick/brush, occMip skip **inside** the interval
- `voxelSize = 0.35`, world `N = 48`, unified stamp
- Compute as the DDA kernel (do **not** rewrite traversal in a fragment shader for v1)

Change:

- Before DDA, **rasterize an occupancy hull** (proxy mesh of occupied 4³ macros)
- Per pixel store linear **`tMin` / `tMax`** along the camera ray
- Compute starts at `tMin` (or skips the pixel = sky if no hull)

Why: Teardown and GPU volume raycasting treat “back away from a surface → slower” as **image-order rays starting at a loose box**. Object-order skip starts rays at the **occupied envelope**. Sky pixels never enter DDA. Hit pixels skip the empty prefix.

Non-goals (v1):

- Nested AABB / KD-tree / BVH of bricks (ray-order tree walk)
- Rasterizing coarse cubes as the **shaded** surface (would destroy 8³ / 2×2×2 grain)
- Moving DDA into the fragment shader (Teardown’s exact path; later optional)
- Beam / low-res depth prepass (complements this; not this note)
- Phase 2 `N = 64`

---

## 2. What the literature actually does

### 2.1 Object-order empty space skipping (Hadwiger et al., SIGGRAPH Course 2009)

Rasterize **boundary faces between active and inactive bricks**, not the volume AABB. Depth (or start/end) clamps each ray to the visible occupied region. The ray-casting program itself can stay unchanged. Overhead is “negligible compared with no ESS.” It does **not** skip every empty segment **between** two occupied regions along one ray (their figure’s ray r3); image-order skip (our `occMip`) still handles the interior.

### 2.2 Haferburg’s OpenGL 4 recipe (the concrete mesh)

On a coarse grid (example: 8³ voxels per cell):

1. Cell **active** iff it contains anything visible.
2. For each pair of 6-connected neighbors: emit a **quad** iff one is active and the other is not (the occupancy **surface**).
3. Pass A: rasterize the mesh, store **minimum and maximum** depth (dual-peel style: blend `MIN` of `z` and `-z`).
4. Pass B: only the closest front (or near plane if the camera is inside), march `[tMin, tMax]`.
5. Camera inside the volume: if the closest back-face is as close as the closest face, start at the **near plane**.

No geometry shader required if the CPU (or a tiny compute) emits the quads. At our size that is the right call (MoltenVK GS is a poor fit).

### 2.3 Teardown (per-object OBB, not a world hull)

Each object: 12 triangles (OBB **back faces** so the camera can enter the box), fragment DDA from box entry, 2×/4× mips as an in-object octree. Front-to-back objects + a depth test so occluded boxes skip DDA.

That win assumes **tight per-object boxes**. Our GPU list is **one 16.8 m world + optional 8³ spinner**. Rasterizing the **world AABB** is almost the same `tEnter` we already compute in `traverseObject`. The Hadwiger/Haferburg **occupancy surface** is what maps onto the unified world.

### 2.4 Object-order vs image-order

| Kind | When | This engine |
|------|------|-------------|
| Object-order | Raster hull → clamp ray interval | **this note** |
| Image-order | During the walk, skip empty blocks | already: 4³ `occMip` |

They **complement**. Object-order does not replace nested DDA or occMip.

---

## 3. Mapping onto this repo

```
Today (image-order only):
  dispatch full screen
  every pixel: intersect world AABB → DDA (+ occMip) → shade or sky

v1 (object-order setup + same DDA):
  rasterize occupancy hull → tMin/tMax image
  dispatch full screen (still)
    no hull → sky, 0 DDA
    else traverseObject starting at tMin, stop by tMax / hit / maxSteps
```

World AABB rasterization is **P0 debug only**, not the accept target.

Proxy grain = **occMip cell** (`S = 4` coarse cells = 1.4 m). Conservative: a 0.35 m wall still produces a macro face. DDA then finds the real 8³ / 2×2×2 hit. Visual grain unchanged.

---

## 4. Occupancy hull (CPU, v1)

`kOccMipRes = 4`, world `N = 48` → macro grid **12³**. At most a few hundred surface quads (ground is a slab, hut is a shell). Rebuild on edit/stamp/rebuild — cheaper than a brick flush.

Active macro: existing occMip bit `1` (any coarse cell in that 4³ has `material != 0`).

Face emission (Haferburg):

```
for each macro m in [0, macroN)³:
  if !active(m): continue
  for each of 6 directions d:
    n = m + d
    if n out of bounds OR !active(n):
      emit quad on the face between m and n
```

Out-of-bounds neighbor **does** emit (outer surface of the occupancy, including the ground sides).

Vertex positions: **world space** corners of the macro AABB, using the same `objectToWorld` as DDA (`VoxelObject::objectToWorld`). Spinner is a separate 8³ OBB (12 triangles), not merged into the world hull.

Conservative option (v1 default **on**): dilate active macros by 1 before emitting faces, so a thin wall on a macro boundary cannot put `tMin` already past the surface. Extra empty DDA inside occMip is cheap.

Index buffer: two triangles per quad. `vec3` positions only. Upload with existing `uploadToBuffer`.

Do **not** greedy-mesh for shading. Do **not** emit one cube per occupied coarse cell (ground overdraw).

---

## 5. Interval image (graphics, dynamic rendering)

The mesh demo already uses `vkCmdBeginRendering` and reverse-Z. Voxel demo adds a **small** graphics pass, not a copy of `Renderer.cpp`.

Store **linear ray parameter `t`** (world meters along the camera forward ray), **not** `gl_FragCoord.z`. Reverse-Z would make min/max depth easy to get wrong.

Two color attachments, same extent as the DDA output:

| Attachment | Format | Blend | Clear | Meaning |
|------------|--------|-------|-------|---------|
| 0 `tMin` | `R32_SFLOAT` | `VK_BLEND_OP_MIN` | `+inf` | closest hull hit |
| 1 `tMax` | `R32_SFLOAT` | `VK_BLEND_OP_MAX` | `0` | farthest hull hit |

No depth test required for this pass (blend does the min/max). Cull: **none** (need both front and back faces, like Teardown/Haferburg).

Vertex shader: `worldPos`, clip `viewProj`.

Fragment shader:

```
vec3 rd = normalize(worldPos - cameraPos);
float t = dot(worldPos - cameraPos, rd);  // or distance
outMin = t;
outMax = t;
```

`PipelineBuilder::setColorBlend` is alpha-only today — **extend** it (or a dedicated builder path) for `MIN`/`MAX` on two attachments. Do not pretend alpha blend is min/max.

After the pass, transition both images to `SHADER_READ_ONLY` for compute.

Debug: visualize `tMin` as grayscale; empty pixels stay `+inf` (sky).

---

## 6. Compute DDA change (`voxel_dda.comp`)

New binding (e.g. **8**): `sampler2D tMinMap`, `sampler2D tMaxMap` — or one `rg32f`. Keep 0–7 as they are if an extra binding is easier than packing.

In `main`, **before** `traverseObject`:

```
float t0 = texelFetch(tMinMap, pixel, 0).r;
float t1 = texelFetch(tMaxMap, pixel, 0).r;
if (!(t0 < t1) || t0 >= largeSentinel) {
    // no hull: sky / steps=0
    imageStore(sky);
    return;
}
```

Pass `t0` into `traverseObject` as the start (replace `tEnter` from the object AABB when the AABB entry is **in front of** `t0`; still intersect the object box so spinner/world stay correct):

```
tEnter = max(tEnter, t0 / voxelSize); // convert meters ↔ voxel units carefully
tExit  = min(tExit,  t1 / voxelSize);
if (tEnter > tExit) miss;
pos = ro + rd * (tEnter + eps);
```

Unit convention: hull `t` is **world meters** along a **normalized** camera ray (same `dir` as `main`). Object DDA uses **voxel units** of `Ol / voxelSize`. Convert with `t_vox = t_world / o.voxelSize` only after transforming into object space, **or** store hull `t` in world space and convert per object using the rigid transform. Simplest v1: store world-space hit **position** is overkill; store camera-ray `t` in meters, and in `traverseObject` convert:

```
vec3 worldStart = cameraPos + dir * t0;
vec3 localStart = (worldToObject * vec4(worldStart,1)).xyz / voxelSize;
```

then begin DDA at `localStart` (clamp into the object grid). `t1` similarly as a world-space end clamp (`dot(hitWorld - camera, dir) <= t1`).

World object: hull matches occupancy. Spinner: no world-hull coverage; either

- rasterize spinner OBB into the **same** min/max images (second draw), or
- keep spinner on the old AABB path (small 8³; cheap)

v1: **second draw** of spinner cube into the same tMin/tMax (12 triangles).

`MODE_STEPS` should drop when hull-skipped (those pixels were the expensive empty walks).

AO / nested: unchanged after a hit.

---

## 7. Camera inside the hull

Hadwiger/Haferburg: near plane can cut the front faces, leaving a hole.

v1 rule (Haferburg):

- Rasterize **front and back** (cull none).
- Per pixel, if the closest surface is a **back face** (or `tMin` is behind the camera), set `tMin = 0` (start at the camera).
- Implement by writing a third channel `closestBackT` with `MIN` blend only for `!gl_FrontFacing`, or by a tiny extra pass.
- If the near plane intersects the proxy, leftover faces are the **wrong** start (often the far wall). Hadwiger/Scharsach: start those rays at the near plane. `cameraInsideWorldAabb()` forces `tMin = 0`. Hull fragments behind the camera are discarded so interpolated `t` cannot collapse to 0 (black sparkles).

Test: walk into the hut; walls around the camera must not vanish.

---

## 8. When to rebuild the hull

| Event | Action |
|-------|--------|
| `rebuildVoxels` | Full extract + upload |
| Import stamp / remove import | Full extract + upload |
| Brush `flushObject` on world | Full extract (12³ is tiny; no dirty-box v1) |
| Spinner enable/transform | OBB matrix only (12 verts transformed in VS) |
| Camera move | **No** CPU rebuild; only `viewProj` |

---

## 9. Frame order (`VoxelRenderer::draw`)

```
beginFrame
upload transforms
1. graphics: hull → tMin/tMax  (dynamic rendering, two color attachments)
2. barrier: color write → compute sample
3. compute DDA (existing dispatch size)
4. blit to swapchain + ImGui  (unchanged)
```

Sky: compute writes sky for `tMin` sentinel, **or** clear output to sky then DDA only where hull exists. v1: keep full dispatch, early-out in shader (simplest). Later: tile list / subgroup skip — not required to accept.

---

## 10. Implementation slices (do in this order)

### P0 — Graphics plumbing (world AABB cube, not the accept hull)

1. Two `R32_SFLOAT` images, swapchain extent, recreate on resize.
2. Extend `PipelineBuilder` (or a local pipeline) for dual-attachment `MIN`/`MAX` blend, no cull, no depth.
3. Shaders `hull.vert` / `hull.frag`.
4. Draw a unit cube transformed by **world** `objectToWorld` (16.8 m box).
5. ImGui: visualize `tMin`. Confirm sky outside the box is `+inf`, box pixels have `tMin < tMax`.
6. **Do not** wire compute yet. Existing DDA still full AABB. This slice only proves MoltenVK min/max blend + dynamic rendering in the voxel demo.

### P1 — Occupancy hull mesh

1. `VoxelScene::rebuildOccupancyHull()` from world `occMip` (or from `cells[]` if simpler).
2. CPU 6-neighbor quads, conservative dilate **on**.
3. `AllocatedBuffer` vertex/index, upload.
4. Draw hull instead of the world cube in P0. `tMin` should hug the ground slab + hut shell, not the 16.8 m cube.
5. Debug: also draw hull wireframe in a later shaded mode if useful.

### P2 — Compute reads the interval

1. Descriptor: two sampled images (or `rg32f`).
2. `voxel_dda.comp`: sentinel → sky; else start at `tMin`.
3. Convert world `t` ↔ object voxel space as in §6.
4. Accept this slice when: **far outside, looking at the hut**, GPU compute ms drops; close to a wall does not regress much; nested carve/AO unchanged.

### P3 — Camera inside

1. Back-face / near-plane rule §7.
2. Walk into the hut and through the ground plane (if possible). No hole at near clip.

### P4 — Spinner

1. Draw spinner OBB (back faces or the same min/max cube) into the **same** tMin/tMax.
2. Toggle “Show Rotating Object”: hull updates, DDA still hits the spinner.

### P5 — Edit / import

1. `flushObject` / stamp / rebuild call `rebuildOccupancyHull`.
2. Place/delete a coarse cell on the hut roof: next frame `tMin` grows/shrinks. No stale hull.

### P6 (optional, not v1)

- Pack `tMin/tMax` into `RG32F` one image.
- Skip compute for sky tiles (indirect dispatch).
- Fragment DDA like Teardown (only if P2 is not enough).
- Beam prepass on top of the hull.
- Dilate off as a checkbox.

---

## 11. File list

| Path | Role |
|------|------|
| `docs/object-order-ess.md` | this file |
| `src/scene/VoxelScene.h/.cpp` | hull mesh CPU, upload, rebuild on edit |
| `src/render/VoxelRenderer.h/.cpp` | graphics pass, tMin/tMax images, descriptors |
| `src/gfx/PipelineBuilder.*` | MIN/MAX dual-attachment blend |
| `shaders/hull.vert` / `hull.frag` | proxy raster |
| `shaders/voxel_dda.comp` | start at `tMin` |
| `CMakeLists.txt` | `hull.vert` `hull.frag` in `VE_SHADER_SOURCES` |

`voxelize_surface.comp` **untouched**. Nested brick pool **untouched**.

---

## 12. Tests / accept

| Case | Expect |
|------|--------|
| Far outside, hut on screen | Compute ms **down** vs current; sky pixels ~0 DDA |
| Close to hut wall | No large regression; grain identical |
| Steps mode | Sky dark (few steps); hut/ground similar to today |
| Nested on, carve 2×2×2 | Same as today |
| Walk inside hut | No near-plane hole (P3) |
| Spinner on/off | Spinner visible, cheap |
| Stamp / brush | Hull matches occupancy next frame |
| Skip Trace | Still works (ignore hull) |
| World AABB P0 only | `tMin` is the big cube — **not** the accept look |

If far-outside does not improve after P2, stop. Debug: is `tMin` actually at the hut (visualize), or still at the world box (forgot to switch from P0 cube)? Is compute still using AABB `tEnter` (forgot to `max` with hull)?

---

## 13. Risks

| Risk | Mitigation |
|------|------------|
| MoltenVK `MIN`/`MAX` blend on two attachments | P0 proves it; fallback: two sequential depth-like passes |
| Reverse-Z vs min depth | Store linear `t`, never `gl_FragCoord.z` as the interval |
| Thin wall, `tMin` past surface | Conservative dilate 1 macro |
| Empty between hut and ground on one ray | Expected (Hadwiger r3); occMip still skips inside `[tMin,tMax]` |
| Overdraw of per-cell cubes | Surface quads only, never a cube soup |
| Shading the hull | Never; hull is proxy only |
| Raster cost > DDA save | 12³ surface is tiny; if not, we failed P1 tightness |
| Descriptor binding 8 | Add CIS pool slots; resize recreates tMin/tMax |

---

## 14. Defaults

| Knob | v1 |
|------|----|
| Hull cell | occMip 4³ (12³ macros at N=48) |
| Conservative dilate | **on** |
| DDA kernel | compute, unchanged nested |
| World AABB proxy | P0 only |
| Spinner | 8³ OBB into same interval |
| Rebuild | whole hull on any world edit |

---

## 15. References

- Hadwiger, Ljung, Rezk-Salama, Bavoil, *Advanced Illumination Techniques for GPU-Based Volume Raycasting*, SIGGRAPH Course 2009, §2.2 Object-Order Empty Space Skipping
- Haferburg, *Object Order Empty Space Skipping in OpenGL 4* — active/inactive quads, min/max depth, camera-in-volume
- Gustafsson / Tuxedo Labs, Teardown: OBB back-face raster + fragment DDA + 2×/4× mips ([frame breakdown](https://juandiegomontoya.github.io/teardown_breakdown.html), [Acko](https://acko.net/blog/teardown-frame-teardown/))
- Deakin & Knackstedt, *Accelerated Volume Rendering with Chebyshev Distance Maps* — object-order vs image-order
- Lund 2023 voxel ray marcher (M1): back-face cube like Teardown; mips help **inside** the box, not instead of a tight start

In-engine: `docs/nested-brick-voxels.md`, `docs/coarse-cell-3d-texture.md`
