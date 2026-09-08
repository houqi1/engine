# Multi-Object Box + Fragment DDA (Main Visibility)

Status: **Phase 2 implemented** — Box Fragment DDA is default; optional batched `DepthSnapshot` occlusion (ImGui: "Snapshot batches" 0/2/4/8). Compute DDA remains as reference.

Supersedes `docs/object-order-ess.md` as the multi-object visibility strategy.
Occupancy-hull → compute interval remains historical; do not extend it for scale.

## Pipeline

```
Shared CoarsePool + BrickPool
  → occupied AABB update
  → frustum cull + near→far buckets
  → instanced unit-cube (back faces)
  → Fragment: depth-snapshot reject + bounded DDA
  → write real depth + HitBuffer
  → optional snapshot copy between batches
  → fullscreen shade / local AO
  → blit + UI
```

## Phase checklist

| Phase | Goal | Accept |
|-------|------|--------|
| 0 | CoarsePool SSBO; `voxelOffset` = pool offset; per-frame object buffers; no transform `waitIdle` | ≥30 CPU shapes, transforms OK |
| 1 | Back-face boxes + Fragment bounded DDA + real depth + deferred shade; compute = reference | **Done** — toggle off for compute reference |
| 2 | Depth snapshot batches (0/2/4/8) | **Done** — 0 = single-pass baseline; N>0 copies `VisibilityDepth`→`DepthSnapshot` between near→far batches; shader discard/shorten is opt-only (real `gl_FragDepth` + depth test remain authoritative) |
| 3 | Scatter spawn (100–1000) + metrics hooks; further cull/proxy/Hi-Z as bottlenecks appear | **Partial** — ImGui "Spawn scatter" / "Clear scatter"; cost target ∝ coverage × layers × voxels |

## First-version choices

- Visibility: back-face OBB raster + Fragment DDA
- Data: shared coarse SSBO + existing brick pool
- Submit: CPU cull/bucket + instanced draw
- Depth: reverse-Z, real hit `gl_FragDepth`
- Occlusion: batched depth snapshot (optional)
- Shading: HitBuffer then fullscreen shade
- Beam: off for box path
- No feedback-loop / interlock / local-read baseline

## Resources

- `VisibilityDepth` (D32, reverse-Z; `DEPTH_ATTACHMENT | SAMPLED | TRANSFER_SRC`)
- `DepthSnapshot` (D32 sampled copy; `TRANSFER_DST | SAMPLED | TRANSFER_SRC`; gfx set binding 9)
- `HitBuffer` (RGBA32F storing `uintBitsToFloat` packs — MoltenVK-friendly)
- `ColorOutput` (existing RGBA8 path)

## Phase 2 notes

- Correctness does **not** depend on snapshots: real `gl_FragDepth` + reverse-Z depth test remain.
- `snapshotBatchCount == 0`: single BeginRendering (baseline). `>0`: clear hit+depth once; LOAD across batches; specials drawn in batch 0; after each non-final batch, barrier → `vkCmdCopyImage` depth aspect → snapshot → shader-read.
- Snapshot cleared to 0 (far) at frame start so batch 0 sees no occluder while `depthSnapshotEnabled=1`.
- Margin: `depthOcclusionMargin * max(tEnter, 1)` (default 0.005 m); prefer missing a cull over holes.

## Files

- `VoxelScene.*` — CoarsePool, FIF objects, bounds
- `VoxelRenderer.*` — graphics visibility + shade; compute reference
- `shaders/voxel_trace.glsl` — shared bounded DDA
- `shaders/proxy_box.vert` / `visibility.frag` / `voxel_shade.frag`
- `fullscreen.vert` — shade
- Compute `voxel_dda.comp` — reference / debug
