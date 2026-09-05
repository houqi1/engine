# Voxel Performance Report

## Status

Measured on 2026-09-05, RTX 4060 Laptop GPU, NVIDIA 560.94, Windows/WDDM.
The short close-hut run exceeded 500 GPU-equivalent FPS on average, without
reducing rendering quality. Sustained 500 FPS is **not fully accepted**:
extended testing exposed thermal throttling, and some views/runs exceed 2 ms.
The user chose to retain the changes and summarize the results before addressing
cooling and continuing sustained acceptance testing.

The original user-reported 100+ display FPS was not used as a measured baseline.
The baseline below uses the original shader with the new reproducible harness
and synchronization/unchanged-transform fixes already applied. It is not an
untouched-executable end-to-end FPS comparison.

## Measurement

- Exact 2560x1440 framebuffer and dispatch, no dynamic resolution or upscaling.
- Piratebuilding.obj, import grid 64, padding 1, conservative voxelization.
- Coarse voxel size 0.35 m, full 8x8x8 micro and 2x2x2 fine occupancy.
- Full shaded mode, AO strength 1, AO power 1/3, original light and HDR sky.
- 1,366,097 occupied fine voxels including the original scene geometry; 2,575
  allocated brick pages. Color changes do not reduce occupancy.
- Normal UI rendering remains included. Camera, editing, and simulation are
  frozen only for repeatability; these runs do not claim a physics budget.
- Raw GPU timestamps, no EMA in CSV, no removal of measured outliers.
- Total includes beam, main compute, blit, and UI. It excludes CPU work,
  presentation latency, scene import, and post-measurement image readback.
- Camera format: `x y z yaw pitch`, angles in radians, vertical FOV 60 degrees.

### Saved Runs

All times are GPU total milliseconds. FPS here is `1000 / mean_ms`, not display
FPS and not the average of reciprocals. These are successive engineering
checkpoints, not simultaneous controlled-clock measurements.

| Run / Local CSV in build/ | Pose | Frames | Mean | Median | P95 | GPU FPS |
|---|---|---:|---:|---:|---:|---:|
| Original gray, close-original.csv | close | 500 | 4.4621 | 4.4560 | 4.4681 | 224.1 |
| Optimized color, accepted-close-color.csv | close | 2000 | 1.9320 | 1.8986 | 2.1709 | 517.6 |
| Extended color, sustained-front-color.csv | front | 8000 | 2.5176 | 2.3917 | 3.6810 | 397.2 |
| Latest AO page reuse, ao-reuse-close.csv | close | 2000 | 2.1123 | 2.0830 | 2.4943 | 473.4 |

- Close: `4 3 5 0.67474094 0` (the benchmark default).
- Front: `0 2.6 5 0 0`.
- The original gray beam-off reference was 4.8177 ms median, in close-exact.csv.
- A prior optimized gray checkpoint was 1.8581 ms median / 1.9522 ms mean, in
  final-close-gray.csv. This is roughly a 56% mean GPU-time reduction relative
  to the original gray run, but thermal conditions were not locked.
- Earlier secondary color checkpoints reached 1.9151 ms median from the front
  and 1.8103 ms from the left (`-4 3 5 -0.67474094 0`). Later sustained runs were
  slower. Do not use the earlier values to promise all-view steady 500 FPS.

### Thermal Evidence

During the extended front run, concurrent `nvidia-smi` telemetry recorded
86-88 C, graphics clocks falling from 2550 MHz to approximately 2055-2295 MHz,
and clock-event reason `0x20` (software thermal slowdown). Memory stayed at
8000 MHz. No clocks, power limits, fan settings, or driver settings were changed.

The shorter 2000-frame 517.6-FPS result still contains 400 frames above 2 ms.
Its P95 is therefore not a 500-FPS guarantee. The final AO-reuse run occurred
after the thermal problem was identified; comparing its wall-clock GPU time
directly with an earlier cooler run does not isolate that change's speedup.

## Retained Changes

1. **Conservative beam bounds.** The old five sampled rays and 5.6-m margin were
   replaced by certification of the entire 8x8 pixel-center frustum's empty
   prefix, first against coarse occupancy and then micro occupancy. Work-budget
   exhaustion retains the last proven prefix, never an assumed miss. The main
   rays still traverse and shade every required fine voxel.
2. **Stable traversal origin.** Beam skipping advances the initial microcell
   without moving its original local ray origin. Coarse skip distances use a
   consistent origin. This avoids most origin-dependent hit/UV differences.
3. **Reuse occupancy.** Micro traversal caches its current octant words; fine
   traversal loads the parent fine byte once instead of rereading it at each
   step. Existing brick data, allocation, and edit upload formats are retained.
4. **Exact AO gather.** The same eight logical neighbor samples are recovered
   from four microcell bytes. The already resolved hit page is reused for
   same-coarse neighbors. AO equations, interpolation, power, and strength are
   unchanged; multi-axis tie cases retain their general path.
5. **Arithmetic direction masks.** Replaced the 512-entry local constant table
   with equivalent bitplane operations, exhaustively tested against Cartesian
   reach masks. This removes function-local array materialization in SPIR-V;
   no unsupported claim about actual SASS spills is needed.
6. **Native indexing when supported.** Optional nonuniform storage-buffer
   indexing removes the eight-way slab branch at each read. Unsupported devices
   retain the existing portable branch implementation.
7. **Full-quality specializations.** Shaded, fine-enabled, one-object pipelines
   strip modes that are not active. Sampled-color and non-color variants are
   selected from actual GPU object flags. Generic/multi-object paths remain.
8. **Avoid static uploads.** Unchanged object records no longer create staging
   buffers, submit a transfer, and drain the queue every frame. Changed shared
   buffers retain synchronization. This is a throughput/CPU saving, not a
   claimed shader millisecond saving.

Also fixed swapchain acquire wait stages, shared output/beam reuse dependencies,
deferred UI resource rebuilding, descriptor-update lifetime, non-coherent upload
flushes/readback invalidation, fine-DDA zero-direction and tied-time errors, and
the coarse shader's missing initial entry normal. The Steps diagnostic now
accumulates earlier nested work rather than overwriting it.

## Negative Optimizations And Traps

### Confirmed Waste In The Previous Implementation

- **Uploading unchanged transforms every frame:** unnecessary staging,
  submission, and fence waits serialized otherwise static frames. Removed.
- **Beam dispatch with no consumer:** coarse/slim and sky-only diagnostic
  kernels did not read beam depth, but the renderer still dispatched it. It now
  runs only when the selected path consumes it.
- **One large full-quality/debug kernel:** runtime alternatives kept unnecessary
  state and instructions alive. One compiler-statistics checkpoint reported
  128 registers for the generic kernel versus 96 for shaded variants. These
  are specific observed pipeline variants, not universal hardware constants.
- **Repeated fine/micro occupancy resolution:** replaced by parent-byte and
  octant-word reuse, and exact four-byte AO gathering.

### Not Proven To Be Negative

- The old beam was **not uniformly slower**. At the original close pose it
  reduced total time from 4.8177 to 4.4560 ms. Its real problems were an unsafe
  five-ray bound and a large margin that gave up useful nearby empty space.
  The replacement sometimes costs more in the prepass but saves much more in
  the main shader. Compare total, not prepass time alone.
- Direction masks were not simply disabled: in the initial medium-view test,
  beam-off with direction skipping was 4.3331 ms, versus 5.1395 ms with both
  direction switches off. The coarse switch also disables occupancy tile
  skipping; the misleading UI description was corrected.
- The color/occupancy slab layout is still the original interleaved format.
  No unmeasured memory-layout benefit is claimed.
- Diagnostic stages change work and sometimes kernels. They are not
  full-quality performance results or an exact additive cost decomposition.

### Rejected Experiments

- 8x4 workgroups did not help; retained 8x8 groups.
- A lazy atomic AO cache increased time and added 128 MiB for this scene. It
  and its allocation/invalidation machinery were removed.
- Planar GPU slab repacking had no isolated demonstrated benefit and doubled
  dirty-page synchronous uploads. Removed, preserving one upload per page.
- Unified 16-cubed fine traversal produced more numerical boundary differences
  without a necessary performance benefit. Removed; retained the nested DDA.

## Quality And Verification

No resolution, voxel count, fine detail, color precision, sky, AO sample
semantics, or light settings were reduced. There is no temporal frame reuse,
frame generation, lower-detail performance mode, or baked lighting shortcut.

The latest AO page-reuse capture is byte-identical to the preceding optimized
color capture. Against the same-pose generic beam-off color reference, the
optimized capture differs at **one of 3,686,400 pixels**, on a ground boundary;
maximum channel difference is 62 and mean absolute channel difference is
0.000015824 on the 0-255 scale. Earlier origin handling differed at two pixels.
This is not claimed to be bit-for-bit identical to the reference. Boundary
normal/tie arithmetic is sensitive to floating-point traversal history; image
comparison alone is not a proof of the exact GPU event at that pixel.

Verification performed:

- Release build of both voxel and grass targets, plus Debug voxel build.
- Native and portable SPIR-V validated with `spirv-val` for Vulkan 1.3.
- Direction masks: all 512 masks and 4096 box combinations, plus wrapped inputs.
- AO: exhaustive stencil/parity tests, mutation invariants, 318420 known-hit
  byte comparisons and 4920 exposed-face comparisons.
- Beam: 12 CPU-model test groups, including micro occupancy, budgets, solid
  sentinels, partial tiles, transformed bounds, and five-ray counterexamples.
- Fine traversal: 53248 exhaustive and 512 seeded CPU-model cases, including
  zero directions and the tied-hit 0.75-versus-1.5 regression.
- Debug synchronization-validation sample runs for full color and coarse
  fallback: no validation errors reported. Loader warnings concerned duplicate
  third-party Epic/OBS layers. The last AO page-reuse edit was subsequently
  compiled, SPIR-V validated, and image-compared; its selection logic was also
  tested in the CPU model.
- `git diff --check`.

CPU models do not execute GLSL or prove every mutable-scene path. Interactive
editing, resize storms, overlapping moving objects, and other GPUs are not
fully runtime-tested. Animated shared transforms still use synchronous uploads;
physics integration will need a separate per-frame data/queue design.

## Reproduce

Run from the repository root. Build directories must already be configured.
The Windows voxel launchers now use Release; Debug remains available for
validation, not performance acceptance.

```powershell
cmake --build build --config Release --target vulkan_engine_voxel --parallel
& "build/Release/vulkan_engine_voxel.exe" --benchmark --width 2560 --height 1440 --camera 4 3 5 0.67474094 0 --color 1 --warmup 300 --frames 2000 --csv "build/retest.csv" --capture "build/retest.ppm"
python scripts/summarize_voxel_benchmarks.py build/retest.csv
```

Use `--beam 0` and `VE_VOXEL_GENERIC_SHADER=1` for the general reference. Use
`VE_VOXEL_PORTABLE_BRICKS=1` to force portable slab access. Remove the environment
overrides before the optimized measurement. CSV records effective kernel,
backend, and compiler-statistics mode as well as scene/camera settings.

Optional compiler statistics: `VE_VOXEL_PIPELINE_STATS=1`. The feature is queried
and enabled only if supported. The old NVIDIA driver returned an implausible
64-GiB local-memory statistic even for small shaders; do not interpret it as
proof of real spill traffic. Nsight Graphics 2025.5 is installed but requires a
newer driver than 560.94; no Nsight capture is claimed.

```powershell
python scripts/compare_voxel_images.py build/reference-close-color.ppm build/ao-reuse-close.ppm --diff build/retest-diff.png
python -B scripts/test_direction_masks.py
python -B scripts/test_voxel_ao.py
python -B scripts/test_voxel_beam.py
python -B scripts/test_voxel_traversal.py
```

Image comparison requires Pillow. CPU model tests use the Python standard
library. Local raw CSVs/captures remain under the ignored build directory;
they are not committed source assets. Re-run after addressing cooling, with a
long warmup and enough measured frames to reach thermal steady state, before
claiming sustained 500 GPU-equivalent FPS.
