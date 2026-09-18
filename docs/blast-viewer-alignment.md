# NVIDIA Asset Viewer alignment

## Reference and scope

Audited the NVIDIA GameWorks Blast SampleAssetViewer at commit
[`9f4fc41dc5d857e3c7c3500fc71953e54d780a39`](https://github.com/NVIDIAGameWorks/Blast/tree/9f4fc41dc5d857e3c7c3500fc71953e54d780a39).
The engine retains its pinned Blast 5.0.6 Low Level, shaders and ExtStress at
`7ef568f5b557a6dad9023ecebc43cb809270e035`, and its existing voxel rigid-body solver.
Alignment means the application damage/routing/split logic, not identical PhysX
trajectories or an identical authored asset. NVIDIA Viewer meshes can have
irregular chunks and subsupport hierarchies; this asset is a flat voxel support
graph. Chunk counts and fracture appearance are therefore not equivalence tests.

Primary reference files:

- [ExtImpactDamageManager](https://github.com/NVIDIAGameWorks/Blast/blob/9f4fc41dc5d857e3c7c3500fc71953e54d780a39/sdk/extensions/physx/source/physics/NvBlastExtImpactDamageManager.cpp): contact reduction, shader selection, thresholds, damage radii and family collision filter.
- [ExtImpactSettings](https://github.com/NVIDIAGameWorks/Blast/blob/9f4fc41dc5d857e3c7c3500fc71953e54d780a39/sdk/extensions/physx/include/NvBlastExtImpactDamageManager.h): default values.
- [BlastController](https://github.com/NVIDIAGameWorks/Blast/blob/9f4fc41dc5d857e3c7c3500fc71953e54d780a39/samples/SampleBase/blast/BlastController.cpp): optional impact-to-stress callback, factor 0.01, shader fallback and update order.
- [ExtPxAssetImpl](https://github.com/NVIDIAGameWorks/Blast/blob/9f4fc41dc5d857e3c7c3500fc71953e54d780a39/sdk/extensions/physx/source/physics/NvBlastExtPxAssetImpl.cpp): default bond/chunk health 1.
- [ExtPxStressSolver](https://github.com/NVIDIAGameWorks/Blast/blob/9f4fc41dc5d857e3c7c3500fc71953e54d780a39/sdk/extensions/physx/source/physics/NvBlastExtPxStressSolverImpl.cpp): gravity on anchored bodies, centrifugal loading on free bodies, SDK-generated fracture commands.
- [ExtPxActorImpl](https://github.com/NVIDIAGameWorks/Blast/blob/9f4fc41dc5d857e3c7c3500fc71953e54d780a39/sdk/extensions/physx/source/physics/NvBlastExtPxActorImpl.cpp): suppress contact notifications for terminal leaf actors.

## Audit and corrections

| Area | Result |
| --- | --- |
| Defaults | Restored Shear=true, self-collision=false, hardness=10, radius=2, thresholds=0.1/1, falloff=2. Impact enabled and stress routing disabled. |
| Impact input | Reduced mass times normal relative contact velocity, magnitude-squared cutoff 1, mean of accepted contact points. Engine snapshots pre-solve contact velocities because its inelastic solver removes approach velocity. Unconstrained separated speculative candidates are not contact reports. |
| Contact grouping | Replaced actor-wide/tick-wide averaging with exact actor generations, support-node collision subshapes and substep groups. Reversed pair order is canonicalized. No hash collisions can merge unrelated pairs. |
| Normalization | Force magnitude / hardness, then material normalization, minimum threshold rejection and maximum threshold clamp. No extra engine damage cap. |
| Damage program | Uses the unmodified SDK Shear or ImpactSpread shaders. Radii are normalized damage times max radius, then the clamped falloff factor. Removed the silent Euclidean fallback for a missing accelerator. |
| Health units | Viewer starts bonds at 1. Blast 5 ExtStress treats health as remaining area, so shaders' fractional bond damage is multiplied by INITIAL bond area before application. Using current health would incorrectly make successive identical hits decay. Chunk health remains 1 and SDK chunk commands remain intact. |
| Stress routing | For multi-node actors the optional callback adds force times the configured factor and bypasses the shader; it is not an extra damage pass. Zero factor is valid. Shear UI is disabled while this override is active. |
| Stress failure | Uses SDK-generated fracture commands, including partial damage, not a custom one-bond-per-frame or full-health threshold rule. Static gravity and free-body centrifugal loading follow the wrapper pattern; material settings use the retained Blast 5 API. |
| Split lifecycle | Destroy/create notifications include recycled actor pointers. Every nonempty island gets a physical body; no 64-voxel filter or arbitrary 128-actor extraction cutoff. Shapes and actor bindings are committed before the next fixed tick, including multiple ticks per render frame. |
| Terminal actors | Contact damage is suppressed when either Blast actor cannot fracture further, matching the Viewer leaf notification filter. |

The UI exposes Viewer material health/thresholds and impact hardness, radii,
thresholds, self collision and stress factor. The four-column height remains
adjustable from 1.0 to 9.2 m, with a 4.0 m default.

## Validation

Release build and all 13 Blast test executables passed (P0-P4, graph regression,
E0-E3, probe export, frame and Viewer alignment). The real four-column scene at
4 m with Shear produced the same 71 actors at 30 and 60 render Hz, with a distinct
physical object for every resulting actor. ImpactSpread also passed the real
landing regression at both rates (97 actors at 60 Hz, 98 at 30 Hz); exact
render-rate-independent trajectories are not established for this backend.
The stress override at factor 0.01 passed actor/object
consistency with no landing-induced split, consistent with its different route.

`blast_viewer_alignment_tests` constructs an independent family with unit bond
health, runs the NVIDIA shaders and Low Level fracture application, and compares
the engine's remaining health fractions bond by bond for both shaders. This also
covers SDK chunk damage removing adjacent bonds. Other cases cover unequal bond
areas, accumulation against initial area, per-shape and per-substep grouping,
reversed body order, velocity filtering, disabled impact and stress override.

The existing frame and E3 suites cover family self-collision suppression, material
normalization, pre-solve velocity capture, stress commands and repeated landings.
The scene regression verifies unique visible physical objects after splits:

```powershell
cmake --build build --config Release --target vulkan_engine_voxel blast_viewer_alignment_tests blast_frame_tests blast_e3_tests --parallel 4
build/Release/blast_viewer_alignment_tests.exe
build/Release/blast_frame_tests.exe
build/Release/blast_e3_tests.exe
build/Release/vulkan_engine_voxel.exe --frame-fail --frame-height 4 --frame-impact shear --frame-render-hz 60
```

Use `--frame-impact spread` for the alternate Viewer shader or `stress` for the
override, and `--frame-render-hz 30` to exercise two physics ticks per frame.
The stress override at factor 0.01 is not asserted to shatter this asset: that is
material/input dependent in Viewer too. Artificially forcing a target number of
large pieces would be a new fracture model, not Viewer alignment.
