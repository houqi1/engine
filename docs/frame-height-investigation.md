# Four-column height and landing fragmentation

Historical measurements below precede the Viewer alignment changes. The current
implementation restores Viewer Shear defaults, converts normalized damage to
area-health units, and groups damage by collision subshape and physics substep.
See [Viewer alignment audit](blast-viewer-alignment.md) for the current behavior.

The interactive Column height slider selects 1.0–9.2 m in 0.1 m voxel increments.
The default is now 4.0 m instead of 8.8 m. Press Spawn / reset four columns to apply
the selected height; Cut 3 columns uses the spawned height and preserves the roof.

## Controlled runs

Release executable, `--frame-fail --frame-height H`: 24 intact ticks, cut three
columns, 32 hold ticks, then 240 ticks at fail strength. ImpactSpread enabled;
Shear and Pass impact to stress disabled. Damage parameters were not changed.

| Column height | Final single-node dynamic pieces | Final multi-node dynamic pieces | Largest dynamic piece (graph nodes) |
| --- | ---: | ---: | ---: |
| 2.0 m | 46 | 4 | 22 |
| 4.0 m | 91 | 3 | 4 |
| 8.8 m | 72 | 3 | 4 |

All runs passed the separate-physical-object check after each update. Counts
include detached column pieces, not just the roof, and are final counts after
four seconds rather than the first contact alone. Different column heights also
change column mass, initial stress failure and subsequent contact trajectories;
this is not a pure free-fall energy experiment.

## Why lowering the roof is insufficient

`viewerPairForce` estimates impact from reduced mass times normal relative
velocity. With hardness 10 and material health 100, normalized damage reaches its
cap of 1 at magnitude 1000. A several-tonne roof can reach this cap even at a low
drop height. Once capped, increasing height no longer increases this scalar.

Before the alignment fix, `applyImpactShaderToActor` gave a capped hit a full-damage graph travel radius of
2 m and falloff out to 4 m. ImpactSpread damages every traversed bond, rather than
choosing a few fracture surfaces. Bond initial health is its face area (for
example 0.16 for a full 0.4 m square face), while the shader emits the normalized
damage directly as the health decrement, up to 1. This scale mismatch further
made individual connections easy to sever; the alignment fix now converts damage
using initial bond area.

Thus height affects the trajectory, but the pre-alignment damage model and its scale
were major contributors to pieces reaching the single-node resolution limit.
The result is not evidence that a real frame should shatter this way. Producing
several large pieces is not guaranteed by lowering the height. The current work
preserves NVIDIA shader behavior instead of forcing a chosen fragment count.

These measurements kept the damage parameters unchanged so that height
comparisons remained meaningful. The existing 8.8 m structural regression fixture
is explicit, and raster tests cover 1.0, 2.0, 4.0, 8.8 and 9.2 m, including roof
preservation during the column cut.
