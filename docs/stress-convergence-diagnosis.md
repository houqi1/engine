# Stress convergence diagnosis — 2026-09-28

## Confirmed failure

The current stress solver can report convergence while severely underestimating
the force and bending moment transmitted through a narrow support. This was
reproduced through the engine's `StructureWorld`, using its current Release core
and Blast libraries, current beam stress formula, contact load mapping, and
4 MPa elastic / 8 MPa fatal limits.

The diagnostic collects fracture candidates without applying them, so geometry
and load remain identical between runs. It does not simulate the resulting fall.

## Controlled comparison

Two horizontal voxel plates are connected by one eccentric vertical square
column. Fine size is 0.1 m, density is 600 kg/m³. Four vertical base reactions
sum to the whole body's weight. The graph is free, without world bonds, matching
the separate-object load route. The reactions' resultant is almost aligned with
the whole-body COM (the column's small eccentric mass causes a small mismatch).

Each case runs 300 ticks, with an iteration cap of 1000 per tick. Contact smoothing
and all material settings are identical. A separate diagnostic executable
compiles a copy of ExtStress changing only `params.tolerance` from `0.001f` to
`0.000001f`. The application's SDK source and executable are unchanged.

| Case | Current tolerance 1e-3 | Diagnostic tolerance 1e-6 |
| --- | ---: | ---: |
| 307.2 kg roof, 10 cm column, agg=1: mid-column vertical force | 635.74 N | 3022.83 N |
| Same: bending moment about x | 7.15 N·m | 1963.91 N·m |
| Same: maximum compression including bending | 0.3109 MPa | 16.9843 MPa |
| Same: fracture candidate count | 0 | 5 |
| 2457.6 kg roof, 20 cm column, agg=2: mid-column vertical force | 5085.95 N | 24187.20 N |
| Same: bending moment about x | 114.32 N·m | 31444.96 N·m |
| Same: maximum compression including bending | 0.6218 MPa | 33.9770 MPa |
| Same: fracture candidate count | 0 | 11 |

Both versions report `converged=true` for all rows above. For the 20 cm case,
the roof weight is 24109.06 N and its COM has a 1.3 m lever arm in each horizontal
direction: the corresponding bending moment is approximately 31341.77 N·m per
axis. The upper half-column adds about 94.18 N. The tightened solve closely
matches those values; the current solve transmits only about 21% of the expected
vertical force and 0.36% of the expected bending moment at this cut.

A larger agg=2 case (8224 nodes, 19660.8 kg roof, 40 cm support) changes from
2.1281 MPa maximum compression and no candidates to 103.0092 MPa and 112
candidates. For multi-bond sections, individual bond moments cannot be compared
directly with the whole-section moment: the force-couple contribution must also
be included. The single-bond 20 cm case above is the clearest equilibrium check.

## Source mechanism

`ConjugateGradientImpulseSolver::solve` in
`build/_deps/nvblast-src/blast/source/sdk/extensions/stress/NvBlastExtStressSolver.cpp`
hardcodes `params.tolerance = 0.001f` (line 199 at diagnosis).

`source/shared/stress_solver/math/cgnr.h` sets the stopping threshold to
`tol² * |b|²` (line 107), then stops when `|Bᵀ r|²` is below it (line 128).
This is the residual of the normal equations, not a direct relative error bound
on each connection force or on a cut's force/moment balance. Weakly constrained
global modes can still have large solution error when this criterion passes.
The observed large error is demonstrated by the independent cut balance above.

Warm-started subsequent frames can immediately accept the same insufficient
solution under the same stopping criterion. Raising only the iteration cap does
not force the solver to continue once that criterion passes.

The hardcoded 1e-3 value also exists in the pinned upstream source. However, the
engine has its own graph construction and patches, including non-equalized
masses and the bending formula. This diagnosis establishes a failure in the
current engine stack; it does not establish that every NVIDIA Viewer setup has
the same failure.

## Scope and next correction

This proves a numerical cause of late/missing fracture before material tuning:
the force and moment supplied to the stress formula can already be far too low.
It is not explained solely by connection area or material strength.

The exact interactive hut cut was not captured. These controlled cases use the
same engine integration, including agg=2, but must not be presented as measured
values from the user's particular damaged hut.

There is also an independent aggregation limitation: a short neck entirely
inside one graph node has no internal failure bond. Tightening tolerance does
not fix that case (the height=1 fine / agg=4 fixture remains below threshold).

A production fix should make convergence accuracy appropriate to these graphs,
check force/moment balance on reference cuts, and account for the additional
iteration cost under the application's per-tick budget. The diagnostic value
1e-6 is evidence, not proof that one global tolerance is sufficient for all
graphs. No production setting was changed in this investigation.

## Reproduction artifacts

- `build/hut-neck-analysis/probe.cpp`
- `build/hut-neck-analysis/CMakeLists.txt`
- `build/hut-neck-analysis/balance-report.txt`
- `build/hut-neck-analysis/tight-balance-report.txt`

The default diagnostic links the current Release libraries. The alternate target
adds a separately compiled ExtStress implementation with `DIAG_TOL=0.000001`.
Both diagnostic executables were rebuilt and run for this investigation.
