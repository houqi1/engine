# Verified stress convergence — 2026-09-28

Implemented the correction investigated in `stress-convergence-diagnosis.md`.

## Behavior

- Default normal-equation tolerance is now `1e-6`, exposed through
  `ExtStressSolverSettings`, `StructureWorld::setSolverAccuracy` and the debug
  panel's **Solver accuracy** controls. The independent equilibrium tolerance is
  `1e-4`, with an absolute floor of `1e-6` in the solver's scaled coordinates.
- Each connected component is checked independently. For free components, remove
  all six mass/inertia-weighted rigid-motion modes from the RHS. Anchored
  components retain their loads; zero inverse-mass rows cannot carry residual.
  The projection follows the actual SDK coupling matrix's angular convention.
- Before reporting convergence, independently accumulate `b - B*J` in double
  precision and check the projected residual. If the recursive float residual
  collapses but this check fails, replace the recursive residual and restart its
  search direction. Ordinary verification rejection continues CG without
  repeatedly throwing away its directions.
- Report **converged (verified)**, **iteration budget reached; continuing**,
  **numerical failure**, or **not solved**. A frame's work cap is separate from
  its accuracy target. Non-finite/broken numerical iterations cannot produce
  non-finite bond loads for fracture.
- Retain Krylov state only when loads, topology, masses and accuracy settings
  permit continuation. Cache the scaled iterate to avoid an unscale/rescale
  round trip corrupting that state. Changed loads and rebuilt graphs restart it.
- P6 is applied reproducibly during CMake configuration. The preceding P5 beam
  patch's clean-source match was corrected as well.

## Validation

Release application rebuilt: `build/Release/vulkan_engine_voxel.exe`.

`blast_convergence_tests` passes with:

- 2457.6 kg eccentric roof on a 20 cm square support, agg=2, 200 iterations per
  tick. At the middle cut, vertical force is 24202.85 N versus an expected
  24203.23 N (roof plus upper half-column); bending components are approximately
  31463.8 N·m versus a roof-only estimate of 31341.8 N·m. The small eccentric
  column mass and free-body acceleration account for the simplified reference's
  approximation. All compared errors are below the 1% acceptance threshold.
- Maximum compression including bending is approximately 34.0 MPa, and 11
  fracture candidates are generated. The diagnostic does not apply those
  candidates, so the comparison geometry remains unchanged.
- The same cut also passes with the old loose normal tolerance `1e-3`; the
  independent residual check still enforces accuracy.
- One-iteration budget is reported as unfinished, sleep retains equilibrium,
  halving the applied reactions halves the internal load after smoothing,
  free-fall/rigid rotation produce no spurious load, anchored gravity remains,
  breaking an anchored bond rebuilds the projection, and NaN input is rejected.

Existing `blast_e5_tests` and `blast_p4_tests` pass, covering anchors, multi-instance
loads, occupancy rebuilds and fracture ownership. Fresh pinned upstream files
successfully replay P1/P3/P4/P5/P6; repeated P6 application is unchanged.

Real hut checks:

- `--dig-converge`: imports the actual 39564-fine hut as a free body (9028 nodes,
  no world bonds), removes 159 fines, and solves the rebuilt 8996-node graph under
  the existing 8 ms budget. Verified convergence first occurs at tick 200 (3.33 s
  simulated); at tick 240 the equilibrium residual is `2.34e-6`. Full output is
  `stress-convergence-hut.txt`. Timing and first convergence tick can vary with
  the adaptive iteration budget and machine load.
- `--import-topple`: passes; removing all but an off-center foot wakes the hut
  and produces approximately 6.87 degrees of tilt. This isolates rigid-body
  motion with stress fracture disabled.

These checks do not reproduce the exact wall cuts from the user's interactive
session. The aggregation limitation (no internal failure bond inside one graph
node), equivalent-square section approximation, and material/joint calibration
remain separate concerns. The verified residual is a numerical acceptance test,
not a guarantee of continuum structural accuracy for every voxel graph.
