#!/usr/bin/env python3
"""Keep the budgeted stress solve in double precision (after P6)."""
from pathlib import Path
import sys
root=Path(sys.argv[1])
base=root / "source/shared/stress_solver"
header=base / "stress.h"
text=header.read_text(encoding="utf-8")
if "VE_P8_DOUBLE_SOLVE" not in text:
    text=text.replace('#include "StressEquilibrium.h" // VE_P6_CONVERGENCE',
                      '#include "StressEquilibrium.h" // VE_P6_CONVERGENCE\n#include "StressDoubleSolve.h" // VE_P8_DOUBLE_SOLVE')
    text=text.replace('    StressEquilibrium m_equilibrium;', '    StressEquilibrium m_equilibrium;\n    StressDoubleSolve m_doubleSolve;')
    header.write_text(text,encoding="utf-8")
cpp=base / "stress.cpp"
text=cpp.read_text(encoding="utf-8")
if "VE_P8_DOUBLE_SOLVE" not in text:
    start=text.index('    struct CheckContext {')
    end=text.index('    if (!m_equilibrium.finite',start)
    text=text[:start]+'''    // VE_P8_DOUBLE_SOLVE: exported float impulses are not the iterative state.
    int result=m_doubleSolve.solve(impulses,m_B,b,m_equilibrium,error_sq,
        params.tolerance,params.equilibriumTolerance,params.equilibriumAbsoluteTolerance,
        maxIter,params.warmStart,hot);

'''+text[end:]
    text=text.replace('    m_can_resume = false;', '    m_can_resume = false;\n    m_doubleSolve.reset();')
    cpp.write_text(text,encoding="utf-8")

# Keep the equation fixed through a budgeted quasi-static solve. Report input
# drift separately from the verified residual of that equation.
text=header.read_text(encoding="utf-8")
if "VE_P8_LOAD_SNAPSHOT" not in text:
    text=text.replace('    StressDoubleSolve m_doubleSolve;', '''    StressDoubleSolve m_doubleSolve;
    POD_Buffer<AngLin6> m_loadSnapshot; // VE_P8_LOAD_SNAPSHOT
    float m_loadSnapshotDrift = 0;''')
    text=text.replace('    float equilibriumError() const', '    float loadSnapshotDrift() const { return m_loadSnapshotDrift; }\n    float equilibriumError() const')
    header.write_text(text,encoding="utf-8")
text=cpp.read_text(encoding="utf-8")
if "VE_P8_LOAD_SNAPSHOT" not in text:
    text=text.replace('    const bool hot = params.warmStart', '    bool hot = params.warmStart')
    text=text.replace('    m_equilibrium.project(b, m_B);', '''    m_equilibrium.project(b, m_B);

    // VE_P8_LOAD_SNAPSHOT: a 0.5% per-island change limit bounds stale input.
    // New topology, changed accuracy, cold starts and larger load changes restart.
    const bool reusable=params.warmStart && m_can_resume && m_loadSnapshot.size()==N_nodes &&
        params.tolerance==m_previousTolerance && params.equilibriumTolerance==m_previousEquilibriumTolerance &&
        params.equilibriumAbsoluteTolerance==m_previousAbsoluteTolerance;
    m_loadSnapshotDrift=reusable ? float(m_equilibrium.relativeChange(b,m_loadSnapshot.data(),m_B,params.equilibriumAbsoluteTolerance)):0;
    hot=reusable && m_loadSnapshotDrift<=0.005f;
    if(hot) b=m_loadSnapshot.data();
    else
    {
        m_loadSnapshot.resize(N_nodes);
        std::copy(b,b+N_nodes,m_loadSnapshot.data());
        m_loadSnapshotDrift=0;
    }''')
    # A completed snapshot must be updated on the next call, not held forever.
    # Snapshot reuse only while solving; double recurrence survives all unfinished calls.
    cpp.write_text(text,encoding="utf-8")
ext=root / "source/sdk/extensions/stress/NvBlastExtStressSolver.cpp"
text=ext.read_text(encoding="utf-8")
if "VE_P8_LOAD_SNAPSHOT" not in text:
    text=text.replace('    float equilibriumError() const { return m_stressProcessor.equilibriumError(); }',
                      '    float loadSnapshotDrift() const { return m_stressProcessor.loadSnapshotDrift(); } // VE_P8_LOAD_SNAPSHOT\n    float equilibriumError() const { return m_stressProcessor.equilibriumError(); }')
    text=text.replace('    float equilibriumError() const { return m_solver.equilibriumError(); }',
                      '    float loadSnapshotDrift() const { return m_solver.loadSnapshotDrift(); }\n    float equilibriumError() const { return m_solver.equilibriumError(); }')
    text=text.replace('    virtual float getEquilibriumError() const override',
                      '    virtual float getLoadSnapshotDrift() const override { return m_graphProcessor->loadSnapshotDrift(); }\n    virtual float getEquilibriumError() const override')
    ext.write_text(text,encoding="utf-8")
api=root / "include/extensions/stress/NvBlastExtStressSolver.h"
text=api.read_text(encoding="utf-8")
if "getLoadSnapshotDrift" not in text:
    text=text.replace('    virtual float getEquilibriumError() const = 0;',
                      '    virtual float getEquilibriumError() const = 0;\n    virtual float getLoadSnapshotDrift() const = 0; // VE_P8_LOAD_SNAPSHOT')
    api.write_text(text,encoding="utf-8")
text=header.read_text(encoding="utf-8")
if "invalidateIteration" not in text:
    text=text.replace('    float loadSnapshotDrift() const',
                      '    void invalidateIteration() { m_can_resume=false; m_doubleSolve.reset(); }\n    float loadSnapshotDrift() const')
    header.write_text(text,encoding="utf-8")
text=ext.read_text(encoding="utf-8")
if "m_stressProcessor.invalidateIteration();" not in text:
    old='        m_inputsChanged = true;\n        m_impulses[bond].lin = impulseLinear;'
    if text.count(old)!=1:
        raise SystemExit("P8: expected one explicit bond impulse seed")
    text=text.replace(old,'        m_stressProcessor.invalidateIteration();\n'+old,1)
    ext.write_text(text,encoding="utf-8")
for name in ["StressDoubleSolve.h", "StressEquilibrium.h"]:
    source=Path(__file__).with_name(name).read_text(encoding="utf-8")
    path=base/name
    if not path.exists() or path.read_text(encoding="utf-8")!=source:
        path.write_text(source,encoding="utf-8")
