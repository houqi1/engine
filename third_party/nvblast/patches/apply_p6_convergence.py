#!/usr/bin/env python3
"""Install verified convergence in the pinned Blast tree, idempotently."""
from pathlib import Path
import sys

root = Path(sys.argv[1])
pending = {}

def patch(rel, edits):
    path = root / rel
    source = path.read_text(encoding="utf-8")
    if "VE_P6_CONVERGENCE" in source:
        return
    for old, new in edits:
        if source.count(old) != 1:
            raise SystemExit(f"P6: expected one match in {rel}: {old[:100]!r}")
        source = source.replace(old, new, 1)
    pending[path] = source

patch("include/extensions/stress/NvBlastExtStressSolver.h", [
    ("\n    // stress pressure limits", """
    // VE_P6_CONVERGENCE: solver accuracy, independent of the per-frame work cap.
    float       solverTolerance = 1.e-6f;
    float       equilibriumTolerance = 1.e-4f;
    float       equilibriumAbsoluteTolerance = 1.e-6f;

    // stress pressure limits"""),
    ("    //////// creation ////////", """    enum class SolveStatus { NotRun, Converged, IterationLimit, NumericalFailure };
    virtual SolveStatus getSolveStatus() const = 0;
    virtual float getEquilibriumError() const = 0;

    //////// creation ////////""")])

patch("source/shared/stress_solver/stress.h", [
    ('#include "buffer.h"', '#include "buffer.h"\n#include "StressEquilibrium.h" // VE_P6_CONVERGENCE'),
    ('        bool        warmStart       = false;', '''        float       equilibriumTolerance = 1.e-4f;
        float       equilibriumAbsoluteTolerance = 1.e-6f;
        bool        warmStart       = false;'''),
    ('protected:\n', '''    float equilibriumError() const { return float(m_equilibrium.relativeError); }
protected:
    StressEquilibrium m_equilibrium;
    POD_Buffer<AngLin6> m_scaled_solution;
    float m_previousTolerance = 0, m_previousEquilibriumTolerance = 0, m_previousAbsoluteTolerance = 0;
''')])

patch("source/shared/stress_solver/math/cgnr.h", [
    ('#include <stdint.h>', '#include <stdint.h>\n#include <cmath>\n#include <climits> // VE_P6_CONVERGENCE'),
    ('        unsigned warmth = 0\n', '''        unsigned warmth = 0,
        bool (*verify)(void*, const Elem*, Elem*) = nullptr,
        void* verifyContext = nullptr
'''),
    ('            const Scalar z_sq = ElemOps().calculate_error(error, z, N);', '            Scalar z_sq = ElemOps().calculate_error(error, z, N);'),
    ('            if (le(z_sq, delta_sq)) break;', '''            if (le(z_sq, delta_sq))
            {
                if (!verify || verify(verifyContext, x, nullptr)) break;
                // Keep Krylov directions when the independent check rejects
                // convergence. Restarting on every rejection stalls weak modes.
                // If the recursive residual collapses while the true residual
                // still fails, replace it using the independent double sum.
                if (le(z_sq, mul(1.e-6f, delta_sq)))
                {
                    verify(verifyContext, x, r);
                    MatOps().lmul(z, r, A, M, N);
                    z_sq = ElemOps().calculate_error(error, z, N);
                    warmth = 0;
                }
            }
            float zCheck;
            store_float(&zCheck, z_sq);
            if (!std::isfinite(zCheck) || zCheck <= 0) return -INT_MAX;
            /* checked above */'''),
    ('            const Scalar mu = div(z_sq, ElemOps().length_sq(s, M));', '''            const Scalar denominator = ElemOps().length_sq(s, M);
            float denomCheck;
            store_float(&denomCheck, denominator);
            if (!std::isfinite(denomCheck) || denomCheck <= 0) return -INT_MAX;
            const Scalar mu = div(z_sq, denominator);
            float muCheck;
            store_float(&muCheck, mu);
            if (!std::isfinite(muCheck)) return -INT_MAX;''')])

patch("source/shared/stress_solver/stress.cpp", [
    ('#include "stress.h"', '#include "stress.h"\n#include <climits> // VE_P6_CONVERGENCE'),
    ('    m_B.set(m_couplings.data(), m_recip_sqrt_I.data(), m_B_scratch.data(), N_nodes, N_bonds);', '''    m_B.set(m_couplings.data(), m_recip_sqrt_I.data(), m_B_scratch.data(), N_nodes, N_bonds);
    m_equilibrium.prepare(nodes, N_nodes, m_length_scale);'''),
    ('    // Apply length and mass scaling to impulses if warm-starting\n    if (params.warmStart)', '''    const bool hot = params.warmStart && resume && m_can_resume &&
        params.tolerance == m_previousTolerance && params.equilibriumTolerance == m_previousEquilibriumTolerance &&
        params.equilibriumAbsoluteTolerance == m_previousAbsoluteTolerance;
    // Preserve the exact scaled iterate when resuming Krylov state.
    if (hot) std::copy(m_scaled_solution.data(), m_scaled_solution.data()+N_bonds, impulses);
    else if (params.warmStart)'''),
    ('(m_can_resume && resume ? 2 : 1)', '(hot ? 2 : 1)'),
    ('    // Solve B*J = b for J,', '''    // Remove only the left-null (rigid motion) part, independently per island.
    m_equilibrium.project(b, m_B);

    // Solve B*J = b for J,'''),
    ('    // Choose solver based on parameters\n    const int result', '''    struct CheckContext { StressEquilibrium* equilibrium; const AngLin6* b; const BondMatrixS* B; const SolverParams* params; };
    CheckContext context{&m_equilibrium, b, &m_B, &params};
    auto verify = [](void* raw, const AngLin6* x, AngLin6* residual) {
        auto& c = *static_cast<CheckContext*>(raw);
        return c.equilibrium->check(x, c.b, *c.B, c.params->equilibriumTolerance, c.params->equilibriumAbsoluteTolerance, residual);
    };

    // Choose solver based on parameters
    int result'''),
    ('cache, error_sq, params.tolerance, maxIter, warmth) :', 'cache, error_sq, params.tolerance, maxIter, warmth, verify, &context) :'),
    ('cache, error_sq, params.tolerance, maxIter, warmth);', 'cache, error_sq, params.tolerance, maxIter, warmth, verify, &context);'),
    ('    // Undo length and mass scaling', '''    verify(&context, impulses, nullptr);
    if (!m_equilibrium.finite || result == -INT_MAX)
    {
        result = -INT_MAX;
        // Never feed non-finite stresses into fracture or the next warm start.
        for (uint32_t j=0; j<N_bonds; ++j) impulses[j] = {{0,0,0},{0,0,0}};
    }

    m_scaled_solution.resize(N_bonds);
    std::copy(impulses, impulses+N_bonds, m_scaled_solution.data());
    m_previousTolerance = params.tolerance;
    m_previousEquilibriumTolerance = params.equilibriumTolerance;
    m_previousAbsoluteTolerance = params.equilibriumAbsoluteTolerance;

    // Undo length and mass scaling'''),
    ('    m_can_resume = true;', '    m_can_resume = result < 0 && result != -INT_MAX;'),
    ('    --m_B.N;', '    --m_B.N;\n    m_equilibrium.topologyChanged();')])

patch("source/sdk/extensions/stress/NvBlastExtStressSolver.cpp", [
    ('#include <algorithm>', '#include <algorithm>\n#include <climits> // VE_P6_CONVERGENCE'),
    ('        m_impulses[bond].lin = impulseLinear;', '        m_inputsChanged = true;\n        m_impulses[bond].lin = impulseLinear;'),
    ('        v.ang = { velocityAngular.x, velocityAngular.y, velocityAngular.z };\n        v.lin = { velocityLinear.x, velocityLinear.y, velocityLinear.z };\n        m_inputsChanged = true;', '''        m_inputsChanged |= v.ang.x != velocityAngular.x || v.ang.y != velocityAngular.y || v.ang.z != velocityAngular.z ||
            v.lin.x != velocityLinear.x || v.lin.y != velocityLinear.y || v.lin.z != velocityLinear.z;
        v.ang = { velocityAngular.x, velocityAngular.y, velocityAngular.z };
        v.lin = { velocityLinear.x, velocityLinear.y, velocityLinear.z };'''),
    ('        m_error_sq = {FLT_MAX, FLT_MAX};', '        m_error_sq = {FLT_MAX, FLT_MAX};\n        m_status = ExtStressSolver::SolveStatus::NotRun;'),
    ('    void solve(uint32_t iterationCount, bool warmStart = true)', '    void solve(const ExtStressSolverSettings& settings, bool warmStart = true)'),
    ('        params.maxIter = iterationCount;\n        params.tolerance = 0.001f;', '''        params.maxIter = settings.maxSolverIterationsPerFrame;
        params.tolerance = settings.solverTolerance;
        params.equilibriumTolerance = settings.equilibriumTolerance;
        params.equilibriumAbsoluteTolerance = settings.equilibriumAbsoluteTolerance;'''),
    ('        m_converged = (m_stressProcessor.solve(m_impulses.data(), m_velocities.data(), params, &m_error_sq) >= 0);', '''        const int result = m_stressProcessor.solve(m_impulses.data(), m_velocities.data(), params, &m_error_sq, !m_inputsChanged);
        m_converged = result >= 0;
        m_status = result == -INT_MAX ? ExtStressSolver::SolveStatus::NumericalFailure :
            m_converged ? ExtStressSolver::SolveStatus::Converged : ExtStressSolver::SolveStatus::IterationLimit;'''),
    ('    bool calcError(float& linear, float& angular) const\n    {\n        linear =', '''    ExtStressSolver::SolveStatus status() const { return m_status; }
    float equilibriumError() const { return m_stressProcessor.equilibriumError(); }
    bool calcError(float& linear, float& angular) const
    {
        linear ='''),
    ('    AngLin6ErrorSq              m_error_sq;', '''    ExtStressSolver::SolveStatus m_status = ExtStressSolver::SolveStatus::NotRun;
    AngLin6ErrorSq              m_error_sq;'''),
    ('        m_solver.solve(settings.maxSolverIterationsPerFrame, warmStart || seeded);', '        m_solver.solve(settings, warmStart || seeded);'),
    ('    bool calcError(float& linear, float& angular) const\n    {\n        return m_solver.calcError', '''    ExtStressSolver::SolveStatus status() const { return m_solver.status(); }
    float equilibriumError() const { return m_solver.equilibriumError(); }
    bool calcError(float& linear, float& angular) const
    {
        return m_solver.calcError'''),
    ('    virtual bool                            converged() const override', '''    virtual SolveStatus getSolveStatus() const override { return m_graphProcessor->status(); }
    virtual float getEquilibriumError() const override { return m_graphProcessor->equilibriumError(); }
    virtual bool                            converged() const override'''),
    ('    void                                    inheritSettingsLimits()\n    {', '''    void                                    inheritSettingsLimits()
    {
        auto positive = [](float value, float fallback) { return std::isfinite(value) && value > 0 ? value : fallback; };
        m_settings.solverTolerance = positive(m_settings.solverTolerance, 1.e-6f);
        m_settings.equilibriumTolerance = positive(m_settings.equilibriumTolerance, 1.e-4f);
        m_settings.equilibriumAbsoluteTolerance = positive(m_settings.equilibriumAbsoluteTolerance, 1.e-6f);''')])

helper = Path(__file__).with_name("StressEquilibrium.h")
pending[root / "source/shared/stress_solver/StressEquilibrium.h"] = helper.read_text(encoding="utf-8")
for path, content in pending.items():
    if not path.exists() or path.read_text(encoding="utf-8") != content:
        path.write_text(content, encoding="utf-8")
