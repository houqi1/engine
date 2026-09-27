#!/usr/bin/env python3
"""P4 ExtStress warm start across rebuilt assets: export solved bond impulses and seed
them into another solver. Idempotent. Requires P1 and P3 patches."""
from __future__ import annotations

import pathlib
import sys

MARKER = "VE_P4_EXTSTRESS"


def patch_once(path: pathlib.Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    if old not in text:
        if new in text or MARKER in text:
            return
        raise SystemExit(f"patch failed, pattern not found in {path}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: apply_p4_extstress.py <blast-root>")
    root = pathlib.Path(sys.argv[1])
    header = root / "include" / "extensions" / "stress" / "NvBlastExtStressSolver.h"
    cpp = root / "source" / "sdk" / "extensions" / "stress" / "NvBlastExtStressSolver.cpp"

    patch_once(
        header,
        """    virtual void                            addLoad(uint32_t graphNodeIndex, NvcVec3 localForce, NvcVec3 localTorque,
                                                    ExtForceMode::Enum mode = ExtForceMode::FORCE) = 0;
};""",
        """    virtual void                            addLoad(uint32_t graphNodeIndex, NvcVec3 localForce, NvcVec3 localTorque,
                                                    ExtForceMode::Enum mode = ExtForceMode::FORCE) = 0;

    // VE_P4_EXTSTRESS: carry solved bond impulses into a rebuilt asset's solver (warm start).
    // node0/node1 are the solver bond's graph nodes; a seed whose nodes arrive swapped is negated.
    struct BondImpulse
    {
        uint32_t    blastBondIndex;
        uint32_t    node0;
        uint32_t    node1;
        NvcVec3     linear;
        NvcVec3     angular;
    };

    virtual uint32_t                        copyBondImpulses(BondImpulse* out, uint32_t maxCount) const = 0;
    // Applied on the next update(), after the solver graph is synced; that solve warm-starts.
    virtual void                            seedBondImpulses(const BondImpulse* in, uint32_t count) = 0;
};""",
    )

    patch_once(
        cpp,
        """#include "NvBlastExtStressSolver.h"
#include "NvBlast.h\"""",
        """#include "NvBlastExtStressSolver.h"
#include "NvBlast.h"
#include <unordered_map> // VE_P4_EXTSTRESS
#include <vector>""",
    )

    patch_once(
        cpp,
        """    void getBondNodes(uint32_t bond, uint32_t& node0, uint32_t& node1) const
    {
        NVBLAST_ASSERT(bond < m_bonds.size());
        const SolverBond& b = m_bonds[bond];
        node0 = b.nodes[0];
        node1 = b.nodes[1];
    }""",
        """    void getBondNodes(uint32_t bond, uint32_t& node0, uint32_t& node1) const
    {
        NVBLAST_ASSERT(bond < m_bonds.size());
        const SolverBond& b = m_bonds[bond];
        node0 = b.nodes[0];
        node1 = b.nodes[1];
    }

    // VE_P4_EXTSTRESS
    void setBondImpulses(uint32_t bond, const NvcVec3& impulseLinear, const NvcVec3& impulseAngular)
    {
        NVBLAST_ASSERT(bond < m_impulses.size());
        m_impulses[bond].lin = impulseLinear;
        m_impulses[bond].ang = impulseAngular;
    }

    void allowWarmStart()
    {
        m_forceColdStart = false;
    }""",
    )

    patch_once(
        cpp,
        """    void solve(const ExtStressSolverSettings& settings, const float* bondHealth, const NvBlastBond* bonds, bool warmStart = true)
    {
        sync(bonds);
""",
        """    // VE_P4_EXTSTRESS
    uint32_t copyBondImpulses(ExtStressSolver::BondImpulse* out, uint32_t maxCount) const
    {
        uint32_t n = 0;
        for (uint32_t i = 0; i < m_solverBondsData.size() && i < m_solver.getBondCount(); ++i)
        {
            NvVec3 lin, ang;
            uint32_t node0, node1;
            m_solver.getBondImpulses(i, lin, ang);
            m_solver.getBondNodes(i, node0, node1);
            for (uint32_t blastBond : m_solverBondsData[i].blastBondIndices)
            {
                if (n >= maxCount)
                {
                    return n;
                }
                out[n++] = { blastBond, node0, node1, { lin.x, lin.y, lin.z }, { ang.x, ang.y, ang.z } };
            }
        }
        return n;
    }

    void seedBondImpulses(const ExtStressSolver::BondImpulse* in, uint32_t count)
    {
        m_seed.assign(in, in + count);
    }

    uint32_t lastSeededBonds() const
    {
        return m_lastSeeded;
    }

    bool applySeed()
    {
        if (m_seed.empty())
        {
            return false;
        }
        std::unordered_map<uint32_t, const ExtStressSolver::BondImpulse*> byBlast;
        byBlast.reserve(m_seed.size() * 2);
        for (const auto& s : m_seed)
        {
            byBlast.emplace(s.blastBondIndex, &s);
        }
        m_lastSeeded = 0;
        for (uint32_t i = 0; i < m_solverBondsData.size() && i < m_solver.getBondCount(); ++i)
        {
            for (uint32_t blastBond : m_solverBondsData[i].blastBondIndices)
            {
                const auto it = byBlast.find(blastBond);
                if (it == byBlast.end())
                {
                    continue;
                }
                const ExtStressSolver::BondImpulse& s = *it->second;
                uint32_t node0, node1;
                m_solver.getBondNodes(i, node0, node1);
                float sign = 0.0f;
                if (node0 == s.node0 && node1 == s.node1)
                    sign = 1.0f;
                else if (node0 == s.node1 && node1 == s.node0)
                    sign = -1.0f;
                if (sign != 0.0f)
                {
                    m_solver.setBondImpulses(i, { sign * s.linear.x, sign * s.linear.y, sign * s.linear.z },
                                             { sign * s.angular.x, sign * s.angular.y, sign * s.angular.z });
                    ++m_lastSeeded;
                }
                break;
            }
        }
        m_seed.clear();
        m_solver.allowWarmStart();
        return true;
    }

    void solve(const ExtStressSolverSettings& settings, const float* bondHealth, const NvBlastBond* bonds, bool warmStart = true)
    {
        sync(bonds);
        const bool seeded = applySeed(); // VE_P4_EXTSTRESS
""",
    )

    patch_once(
        cpp,
        """        m_solver.solve(settings.maxSolverIterationsPerFrame, warmStart);

        resetVelocities();""",
        """        m_solver.solve(settings.maxSolverIterationsPerFrame, warmStart || seeded);

        resetVelocities();""",
    )

    patch_once(
        cpp,
        """    ConjugateGradientImpulseSolver      m_solver;""",
        """    ConjugateGradientImpulseSolver      m_solver;
    std::vector<ExtStressSolver::BondImpulse> m_seed; // VE_P4_EXTSTRESS
    uint32_t                            m_lastSeeded = 0;""",
    )

    patch_once(
        cpp,
        """    virtual void                            addLoad(uint32_t graphNodeIndex, NvcVec3 localForce, NvcVec3 localTorque,
                                                    ExtForceMode::Enum mode) override;
""",
        """    virtual void                            addLoad(uint32_t graphNodeIndex, NvcVec3 localForce, NvcVec3 localTorque,
                                                    ExtForceMode::Enum mode) override;
    virtual uint32_t                        copyBondImpulses(BondImpulse* out, uint32_t maxCount) const override // VE_P4_EXTSTRESS
    {
        return m_graphProcessor->copyBondImpulses(out, maxCount);
    }
    virtual void                            seedBondImpulses(const BondImpulse* in, uint32_t count) override
    {
        m_graphProcessor->seedBondImpulses(in, count);
    }
""",
    )


if __name__ == "__main__":
    main()
