#!/usr/bin/env python3
"""P3 ExtStress generalized node load (force + torque). Idempotent. Requires P1 patch."""
from __future__ import annotations

import pathlib
import sys

MARKER = "VE_P3_EXTSTRESS"


def patch_once(path: pathlib.Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    if old not in text:
        if new in text or MARKER in text:
            return
        raise SystemExit(f"patch failed, pattern not found in {path}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: apply_p3_extstress.py <blast-root>")
    root = pathlib.Path(sys.argv[1])
    header = root / "include" / "extensions" / "stress" / "NvBlastExtStressSolver.h"
    cpp = root / "source" / "sdk" / "extensions" / "stress" / "NvBlastExtStressSolver.cpp"

    patch_once(
        header,
        """    virtual uint32_t                        copyBondProbes(BondProbe* out, uint32_t maxCount) const = 0;
    virtual uint32_t                        copyBondProbesLegacy(BondProbe* out, uint32_t maxCount) const = 0; // VE_P1_COPYBOND_LINEAR
    virtual uint32_t                        topologyEpoch() const = 0;
    virtual void                            syncBrokenBonds() = 0;
};""",
        """    virtual uint32_t                        copyBondProbes(BondProbe* out, uint32_t maxCount) const = 0;
    virtual uint32_t                        copyBondProbesLegacy(BondProbe* out, uint32_t maxCount) const = 0; // VE_P1_COPYBOND_LINEAR
    virtual uint32_t                        topologyEpoch() const = 0;
    virtual void                            syncBrokenBonds() = 0;

    // VE_P3_EXTSTRESS: node force + torque. Not a fictional addTorque-only API.
    virtual void                            addLoad(uint32_t graphNodeIndex, NvcVec3 localForce, NvcVec3 localTorque,
                                                    ExtForceMode::Enum mode = ExtForceMode::FORCE) = 0;
};""",
    )

    patch_once(
        cpp,
        """        NvVec3 localPos;
        NvVec3 localVel;
        uint32_t solverNode;""",
        """        NvVec3 localPos;
        NvVec3 localVel;
        NvVec3 localAngVel; // VE_P3_EXTSTRESS
        uint32_t solverNode;""",
    )

    patch_once(
        cpp,
            """        for (auto& node : m_nodesData)
        {
            node.localVel = NvVec3(NvZero);
        }""",
            """        for (auto& node : m_nodesData)
        {
            node.localVel = NvVec3(NvZero);
            node.localAngVel = NvVec3(NvZero); // VE_P3_EXTSTRESS
        }""",
    )

    patch_once(
        cpp,
        """    void addNodeForce(uint32_t node, const NvVec3& force, ExtForceMode::Enum mode)
    {
        const float mass = m_nodesData[node].mass;
        if (mass > 0)
        {
            // NOTE - passing in acceleration as velocity.  The impulse solver's output will be interpreted as force.
            m_nodesData[node].localVel += (mode == ExtForceMode::FORCE) ? force/mass : force;
        }
    }""",
        """    void addNodeForce(uint32_t node, const NvVec3& force, ExtForceMode::Enum mode)
    {
        const float mass = m_nodesData[node].mass;
        if (mass > 0)
        {
            // NOTE - passing in acceleration as velocity.  The impulse solver's output will be interpreted as force.
            m_nodesData[node].localVel += (mode == ExtForceMode::FORCE) ? force/mass : force;
        }
    }

    void addNodeLoad(uint32_t node, const NvVec3& force, const NvVec3& torque, ExtForceMode::Enum mode) // VE_P3_EXTSTRESS
    {
        addNodeForce(node, force, mode);
        const float mass = m_nodesData[node].mass;
        const float volume = m_nodesData[node].volume;
        if (mass <= 0.0f)
        {
            return;
        }
        const float R = NvPow(volume * 3.0f * NvInvPi / 4.0f, 1.0f / 3.0f);
        const float inertia = mass * (R * R * 0.4f);
        if (inertia > 0.0f)
        {
            m_nodesData[node].localAngVel += (mode == ExtForceMode::FORCE) ? torque / inertia : torque;
        }
    }""",
    )

    patch_once(
        cpp,
        """        for (const NodeData& node : m_nodesData)
        {
            m_solver.setNodeVelocities(node.solverNode, node.localVel, NvVec3(NvZero));
        }""",
        """        for (const NodeData& node : m_nodesData) // VE_P3_EXTSTRESS
        {
            m_solver.setNodeVelocities(node.solverNode, node.localVel, node.localAngVel);
        }""",
    )

    patch_once(
        cpp,
        """    virtual void                            syncBrokenBonds() override;


    //////// ExtStressSolverImpl public methods ////////""",
        """    virtual void                            syncBrokenBonds() override;
    virtual void                            addLoad(uint32_t graphNodeIndex, NvcVec3 localForce, NvcVec3 localTorque,
                                                    ExtForceMode::Enum mode) override;


    //////// ExtStressSolverImpl public methods ////////""",
    )

    impl = r'''
void ExtStressSolverImpl::addLoad(uint32_t graphNode, NvcVec3 localForce, NvcVec3 localTorque, ExtForceMode::Enum mode)
{
    m_graphProcessor->addNodeLoad(graphNode, toNvShared(localForce), toNvShared(localTorque), mode);
}

'''
    cpp_text = cpp.read_text(encoding="utf-8")
    if "void ExtStressSolverImpl::addLoad(" not in cpp_text:
        needle = "void ExtStressSolverImpl::addForce(uint32_t graphNode, NvcVec3 localForce, ExtForceMode::Enum mode)\n{"
        if needle not in cpp_text:
            raise SystemExit("addForce(graphNode) not found for P3 insert")
        cpp.write_text(cpp_text.replace(needle, impl + needle, 1), encoding="utf-8")

    print("P3 ExtStress addLoad applied")


if __name__ == "__main__":
    main()
