#!/usr/bin/env python3
"""Apply P1 ExtStress adapters onto a sparse-cloned Blast tree. Idempotent."""
from __future__ import annotations

import pathlib
import sys

MARKER = "VE_P1_EXTSTRESS"
LINEAR = "VE_P1_COPYBOND_LINEAR"

LEGACY_IMPL = r'''
uint32_t ExtStressSolverImpl::copyBondProbesLegacy(BondProbe* out, uint32_t maxCount) const
{
    if (out == nullptr || maxCount == 0)
    {
        return 0;
    }
    const NvBlastAsset* asset = NvBlastFamilyGetAsset(&m_family, logLL);
    if (asset == nullptr)
    {
        return 0;
    }
    const uint32_t bondCount = NvBlastAssetGetBondCount(asset, logLL);
    uint32_t written = 0;
    for (uint32_t i = 0; i < bondCount && written < maxCount; ++i)
    {
        BondProbe p{};
        p.blastBondIndex = i;
        p.health = m_bondHealths != nullptr ? m_bondHealths[i] : 0.0f;
        p.node0 = 0xFFFFFFFF;
        p.node1 = 0xFFFFFFFF;
        for (uint32_t n0 = 0; n0 < m_graph.nodeCount; ++n0)
        {
            for (uint32_t adj = m_graph.adjacencyPartition[n0]; adj < m_graph.adjacencyPartition[n0 + 1]; ++adj)
            {
                if (m_graph.adjacentBondIndices[adj] == i)
                {
                    p.node0 = n0;
                    p.node1 = m_graph.adjacentNodeIndices[adj];
                    n0 = m_graph.nodeCount;
                    break;
                }
            }
        }
        float compression = 0.0f;
        float tension = 0.0f;
        float shear = 0.0f;
        m_graphProcessor->getBondStress(i, compression, tension, shear);
        p.compression = compression;
        p.tension = tension;
        p.shear = shear;
        NvVec3 force(NvZero);
        NvVec3 torque(NvZero);
        const uint32_t solverCount = m_graphProcessor->getSolverBondCount();
        for (uint32_t sb = 0; sb < solverCount; ++sb)
        {
            const auto& sbd = m_graphProcessor->getSolverBondData(sb);
            for (uint32_t bbi : sbd.blastBondIndices)
            {
                if (bbi == i)
                {
                    m_graphProcessor->getSolverInternalBondImpulses(sb, force, torque);
                    sb = solverCount;
                    break;
                }
            }
        }
        p.forceLinear = {force.x, force.y, force.z};
        p.moment = {torque.x, torque.y, torque.z};
        out[written++] = p;
    }
    return written;
}

'''

LINEAR_IMPL = r'''
uint32_t ExtStressSolverImpl::copyBondProbes(BondProbe* out, uint32_t maxCount) const
{
    // VE_P1_COPYBOND_LINEAR: three linear passes. First adjacency hit and first
    // solver-bond mapping win, matching copyBondProbesLegacy.
    if (out == nullptr || maxCount == 0)
    {
        return 0;
    }
    const NvBlastAsset* asset = NvBlastFamilyGetAsset(&m_family, logLL);
    if (asset == nullptr)
    {
        return 0;
    }
    const uint32_t bondCount = NvBlastAssetGetBondCount(asset, logLL);
    const uint32_t written = bondCount < maxCount ? bondCount : maxCount;

    for (uint32_t i = 0; i < written; ++i)
    {
        BondProbe& p = out[i];
        p.blastBondIndex = i;
        p.health = m_bondHealths != nullptr ? m_bondHealths[i] : 0.0f;
        p.node0 = 0xFFFFFFFF;
        p.node1 = 0xFFFFFFFF;
        p.forceLinear = {0.0f, 0.0f, 0.0f};
        p.moment = {0.0f, 0.0f, 0.0f};
        float compression = 0.0f;
        float tension = 0.0f;
        float shear = 0.0f;
        m_graphProcessor->getBondStress(i, compression, tension, shear);
        p.compression = compression;
        p.tension = tension;
        p.shear = shear;
    }

    for (uint32_t n0 = 0; n0 < m_graph.nodeCount; ++n0)
    {
        for (uint32_t adj = m_graph.adjacencyPartition[n0]; adj < m_graph.adjacencyPartition[n0 + 1]; ++adj)
        {
            const uint32_t bond = m_graph.adjacentBondIndices[adj];
            if (bond >= written)
            {
                continue;
            }
            BondProbe& p = out[bond];
            if (p.node0 != 0xFFFFFFFF)
            {
                continue;
            }
            p.node0 = n0;
            p.node1 = m_graph.adjacentNodeIndices[adj];
        }
    }

    Array<uint8_t>::type impulseSet;
    impulseSet.resize(written);
    if (written > 0)
    {
        memset(impulseSet.begin(), 0, written);
    }
    const uint32_t solverCount = m_graphProcessor->getSolverBondCount();
    for (uint32_t sb = 0; sb < solverCount; ++sb)
    {
        NvVec3 force(NvZero);
        NvVec3 torque(NvZero);
        bool fetched = false;
        const auto& sbd = m_graphProcessor->getSolverBondData(sb);
        for (uint32_t bbi : sbd.blastBondIndices)
        {
            if (bbi >= written || impulseSet[bbi] != 0)
            {
                continue;
            }
            if (!fetched)
            {
                m_graphProcessor->getSolverInternalBondImpulses(sb, force, torque);
                fetched = true;
            }
            impulseSet[bbi] = 1;
            out[bbi].forceLinear = {force.x, force.y, force.z};
            out[bbi].moment = {torque.x, torque.y, torque.z};
        }
    }
    return written;
}

'''


def patch_once(path: pathlib.Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    if MARKER in text and old not in text:
        return
    if old not in text:
        if new in text:
            return
        raise SystemExit(f"patch failed, pattern not found in {path}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def upgrade_copy_bond_probes(header: pathlib.Path, cpp: pathlib.Path) -> None:
    h = header.read_text(encoding="utf-8")
    if LINEAR not in h:
        old = """    virtual uint32_t                        copyBondProbes(BondProbe* out, uint32_t maxCount) const = 0;
    virtual uint32_t                        topologyEpoch() const = 0;"""
        new = """    virtual uint32_t                        copyBondProbes(BondProbe* out, uint32_t maxCount) const = 0;
    virtual uint32_t                        copyBondProbesLegacy(BondProbe* out, uint32_t maxCount) const = 0; // VE_P1_COPYBOND_LINEAR
    virtual uint32_t                        topologyEpoch() const = 0;"""
        if old not in h:
            raise SystemExit("header copyBondProbes declaration not found for linear upgrade")
        header.write_text(h.replace(old, new, 1), encoding="utf-8")

    c = cpp.read_text(encoding="utf-8")
    if "copyBondProbesLegacy(BondProbe* out, uint32_t maxCount) const override" not in c:
        old = """    virtual uint32_t                        copyBondProbes(BondProbe* out, uint32_t maxCount) const override;
    virtual uint32_t                        topologyEpoch() const override { return m_topologyEpoch; }"""
        new = """    virtual uint32_t                        copyBondProbes(BondProbe* out, uint32_t maxCount) const override;
    virtual uint32_t                        copyBondProbesLegacy(BondProbe* out, uint32_t maxCount) const override;
    virtual uint32_t                        topologyEpoch() const override { return m_topologyEpoch; }"""
        if old not in c:
            raise SystemExit("cpp copyBondProbes override not found for linear upgrade")
        c = c.replace(old, new, 1)
        cpp.write_text(c, encoding="utf-8")
        c = cpp.read_text(encoding="utf-8")

    if LINEAR in c and "copyBondProbesLegacy(BondProbe* out, uint32_t maxCount) const\n{" in c:
        return

    start = c.find("uint32_t ExtStressSolverImpl::copyBondProbes(BondProbe* out, uint32_t maxCount) const\n{")
    if start < 0:
        raise SystemExit("copyBondProbes definition not found for linear upgrade")
    end = c.find("void ExtStressSolverImpl::removeBrokenBonds()\n{", start)
    if end < 0:
        raise SystemExit("removeBrokenBonds after copyBondProbes not found")
    cpp.write_text(c[:start] + LEGACY_IMPL + LINEAR_IMPL + c[end:], encoding="utf-8")


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: apply_p1_extstress.py <blast-root>")
    root = pathlib.Path(sys.argv[1])
    header = root / "include" / "extensions" / "stress" / "NvBlastExtStressSolver.h"
    cpp = root / "source" / "sdk" / "extensions" / "stress" / "NvBlastExtStressSolver.cpp"

    patch_once(
        header,
        """    virtual const DebugBuffer               fillDebugRender(const uint32_t* nodes, uint32_t nodeCount, DebugRenderMode mode, float scale = 1.0f) = 0;
};""",
        """    virtual const DebugBuffer               fillDebugRender(const uint32_t* nodes, uint32_t nodeCount, DebugRenderMode mode, float scale = 1.0f) = 0;

    // VE_P1_EXTSTRESS
    struct BondProbe
    {
        uint32_t    blastBondIndex;
        uint32_t    node0;
        uint32_t    node1;
        NvcVec3     forceLinear;
        NvcVec3     moment;
        float       compression;
        float       tension;
        float       shear;
        float       health;
    };

    virtual uint32_t                        copyBondProbes(BondProbe* out, uint32_t maxCount) const = 0;
    virtual uint32_t                        copyBondProbesLegacy(BondProbe* out, uint32_t maxCount) const = 0; // VE_P1_COPYBOND_LINEAR
    virtual uint32_t                        topologyEpoch() const = 0;
    virtual void                            syncBrokenBonds() = 0;
};""",
    )

    patch_once(
        cpp,
        """        StressProcessor::DataParams params;
        params.centerBonds = true;
        params.equalizeMasses = true;
        m_stressProcessor.prepare(m_nodes.begin(), m_nodes.size(), m_bonds.begin(), m_bonds.size(), params);""",
        """        StressProcessor::DataParams params;
        params.centerBonds = true;
        params.equalizeMasses = false; // VE_P1_EXTSTRESS
        m_stressProcessor.prepare(m_nodes.begin(), m_nodes.size(), m_bonds.begin(), m_bonds.size(), params);""",
    )

    patch_once(
        cpp,
        """    uint32_t getInternalBondIndex(uint32_t blastBondIndex)
    {
        return m_blastBondIndexMap[blastBondIndex];
    }""",
        """    uint32_t getInternalBondIndex(uint32_t blastBondIndex) const // VE_P1_EXTSTRESS
    {
        return m_blastBondIndexMap[blastBondIndex];
    }""",
    )

    patch_once(
        cpp,
        """    virtual const DebugBuffer               fillDebugRender(const uint32_t* nodes, uint32_t nodeCount, DebugRenderMode mode, float scale) override;


    //////// ExtStressSolverImpl public methods ////////""",
        """    virtual const DebugBuffer               fillDebugRender(const uint32_t* nodes, uint32_t nodeCount, DebugRenderMode mode, float scale) override;

    virtual uint32_t                        copyBondProbes(BondProbe* out, uint32_t maxCount) const override;
    virtual uint32_t                        copyBondProbesLegacy(BondProbe* out, uint32_t maxCount) const override;
    virtual uint32_t                        topologyEpoch() const override { return m_topologyEpoch; }
    virtual void                            syncBrokenBonds() override;


    //////// ExtStressSolverImpl public methods ////////""",
    )

    patch_once(
        cpp,
        """    Array<DebugLine>::type                                              m_debugLineBuffer;
    bool                                                                m_valid;
};""",
        """    Array<DebugLine>::type                                              m_debugLineBuffer;
    bool                                                                m_valid;
    uint32_t                                                            m_topologyEpoch = 0; // VE_P1_EXTSTRESS
};""",
    )

    patch_once(
        cpp,
        """    if (m_isDirty)
    {
        removeBrokenBonds();
    }""",
        """    // VE_P1_EXTSTRESS: drop zero-health bonds before the next solve, even without split.
    removeBrokenBonds();
    if (m_isDirty)
    {
        m_isDirty = false;
    }""",
    )

    cpp_text = cpp.read_text(encoding="utf-8")
    if "uint32_t ExtStressSolverImpl::copyBondProbes" not in cpp_text:
        needle = "void ExtStressSolverImpl::removeBrokenBonds()\n{"
        if needle not in cpp_text:
            raise SystemExit("removeBrokenBonds not found for impl insert")
        cpp.write_text(cpp_text.replace(needle, "void ExtStressSolverImpl::syncBrokenBonds()\n{\n    removeBrokenBonds();\n    ++m_topologyEpoch;\n}\n" + LEGACY_IMPL + LINEAR_IMPL + needle, 1), encoding="utf-8")

    upgrade_copy_bond_probes(header, cpp)
    print("P1 ExtStress adapters applied")


if __name__ == "__main__":
    main()
