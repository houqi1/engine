#!/usr/bin/env python3
"""Convert physical node torque to the pinned SDK coupling convention."""
from pathlib import Path
import sys

root = Path(sys.argv[1])
path = root / "source/sdk/extensions/stress/NvBlastExtStressSolver.cpp"
source = path.read_text(encoding="utf-8")
old = "            m_nodesData[node].localAngVel += (mode == ExtForceMode::FORCE) ? torque / inertia : torque;"
new = """            // VE_P7_LOAD_TORQUE: C uses M - offset cross F. Physical torque
            // therefore enters its angular coordinates with the opposite sign.
            m_nodesData[node].localAngVel -= (mode == ExtForceMode::FORCE) ? torque / inertia : torque;"""
if new not in source:
    if source.count(old) != 1:
        raise SystemExit("P7: expected one P3 angular load assignment")
    path.write_text(source.replace(old, new, 1), encoding="utf-8")
