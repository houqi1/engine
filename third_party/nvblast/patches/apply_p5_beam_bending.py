#!/usr/bin/env python3
"""Use an equivalent-square beam section for NvBlast ExtStress bending stress."""
from __future__ import annotations

import pathlib
import sys


MARKER = "VE_P5_BEAM_BENDING"


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: apply_p5_beam_bending.py <blast-root>")
    cpp = pathlib.Path(sys.argv[1]) / "source" / "sdk" / "extensions" / "stress" / "NvBlastExtStressSolver.cpp"
    text = cpp.read_text(encoding="utf-8")
    if MARKER in text:
        updated = text.replace(
            "// ignore impulseAngular for now, not sure how to account for that",
            "// Angular impulse contributes the bond's twist and bending stress below.",
            1,
        )
        if updated != text:
            cpp.write_text(updated, encoding="utf-8")
        return

    old = """        const float bendContribution = bend * 2.0f / nodeDist;
        stressNormal += copysignf(bendContribution, stressNormal);"""
    new = """        // VE_P5_BEAM_BENDING: approximate the remaining bond section as a square.
        // For side b=sqrt(A), I=b^4/12 and c=b/2, so Mc/I = 6M/b^3.
        // Since bend above is |M|/A, this is 6*bend/sqrt(A).
        const float bendContribution = bend * 6.0f / sqrtf(bondArea);
        stressNormal += copysignf(bendContribution, stressNormal);"""
    if old not in text:
        raise SystemExit(f"beam-bending patch failed, pattern not found in {cpp}")
    updated = text.replace(old, new, 1).replace(
        "// ignore impulseAngular for now, not sure how to account for that",
        "// Angular impulse contributes the bond's twist and bending stress below.",
        1,
    )
    cpp.write_text(updated, encoding="utf-8")


if __name__ == "__main__":
    main()
