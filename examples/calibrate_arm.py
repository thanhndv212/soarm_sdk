#!/usr/bin/env python
"""Thin launcher for the ROM-sweep calibration CLI — see :mod:`soarm_sdk.calibration.sweep_cli`.

Kept for running directly from a checkout without re-installing the package
first; an installed ``soarm-sdk`` gives you the ``soarm-calibrate-rom``
console script instead, which does the same thing.
"""

from __future__ import annotations

import sys
from pathlib import Path

_src_root = Path(__file__).resolve().parents[1] / "src"
if _src_root.is_dir() and str(_src_root) not in sys.path:
    sys.path.insert(0, str(_src_root))

from soarm_sdk.calibration.sweep_cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
