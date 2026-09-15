#!/usr/bin/env python
"""Thin launcher for the calibration dashboard — see :mod:`soarm_sdk.cli.dashboard`.

The guided, four-step URDF-frame calibration and acceptance workflow.

Kept for running directly from a checkout without installing the package
first; ``pip install soarm-sdk`` gives you the ``soarm-dashboard-calibration``
console script instead, which does the same thing.
"""

from __future__ import annotations

import sys
from pathlib import Path

_src_root = Path(__file__).resolve().parents[1] / "src"
if _src_root.is_dir() and str(_src_root) not in sys.path:
    sys.path.insert(0, str(_src_root))

from soarm_sdk.cli.dashboard import main_calibration  # noqa: E402

if __name__ == "__main__":
    main_calibration()
