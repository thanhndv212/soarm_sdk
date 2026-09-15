#!/usr/bin/env python
"""Thin launcher for the ROM calibration sweep — see
:mod:`soarm_sdk.calibration.sweep_cli`.

Drives every joint into both of its mechanical hard stops, measures the
travel, and writes this arm's URDF-frame calibration to
``~/.soarm_sdk/calibration.json``. The hard stops are the only physical
reference a servo and the URDF can both name, which is why the calibration
is derived from them.

    python examples/calibrate_rom.py --arm-id thanh_arm

This does not settle the direction signs — a travel range says how far a
joint moves, not which end is which — so it writes ``validated: false``.
``soarm-dashboard-calibration`` (or ``python -m soarm_tamp.validate_calibration``)
confirms them on the arm before anything streams a planned trajectory
against the file.

Kept for running directly from a checkout without installing the package
first; ``pip install soarm-sdk`` gives you the ``soarm-calibrate-rom``
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
