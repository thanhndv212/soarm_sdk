#!/usr/bin/env python
"""Thin launcher for servo reconfiguration — see :mod:`soarm_sdk.cli.reconfigure`.

Servo setup on the wire: discover servos, assign IDs, set angle limits,
speed, acceleration, torque, mode and baud rate in their EEPROM. Nothing
to do with the URDF — for that, see ``calibrate_rom.py``.

    python examples/reconfigure.py --scan-range 1-6 --ui

Kept for running directly from a checkout without installing the package
first; ``pip install soarm-sdk`` gives you the ``soarm-reconfigure``
console script instead, which does the same thing.
"""

from __future__ import annotations

import sys
from pathlib import Path

_src_root = Path(__file__).resolve().parents[1] / "src"
if _src_root.is_dir() and str(_src_root) not in sys.path:
    sys.path.insert(0, str(_src_root))

from soarm_sdk.cli.reconfigure import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
