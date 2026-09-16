#!/usr/bin/env python
"""Thin launcher for the live telemetry monitor — see :mod:`soarm_sdk.cli.monitor`.

Read-only Rerun viewer: position/velocity/current/load/temperature/voltage
across several servos at once, in two switchable layouts (by servo, by
telemetry channel). Never commands the arm.

    python examples/monitor.py --joint-ids 1-6

Needs the ``telemetry`` extra: ``pip install soarm-sdk[telemetry]``.

Kept for running directly from a checkout without installing the package
first; ``pip install soarm-sdk[telemetry]`` gives you the ``soarm-monitor``
console script instead, which does the same thing.
"""

from __future__ import annotations

import sys
from pathlib import Path

_src_root = Path(__file__).resolve().parents[1] / "src"
if _src_root.is_dir() and str(_src_root) not in sys.path:
    sys.path.insert(0, str(_src_root))

from soarm_sdk.cli.monitor import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
