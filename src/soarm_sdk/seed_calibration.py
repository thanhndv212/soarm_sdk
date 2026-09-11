"""Deprecated import path — use :mod:`soarm_sdk.calibration.seed`.

Kept so ``python -m soarm_sdk.seed_calibration --lerobot ...`` (documented
in soarm_tamp's README and CLAUDE.md) keeps working.
"""

from __future__ import annotations

import sys

from .calibration.seed import *  # noqa: F401,F403
from .calibration.seed import main, SO101_URDF_LIMITS, DEFAULT_OUT  # noqa: F401

if __name__ == "__main__":
    sys.exit(main())
