"""Deprecated import path — use :mod:`soarm_sdk.calibration.frame`.

Kept because downstream repos (soarm_tamp) import this module by its old
path directly: ``from soarm_sdk.frame_calibration import RobotCalibration``.
"""

from __future__ import annotations

from .calibration.frame import *  # noqa: F401,F403
from .calibration.frame import (  # noqa: F401
    JointCalibration,
    RobotCalibration,
    seed_from_travel,
    seed_from_lerobot,
)
