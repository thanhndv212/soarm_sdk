"""Deprecated import path — use :mod:`soarm_sdk.calibration.frame`.

Importing this module emits a :class:`DeprecationWarning`; it is scheduled
for removal in **0.3.0**.
"""

from __future__ import annotations

import warnings

from .calibration.frame import *  # noqa: F401,F403
from .calibration.frame import (  # noqa: F401
    JointCalibration,
    RobotCalibration,
    seed_from_travel,
    seed_from_lerobot,
)

warnings.warn(
    "soarm_sdk.frame_calibration is deprecated; import from soarm_sdk.calibration.frame "
    "(or the top-level soarm_sdk namespace) instead. "
    "This shim will be removed in 0.3.0.",
    DeprecationWarning,
    stacklevel=2,
)
