"""Deprecated import path — use :mod:`soarm_sdk.robot.servo`.

Importing this module emits a :class:`DeprecationWarning`; it is scheduled
for removal in **0.3.0**.
"""

from __future__ import annotations

import warnings

from .robot.servo import *  # noqa: F401,F403
from .robot.servo import ServoRobot  # noqa: F401

warnings.warn(
    "soarm_sdk.servo_robot is deprecated; import from soarm_sdk.robot.servo "
    "(or the top-level soarm_sdk namespace) instead. "
    "This shim will be removed in 0.3.0.",
    DeprecationWarning,
    stacklevel=2,
)
