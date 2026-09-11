"""Deprecated import path — use :mod:`soarm_sdk.robot.hardware`.

Importing this module emits a :class:`DeprecationWarning`; it is scheduled
for removal in **0.3.0**.
"""

from __future__ import annotations

import warnings

from .robot.hardware import *  # noqa: F401,F403
from .robot.hardware import ServoHardwareInterface  # noqa: F401

warnings.warn(
    "soarm_sdk.hardware_interface is deprecated; import from soarm_sdk.robot.hardware "
    "(or the top-level soarm_sdk namespace) instead. "
    "This shim will be removed in 0.3.0.",
    DeprecationWarning,
    stacklevel=2,
)
