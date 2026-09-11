"""Deprecated import path — use :mod:`soarm_sdk.robot.types`.

Importing this module emits a :class:`DeprecationWarning`; it is scheduled
for removal in **0.3.0**.
"""

from __future__ import annotations

import warnings

from .robot.types import *  # noqa: F401,F403
from .robot.types import (  # noqa: F401
    Pose,
    JointState,
)

warnings.warn(
    "soarm_sdk.types is deprecated; import from soarm_sdk.robot.types "
    "(or the top-level soarm_sdk namespace) instead. "
    "This shim will be removed in 0.3.0.",
    DeprecationWarning,
    stacklevel=2,
)
