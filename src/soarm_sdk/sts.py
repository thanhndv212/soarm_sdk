"""Deprecated import path — use :mod:`soarm_sdk.protocol.sts`.

Importing this module emits a :class:`DeprecationWarning`; it is scheduled
for removal in **0.3.0**.
"""

from __future__ import annotations

import warnings

from .protocol.sts import *  # noqa: F401,F403
from .protocol.sts import sts  # noqa: F401

warnings.warn(
    "soarm_sdk.sts is deprecated; import from soarm_sdk.protocol.sts "
    "(or the top-level soarm_sdk namespace) instead. "
    "This shim will be removed in 0.3.0.",
    DeprecationWarning,
    stacklevel=2,
)
