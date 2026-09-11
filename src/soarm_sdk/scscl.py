"""Deprecated import path — use :mod:`soarm_sdk.protocol.scscl`.

Importing this module emits a :class:`DeprecationWarning`; it is scheduled
for removal in **0.3.0**.
"""

from __future__ import annotations

import warnings

from .protocol.scscl import *  # noqa: F401,F403
from .protocol.scscl import scscl  # noqa: F401

warnings.warn(
    "soarm_sdk.scscl is deprecated; import from soarm_sdk.protocol.scscl "
    "(or the top-level soarm_sdk namespace) instead. "
    "This shim will be removed in 0.3.0.",
    DeprecationWarning,
    stacklevel=2,
)
