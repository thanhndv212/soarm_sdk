"""Deprecated import path — use :mod:`soarm_sdk.protocol.registers`.

Importing this module emits a :class:`DeprecationWarning`; it is scheduled
for removal in **0.3.0**. Every register constant (``STS_*``, ``COMM_*``, ...) is re-exported.
"""

from __future__ import annotations

import warnings

from .protocol.registers import *  # noqa: F401,F403

warnings.warn(
    "soarm_sdk.stservo_def is deprecated; import from soarm_sdk.protocol.registers "
    "(or the top-level soarm_sdk namespace) instead. "
    "This shim will be removed in 0.3.0.",
    DeprecationWarning,
    stacklevel=2,
)
