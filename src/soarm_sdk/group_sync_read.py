"""Deprecated import path — use :mod:`soarm_sdk.protocol.group_sync_read`.

Importing this module emits a :class:`DeprecationWarning`; it is scheduled
for removal in **0.3.0**.
"""

from __future__ import annotations

import warnings

from .protocol.group_sync_read import *  # noqa: F401,F403
from .protocol.group_sync_read import GroupSyncRead  # noqa: F401

warnings.warn(
    "soarm_sdk.group_sync_read is deprecated; import from soarm_sdk.protocol.group_sync_read "
    "(or the top-level soarm_sdk namespace) instead. "
    "This shim will be removed in 0.3.0.",
    DeprecationWarning,
    stacklevel=2,
)
