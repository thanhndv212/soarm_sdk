"""Deprecated import path — use :mod:`soarm_sdk.protocol.port_handler`.

Importing this module emits a :class:`DeprecationWarning`; it is scheduled
for removal in **0.3.0**.
"""

from __future__ import annotations

import warnings

from .protocol.port_handler import *  # noqa: F401,F403
from .protocol.port_handler import PortHandler  # noqa: F401

warnings.warn(
    "soarm_sdk.port_handler is deprecated; import from soarm_sdk.protocol.port_handler "
    "(or the top-level soarm_sdk namespace) instead. "
    "This shim will be removed in 0.3.0.",
    DeprecationWarning,
    stacklevel=2,
)
