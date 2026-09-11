"""Deprecated import path — use :mod:`soarm_sdk.protocol.packet_handler`.

Importing this module emits a :class:`DeprecationWarning`; it is scheduled
for removal in **0.3.0**.
"""

from __future__ import annotations

import warnings

from .protocol.packet_handler import *  # noqa: F401,F403
from .protocol.packet_handler import (  # noqa: F401
    ProtocolPacketHandler,
    protocol_packet_handler,
    PacketResult,
)

warnings.warn(
    "soarm_sdk.protocol_packet_handler is deprecated; import from soarm_sdk.protocol.packet_handler "
    "(or the top-level soarm_sdk namespace) instead. "
    "This shim will be removed in 0.3.0.",
    DeprecationWarning,
    stacklevel=2,
)
