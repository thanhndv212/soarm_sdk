"""Feetech STS/SCS wire protocol: ports, packets, and sync read/write.

This is the lowest layer of soarm_sdk — serial framing, checksums, and the
per-model register maps (``sts`` for STS/SMS series, ``scscl`` for SCS
series). Nothing here knows about a specific robot's joint layout; that
lives one layer up, in :mod:`soarm_sdk.bus` and :mod:`soarm_sdk.robot`.

Most applications don't need to import from here directly — the commonly
used names are re-exported from the top-level ``soarm_sdk`` package.
"""

from __future__ import annotations

from .port_handler import PortHandler
from .packet_handler import ProtocolPacketHandler, protocol_packet_handler
from .group_sync_read import GroupSyncRead
from .group_sync_write import GroupSyncWrite
from .sts import sts
from .scscl import scscl

__all__ = [
    "PortHandler",
    "ProtocolPacketHandler",
    "protocol_packet_handler",
    "GroupSyncRead",
    "GroupSyncWrite",
    "sts",
    "scscl",
]
