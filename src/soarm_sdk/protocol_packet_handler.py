"""Deprecated import path — use :mod:`soarm_sdk.protocol.packet_handler`."""

from __future__ import annotations

from .protocol.packet_handler import *  # noqa: F401,F403
from .protocol.packet_handler import (  # noqa: F401
    ProtocolPacketHandler,
    protocol_packet_handler,
    PacketResult,
)
