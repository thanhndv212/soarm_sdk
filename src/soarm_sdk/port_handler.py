"""Deprecated import path — use :mod:`soarm_sdk.protocol.port_handler`.

Kept so ``from soarm_sdk.port_handler import PortHandler`` keeps working
after the protocol layer moved under :mod:`soarm_sdk.protocol`.
"""

from __future__ import annotations

from .protocol.port_handler import *  # noqa: F401,F403
from .protocol.port_handler import PortHandler  # noqa: F401
