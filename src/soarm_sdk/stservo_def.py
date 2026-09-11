"""Deprecated import path — use :mod:`soarm_sdk.protocol.registers`.

``stservo_def`` was the STS/SCS register-map module's original name; it
moved to ``protocol/registers.py`` when the wire-protocol layer was split
out of the flat top-level namespace. This shim keeps every register
constant (``STS_*``, ``COMM_*``, ...) importable from its old path.
"""

from __future__ import annotations

from .protocol.registers import *  # noqa: F401,F403
