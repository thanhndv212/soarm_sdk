"""Live Rerun-based telemetry monitoring for the SO-101 servo bus.

Optional: only imported by ``soarm-monitor`` (:mod:`soarm_sdk.cli.monitor`),
which requires the ``telemetry`` extra (``pip install soarm-sdk[telemetry]``).
Nothing else in the package imports this subpackage.
"""

from __future__ import annotations

from .blueprint import (
    DEFAULT_FIELDS,
    build_by_channel_blueprint,
    build_by_servo_blueprint,
    build_monitor_blueprint,
)

__all__ = [
    "DEFAULT_FIELDS",
    "build_by_servo_blueprint",
    "build_by_channel_blueprint",
    "build_monitor_blueprint",
]
