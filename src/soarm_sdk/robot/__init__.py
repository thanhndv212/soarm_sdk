"""The robot abstraction layer: types, the RobotInterface Protocol, and backends.

This is the single boundary between *algorithm code* (planners, teleop,
RL policies) and *hardware code*. Everything above this layer should speak
:class:`RobotInterface` / :class:`Robot`, not a specific backend.

Backends
--------
- :class:`ServoRobot` — real STS3215 hardware over RS-485.
- :class:`LeRobotRobot` — the same bus via lerobot's ``SOFollower``
  (optional dependency: ``pip install soarm-sdk[lerobot]``).
- :class:`NullRobot` — no hardware; tracks commanded state in memory.

A simulation backend elsewhere in the workspace (mjlab, a MuJoCo wrapper,
...) can satisfy :class:`RobotInterface` structurally without depending on
soarm_sdk at all — see :mod:`soarm_sdk.robot.interfaces`.
"""

from __future__ import annotations

from .telemetry import ServoSample, TelemetryStream
from .telemetry_sinks import (
    JsonlSink,
    RerunSink,
    TelemetryRecorder,
    TelemetrySink,
)
from .types import Pose, JointState, ServoHealth
from .interfaces import RobotInterface
from .config import ConfigError, validate_robot_config
from .base import Robot, load_robot_config
from .hardware import ServoHardwareInterface
from .servo import ServoRobot
from .null import NullRobot
from .lerobot import LeRobotRobot

__all__ = [
    "Pose",
    "JointState",
    "ServoHealth",
    "ServoSample",
    "TelemetryStream",
    "TelemetrySink",
    "TelemetryRecorder",
    "JsonlSink",
    "RerunSink",
    "RobotInterface",
    "ConfigError",
    "validate_robot_config",
    "Robot",
    "load_robot_config",
    "ServoHardwareInterface",
    "ServoRobot",
    "NullRobot",
    "LeRobotRobot",
]
