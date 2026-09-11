"""Deprecated import path — use :mod:`soarm_sdk.robot.servo`.

Kept because downstream repos (soarm_tamp) import this module by its
old path directly: ``from soarm_sdk.servo_robot import ServoRobot``.
"""

from __future__ import annotations

from .robot.servo import *  # noqa: F401,F403
from .robot.servo import ServoRobot  # noqa: F401
