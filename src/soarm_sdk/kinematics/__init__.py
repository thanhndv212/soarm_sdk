"""URDF-based kinematics helpers with no dependency on any viewer or planner.

This is separated from :mod:`soarm_sdk.dashboard` so forward kinematics
can be used by anything that needs it (tests, a planner, an
``fk_fn`` passed to :class:`~soarm_sdk.robot.servo.ServoRobot`) without
pulling in Viser.
"""

from __future__ import annotations

from .urdf_fk import URDF_AVAILABLE, mat3_to_wxyz, load_urdf, link_transforms

__all__ = ["URDF_AVAILABLE", "mat3_to_wxyz", "load_urdf", "link_transforms"]
