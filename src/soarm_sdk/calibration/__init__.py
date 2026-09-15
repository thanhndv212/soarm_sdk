"""Mapping raw servo ticks to the URDF's joint frame — measurement, seeding, and storage.

Distinct from :mod:`soarm_sdk.bus.servo_config` (formerly the flat
``soarm_sdk.calibration`` module), which reconfigures servo EEPROM — IDs,
angle limits, speed, torque, baud. This package answers a different
question: *what tick value means zero radians to the URDF?*

- :mod:`soarm_sdk.calibration.frame` — :class:`RobotCalibration`, the
  persisted tick<->radian mapping, and :func:`seed_from_travel`.
- :mod:`soarm_sdk.calibration.rom_sweep` — the range-of-motion sweep that
  produces the measured travel :func:`~frame.seed_from_travel` consumes
  (the ``soarm-calibrate-rom`` CLI). Seeding from an existing lerobot
  calibration file instead (``soarm-seed-calibration``) was removed —
  a travel range borrowed from another tool's file is not a measurement
  of this arm, and the guided ``soarm-dashboard-calibration`` workflow is
  the one path this package now supports.
- :mod:`soarm_sdk.calibration.reference` — the named physical poses a zero
  can be pinned to, checked against the URDF rather than described in prose.
"""

from __future__ import annotations

from .frame import (
    JointCalibration,
    RobotCalibration,
    rezero_from_pose,
    seed_from_travel,
)
from .reference import REFERENCE_POSES, ReferencePose
from .rom_sweep import run_rom_sweep, simulate_rom_sweep
from .pipeline import AcceptanceTolerances, CalibrationPipeline, CalibrationReport

__all__ = [
    "JointCalibration",
    "RobotCalibration",
    "rezero_from_pose",
    "seed_from_travel",
    "REFERENCE_POSES",
    "ReferencePose",
    "run_rom_sweep",
    "simulate_rom_sweep",
    "AcceptanceTolerances",
    "CalibrationPipeline",
    "CalibrationReport",
]
