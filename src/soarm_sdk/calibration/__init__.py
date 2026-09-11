"""Mapping raw servo ticks to the URDF's joint frame — measurement, seeding, and storage.

Distinct from :mod:`soarm_sdk.bus.servo_config` (formerly the flat
``soarm_sdk.calibration`` module), which reconfigures servo EEPROM — IDs,
angle limits, speed, torque, baud. This package answers a different
question: *what tick value means zero radians to the URDF?*

- :mod:`soarm_sdk.calibration.frame` — :class:`RobotCalibration`, the
  persisted tick<->radian mapping, and :func:`seed_from_travel`.
- :mod:`soarm_sdk.calibration.seed` — seeds a calibration offline from an
  existing lerobot calibration file (``python -m soarm_sdk.seed_calibration``).
- :mod:`soarm_sdk.calibration.rom_sweep` — the range-of-motion sweep that
  produces the measured travel :func:`~frame.seed_from_travel` consumes.
"""

from __future__ import annotations

from .frame import JointCalibration, RobotCalibration, seed_from_travel, seed_from_lerobot
from .rom_sweep import run_rom_sweep, simulate_rom_sweep

__all__ = [
    "JointCalibration",
    "RobotCalibration",
    "seed_from_travel",
    "seed_from_lerobot",
    "run_rom_sweep",
    "simulate_rom_sweep",
]
