"""Unit conversion utilities for STS3215 servo position and speed values.

The STS3215 encoder has 4096 ticks per revolution. The electrical midpoint
(default homing reference) is tick 2048, which maps to 0 radians by default.

Tick-to-radian mapping
----------------------
  angle_rad = (ticks - zero_offset) / TICKS_PER_REV * 2π

Speed mapping
-------------
  The GOAL_SPEED and PRESENT_SPEED registers are in ticks/second.
  rad_s = speed_ticks_per_sec * 2π / TICKS_PER_REV

soarm100 direction signs
------------------------
SOARM100_DIRECTION_SIGNS is a list of +1 or -1 per joint (index 0 = ID 1).
A sign of -1 inverts the axis so that positive radians always corresponds to
the robot's kinematic convention (increasing joint angle).

  **These defaults must be verified against the physical robot.**
  Flip individual signs if a joint moves in the wrong direction.
"""

from __future__ import annotations

import math
from typing import List

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

TICKS_PER_REV: int = 4096
"""Number of encoder ticks per full revolution (360°)."""

TICK_ZERO: int = 2048
"""Default electrical midpoint; maps to 0 radians unless overridden."""

TICKS_PER_RAD: float = TICKS_PER_REV / (2.0 * math.pi)
"""Encoder ticks per radian ≈ 651.9 ticks/rad."""

RADS_PER_TICK: float = (2.0 * math.pi) / TICKS_PER_REV
"""Radians per encoder tick ≈ 0.001534 rad/tick."""

# soarm100-specific direction signs (joint ID 1–6 → index 0–5).
# +1 : positive radian = increasing raw tick value.
# -1 : positive radian = decreasing raw tick value (mechanically inverted).
# Verify by commanding +0.1 rad on each joint and confirming physical motion.
SOARM100_DIRECTION_SIGNS: List[int] = [1, 1, 1, 1, 1, 1]

# ---------------------------------------------------------------------------
# Scalar helpers
# ---------------------------------------------------------------------------


def ticks_to_radians(ticks: int, zero_offset: int = TICK_ZERO) -> float:
    """Convert raw encoder ticks to radians relative to *zero_offset*.

    Parameters
    ----------
    ticks:
        Raw signed position value from ``ReadPos`` / ``ReadPosSpeed``.
    zero_offset:
        The tick value that corresponds to 0 rad. Defaults to ``TICK_ZERO``
        (2048), the electrical midpoint for a freshly homed STS3215.

    Returns
    -------
    float
        Angle in radians.
    """
    return (ticks - zero_offset) * RADS_PER_TICK


def radians_to_ticks(radians: float, zero_offset: int = TICK_ZERO) -> int:
    """Convert radians to raw encoder ticks, clamped to [0, 4095].

    Parameters
    ----------
    radians:
        Target angle in radians.
    zero_offset:
        The tick value corresponding to 0 rad. Defaults to ``TICK_ZERO``.

    Returns
    -------
    int
        Clamped encoder tick value in the range [0, 4095].
    """
    raw = round(radians * TICKS_PER_RAD + zero_offset)
    return max(0, min(TICKS_PER_REV - 1, raw))


def speed_ticks_to_rad_s(speed_ticks: int) -> float:
    """Convert servo speed (ticks/sec) to radians per second.

    Parameters
    ----------
    speed_ticks:
        Signed speed value from ``ReadSpeed`` / ``ReadPosSpeed``.
        Negative values indicate reverse rotation.

    Returns
    -------
    float
        Angular velocity in rad/s.
    """
    return speed_ticks * RADS_PER_TICK


def rad_s_to_speed_ticks(rad_s: float) -> int:
    """Convert radians per second to servo speed ticks.

    The result is signed; negative values indicate reverse rotation.
    Use ``abs()`` before writing to GOAL_SPEED if the servo register
    requires a separate direction flag.

    Parameters
    ----------
    rad_s:
        Angular velocity in rad/s.

    Returns
    -------
    int
        Signed speed in ticks/sec, clamped to ±3000 (servo max).
    """
    raw = round(rad_s * TICKS_PER_RAD)
    return max(-3000, min(3000, raw))


# ---------------------------------------------------------------------------
# soarm100 array helpers
# ---------------------------------------------------------------------------


def joint_ticks_to_radians(
    ticks_array: List[int],
    zero_offsets: List[int] | None = None,
    direction_signs: List[int] | None = None,
) -> List[float]:
    """Convert a list of per-joint tick values to radians for soarm100.

    Parameters
    ----------
    ticks_array:
        Raw tick values for joints 1–6 (list of 6 integers).
    zero_offsets:
        Per-joint zero tick value. Defaults to TICK_ZERO for every joint.
    direction_signs:
        Per-joint direction multiplier. Defaults to SOARM100_DIRECTION_SIGNS.

    Returns
    -------
    List[float]
        Joint angles in radians, length == len(ticks_array).
    """
    n = len(ticks_array)
    zeros = zero_offsets if zero_offsets is not None else [TICK_ZERO] * n
    signs = direction_signs if direction_signs is not None else SOARM100_DIRECTION_SIGNS[:n]
    return [
        signs[i] * ticks_to_radians(ticks_array[i], zeros[i])
        for i in range(n)
    ]


def joint_radians_to_ticks(
    radians_array: List[float],
    zero_offsets: List[int] | None = None,
    direction_signs: List[int] | None = None,
) -> List[int]:
    """Convert per-joint radians back to raw ticks for soarm100.

    Parameters
    ----------
    radians_array:
        Joint angles in radians, length 6.
    zero_offsets:
        Per-joint zero tick value. Defaults to TICK_ZERO for every joint.
    direction_signs:
        Per-joint direction multiplier. Defaults to SOARM100_DIRECTION_SIGNS.

    Returns
    -------
    List[int]
        Clamped encoder tick values in [0, 4095], length == len(radians_array).
    """
    n = len(radians_array)
    zeros = zero_offsets if zero_offsets is not None else [TICK_ZERO] * n
    signs = direction_signs if direction_signs is not None else SOARM100_DIRECTION_SIGNS[:n]
    return [
        radians_to_ticks(signs[i] * radians_array[i], zeros[i])
        for i in range(n)
    ]
