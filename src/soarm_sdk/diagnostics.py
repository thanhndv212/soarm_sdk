"""Servo characterisation rigs built on the telemetry stream.

Two mechanical properties this arm has been bitten by, and what the servos can
and cannot tell you about each:

**Backlash** is fully observable from servo data. Drive to the same commanded
position from below and from above; the gap between the two settled positions
is lost motion in the gear train, and it needs no external reference because
both measurements come from the same encoder.

**Compliance under load** is only *partly* observable. What you get here is
steady-state position error — the servo commanded somewhere and stopping short
because the load exceeds what its position loop will push through. That is real
and worth measuring: it is why a loaded joint parks 0.03-0.05 rad off target.
What you do NOT get is deflection *downstream of the encoder* — a link flexing,
a horn twisting. The servo reports arrival while the tool is centimetres away,
and no amount of telemetry will see it. Closing that gap needs an external
reference (camera, tape measure); :func:`measure_droop` is the servo-side half,
and its result is a lower bound on true deflection.

Every rig here commands motion. Callers are responsible for the arm having room
to move, and for torque being enabled.
"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

from .conversions import RADS_PER_TICK

__all__ = [
    "BacklashResult",
    "DroopResult",
    "measure_backlash",
    "measure_droop",
]


@dataclass
class BacklashResult:
    """Lost motion at one joint, measured as approach-direction hysteresis."""

    joint_index: int
    target_rad: float
    from_below_rad: List[float] = field(default_factory=list)
    from_above_rad: List[float] = field(default_factory=list)

    @property
    def backlash_rad(self) -> float:
        """Mean settled position approaching from above minus from below.

        Positive means the joint stops short in whichever direction it came —
        the signature of lost motion. Near zero means the gear train takes up
        the same way regardless of approach.
        """
        if not self.from_below_rad or not self.from_above_rad:
            return float("nan")
        return statistics.fmean(self.from_above_rad) - statistics.fmean(
            self.from_below_rad
        )

    @property
    def backlash_ticks(self) -> float:
        return self.backlash_rad / RADS_PER_TICK

    @property
    def spread_rad(self) -> float:
        """Worst repeatability within a single approach direction.

        Backlash is only meaningful if it exceeds this: a hysteresis gap
        smaller than the scatter between repeats of the *same* approach is
        noise, not lost motion.
        """
        spreads = [
            max(vals) - min(vals)
            for vals in (self.from_below_rad, self.from_above_rad)
            if len(vals) > 1
        ]
        return max(spreads) if spreads else 0.0

    @property
    def significant(self) -> bool:
        """Whether the hysteresis gap stands clear of within-direction scatter."""
        gap = abs(self.backlash_rad)
        return bool(gap == gap and gap > self.spread_rad)  # NaN-safe

    def summary(self) -> str:
        if self.backlash_rad != self.backlash_rad:  # NaN
            return f"joint {self.joint_index}: no data"
        verdict = "significant" if self.significant else "within scatter"
        return (
            f"joint {self.joint_index}: backlash {self.backlash_rad:+.5f} rad "
            f"({self.backlash_ticks:+.1f} ticks), repeat scatter "
            f"{self.spread_rad:.5f} rad — {verdict}"
        )


@dataclass
class DroopResult:
    """Steady-state position error at one joint, against the load holding it."""

    joint_index: int
    commanded_rad: List[float] = field(default_factory=list)
    settled_rad: List[float] = field(default_factory=list)
    load_percent: List[float] = field(default_factory=list)
    current_mA: List[float] = field(default_factory=list)

    @property
    def error_rad(self) -> List[float]:
        """Commanded minus settled, per test point."""
        return [c - s for c, s in zip(self.commanded_rad, self.settled_rad)]

    @property
    def worst_error_rad(self) -> float:
        errs = self.error_rad
        return max(errs, key=abs) if errs else float("nan")

    def load_error_correlation(self) -> float:
        """Pearson r between |load| and |steady-state error|.

        A strong positive correlation is the signature of load-dependent
        compliance rather than a fixed offset — the input a gravity
        compensation term needs. Returns NaN if either series is constant.
        """
        if len(self.load_percent) < 2:
            return float("nan")
        x = np.abs(np.array(self.load_percent, dtype=float))
        y = np.abs(np.array(self.error_rad, dtype=float))
        if x.std() == 0 or y.std() == 0:
            return float("nan")
        return float(np.corrcoef(x, y)[0, 1])

    def summary(self) -> str:
        r = self.load_error_correlation()
        return (
            f"joint {self.joint_index}: worst steady-state error "
            f"{self.worst_error_rad:+.5f} rad over {len(self.settled_rad)} "
            f"points, load-vs-error r={r:.2f}"
        )


def _settle(hw, seconds: float, samples: int = 10):
    """Dwell, then return the mean of the last *samples* stream readings.

    Averaging rather than taking one reading matters: a servo at rest still
    dithers by a tick or two, and backlash here is a few ticks.
    """
    stream = hw.subscribe(maxlen=max(samples * 4, 64))
    try:
        time.sleep(seconds)
        got = stream.drain()
        if not got:
            raise RuntimeError(
                "no telemetry while settling — is the interface running?"
            )
        tail = got[-samples:]
        pos = np.mean([s.position_rad for s in tail], axis=0)
        load = np.mean([s.load_percent for s in tail], axis=0)
        curr = np.mean([s.current_mA for s in tail], axis=0)
        return pos, load, curr
    finally:
        hw.unsubscribe(stream)


def _command(hw, base_q: np.ndarray, joint_index: int, value_rad: float, speed):
    q = np.array(base_q, dtype=float)
    q[joint_index] = value_rad
    hw.set_robot_joint_positions(q, speed=speed)
    return q


def measure_backlash(
    hw,
    joint_index: int,
    *,
    excursion_rad: float = 0.20,
    cycles: int = 3,
    settle_s: float = 1.2,
    speed: Optional[int] = None,
    target_rad: Optional[float] = None,
) -> BacklashResult:
    """Measure lost motion at *joint_index* by approaching one target twice.

    Each cycle drives away by ``excursion_rad`` below the target and back to
    it, then away above and back. Both returns command the *same* position, so
    any difference in where the joint settles is approach-dependent — lost
    motion, not a control error.

    Torque must already be enabled, and the joint needs ``excursion_rad`` of
    clearance either side of the target.
    """
    base = hw.get_robot_joint_positions()
    target = float(base[joint_index]) if target_rad is None else float(target_rad)
    result = BacklashResult(joint_index=joint_index, target_rad=target)

    for _ in range(cycles):
        _command(hw, base, joint_index, target - excursion_rad, speed)
        _settle(hw, settle_s)
        _command(hw, base, joint_index, target, speed)
        pos, _, _ = _settle(hw, settle_s)
        result.from_below_rad.append(float(pos[joint_index]))

        _command(hw, base, joint_index, target + excursion_rad, speed)
        _settle(hw, settle_s)
        _command(hw, base, joint_index, target, speed)
        pos, _, _ = _settle(hw, settle_s)
        result.from_above_rad.append(float(pos[joint_index]))

    return result


def measure_droop(
    hw,
    joint_index: int,
    offsets_rad: Sequence[float] = (-0.3, -0.15, 0.0, 0.15, 0.3),
    *,
    settle_s: float = 1.5,
    speed: Optional[int] = None,
) -> DroopResult:
    """Command *joint_index* through several poses and record where it stops.

    Each offset is applied relative to the joint's current position, so the
    sweep is centred wherever the arm already is. At each point the settled
    position, the load holding it, and the current are recorded together —
    they come from the same telemetry samples, so they are aligned by
    construction.

    The steady-state error this returns is a **lower bound** on true tool
    deflection: it cannot see compliance downstream of the encoder.
    """
    base = hw.get_robot_joint_positions()
    centre = float(base[joint_index])
    result = DroopResult(joint_index=joint_index)

    for offset in offsets_rad:
        commanded = centre + float(offset)
        _command(hw, base, joint_index, commanded, speed)
        pos, load, curr = _settle(hw, settle_s)
        result.commanded_rad.append(commanded)
        result.settled_rad.append(float(pos[joint_index]))
        result.load_percent.append(float(load[joint_index]))
        result.current_mA.append(float(curr[joint_index]))

    _command(hw, base, joint_index, centre, speed)
    _settle(hw, settle_s)
    return result
