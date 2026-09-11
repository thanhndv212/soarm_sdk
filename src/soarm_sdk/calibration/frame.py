"""Mapping between raw servo ticks and the URDF's joint frame.

Distinct from :mod:`soarm_sdk.calibration`, which reconfigures servos (IDs,
angle limits, speed, torque, baud). This module answers a different
question: *what tick value means zero radians to the URDF?*

Why it exists
-------------
Three joint-angle conventions are in play on this arm and no two of them
agree:

1. **Raw ticks.** ``Present_Position``, 0..4095. lerobot writes a
   ``Homing_Offset`` into each servo's EEPROM, so the servo itself has
   already applied that correction before the value reaches the bus.
2. **lerobot normalized.** Zero is the midpoint of the calibrated travel
   range, per its calibration JSON.
3. **The URDF's kinematic zero.** A specific pose — arm straight out,
   TCP at (0.391, 0, 0.227) m. This is the one a motion planner speaks.

``ServoHardwareInterface`` has always accepted ``zero_offsets`` and
``direction_signs``, but nothing ever measured them: the package config
ships a static 2048 per joint, which is a fourth convention again. This
module produces those numbers, ties them to the URDF, and persists them.

Seeding, and why the result is not trusted yet
----------------------------------------------
:func:`seed_from_travel` solves for the zero offset from two facts that are
already known: the measured travel range (which ends at the mechanical hard
stops) and the URDF's joint limits (which describe those same stops in
radians). Each end of the range gives an independent estimate of the zero;
their disagreement is recorded as ``seed_residual_rad`` and is the honest
uncertainty of the seed.

**The direction sign cannot be recovered this way.** A travel range is just
a min and a max — it does not say which physical end corresponds to the
URDF's lower limit. Seeding therefore assumes +1 and marks the calibration
unvalidated. Only a physical check (drive one joint a little and watch
which way it goes) can settle it, which is why :attr:`RobotCalibration.
validated` starts ``False`` and why callers should refuse to stream a
planned trajectory until it is ``True``.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..conversions import RADS_PER_TICK, TICKS_PER_RAD

__all__ = [
    "JointCalibration",
    "RobotCalibration",
    "seed_from_travel",
    "seed_from_lerobot",
]

# A span ratio this far from 1.0 means the measured travel and the URDF
# limits disagree about how big the joint's range is. The gearing is fixed
# at 4096 ticks/rev, so the discrepancy is in the URDF's limits (usually
# conservative) or in how the range was captured — not in the conversion.
# Worth surfacing rather than silently averaging away.
SPAN_RATIO_TOLERANCE = 0.10


@dataclass(frozen=True)
class JointCalibration:
    """Tick-to-URDF-radian mapping for one joint.

    ``rad = direction_sign * (ticks - zero_offset_ticks) * RADS_PER_TICK``
    """

    name: str
    zero_offset_ticks: float
    direction_sign: int
    tick_min: int
    tick_max: int
    # Half the disagreement between the two endpoint estimates of the zero.
    # This is the seed's uncertainty, not a measurement error.
    seed_residual_rad: float = 0.0
    # measured travel / URDF travel. Should be ~1.0; see SPAN_RATIO_TOLERANCE.
    span_ratio: float = 1.0

    def to_rad(self, ticks: float) -> float:
        return self.direction_sign * (ticks - self.zero_offset_ticks) * RADS_PER_TICK

    def to_ticks(self, rad: float) -> float:
        return self.zero_offset_ticks + self.direction_sign * rad * TICKS_PER_RAD

    @property
    def suspect(self) -> bool:
        """True when the span ratio says the two sources disagree materially."""
        return abs(self.span_ratio - 1.0) > SPAN_RATIO_TOLERANCE


@dataclass
class RobotCalibration:
    """Per-arm calibration. Persisted as JSON; one file per physical arm."""

    joints: List[JointCalibration]
    arm_id: str = "unknown"
    #: False until a physical check has confirmed the direction signs.
    #: Callers streaming a planned trajectory should refuse while False.
    validated: bool = False
    source: str = "seeded"
    created: str = ""
    notes: Dict[str, Any] = field(default_factory=dict)

    # -- conversion ----------------------------------------------------

    @property
    def names(self) -> List[str]:
        return [j.name for j in self.joints]

    @property
    def zero_offsets(self) -> List[float]:
        """In the shape ``ServoHardwareInterface`` already accepts."""
        return [j.zero_offset_ticks for j in self.joints]

    @property
    def direction_signs(self) -> List[int]:
        return [j.direction_sign for j in self.joints]

    def ticks_to_rad(self, ticks: Sequence[float]) -> List[float]:
        if len(ticks) != len(self.joints):
            raise ValueError(f"expected {len(self.joints)} values, got {len(ticks)}")
        return [j.to_rad(t) for j, t in zip(self.joints, ticks)]

    def rad_to_ticks(self, rad: Sequence[float]) -> List[float]:
        if len(rad) != len(self.joints):
            raise ValueError(f"expected {len(self.joints)} values, got {len(rad)}")
        return [j.to_ticks(r) for j, r in zip(self.joints, rad)]

    @property
    def suspect_joints(self) -> List[str]:
        return [j.name for j in self.joints if j.suspect]

    @property
    def worst_seed_residual_rad(self) -> float:
        return max((j.seed_residual_rad for j in self.joints), default=0.0)

    # -- persistence ---------------------------------------------------

    def save(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        data = asdict(self)
        data["created"] = self.created or datetime.now(timezone.utc).isoformat()
        p.write_text(json.dumps(data, indent=2) + "\n")
        return p

    @classmethod
    def load(cls, path: str | Path) -> "RobotCalibration":
        data = json.loads(Path(path).read_text())
        joints = [JointCalibration(**j) for j in data.pop("joints")]
        return cls(joints=joints, **data)

    def mark_validated(self, how: str) -> None:
        """Record that a physical check confirmed this calibration.

        *how* should say what was actually done — the signs and the zero are
        only as good as the check behind them, and a year from now the file
        is the only record of it.
        """
        self.validated = True
        self.notes["validated_by"] = how
        self.notes["validated_at"] = datetime.now(timezone.utc).isoformat()


def seed_from_travel(
    names: Sequence[str],
    urdf_limits: Sequence[Tuple[float, float]],
    tick_ranges: Sequence[Tuple[int, int]],
    *,
    arm_id: str = "unknown",
    source: str = "seeded from travel range + URDF limits",
) -> RobotCalibration:
    """Estimate a calibration from measured travel and URDF joint limits.

    Both describe the same mechanical hard stops, so each end of the range
    gives an estimate of the zero tick. They will not agree exactly — the
    URDF's limits are approximate — and half that disagreement is kept per
    joint as ``seed_residual_rad``.

    The direction sign is assumed +1 throughout; see the module docstring
    for why a travel range cannot determine it. The returned calibration is
    ``validated=False``.
    """
    if not (len(names) == len(urdf_limits) == len(tick_ranges)):
        raise ValueError("names, urdf_limits and tick_ranges must be the same length")

    joints: List[JointCalibration] = []
    for name, (u_lo, u_hi), (t_min, t_max) in zip(names, urdf_limits, tick_ranges):
        if u_hi <= u_lo:
            raise ValueError(f"{name}: URDF upper limit must exceed lower")
        if t_max <= t_min:
            raise ValueError(f"{name}: tick max must exceed min")

        # Each endpoint pins the zero independently, assuming sign +1.
        zero_from_lo = t_min - u_lo * TICKS_PER_RAD
        zero_from_hi = t_max - u_hi * TICKS_PER_RAD
        zero = (zero_from_lo + zero_from_hi) / 2.0
        residual = abs(zero_from_lo - zero_from_hi) / 2.0 * RADS_PER_TICK

        measured_span = (t_max - t_min) * RADS_PER_TICK
        joints.append(
            JointCalibration(
                name=name,
                zero_offset_ticks=zero,
                direction_sign=1,
                tick_min=int(t_min),
                tick_max=int(t_max),
                seed_residual_rad=residual,
                span_ratio=measured_span / (u_hi - u_lo),
            )
        )

    return RobotCalibration(
        joints=joints,
        arm_id=arm_id,
        validated=False,
        source=source,
        created=datetime.now(timezone.utc).isoformat(),
        notes={
            "direction_signs": "ASSUMED +1 — not derivable from a travel range",
            "next_step": "confirm signs and zero physically, then mark_validated()",
        },
    )


def seed_from_lerobot(
    calibration_json: str | Path,
    urdf_limits: Dict[str, Tuple[float, float]],
    *,
    arm_id: Optional[str] = None,
) -> RobotCalibration:
    """Seed from a lerobot calibration file plus the URDF's joint limits.

    Reads only ``range_min``/``range_max`` — the measured travel. The file's
    ``homing_offset`` is deliberately **not** used: lerobot writes it into
    the servo's EEPROM, so the ticks this SDK reads already have it applied,
    and subtracting it again would double-count. ``drive_mode`` is likewise
    lerobot's own convention, not a statement about the URDF.

    Joint order follows *urdf_limits*, so the caller controls it rather than
    inheriting whatever order the JSON happened to serialise in.
    """
    p = Path(calibration_json)
    data = json.loads(p.read_text())
    missing = [n for n in urdf_limits if n not in data]
    if missing:
        raise KeyError(f"{p.name} has no entry for: {', '.join(missing)}")

    names = list(urdf_limits)
    cal = seed_from_travel(
        names=names,
        urdf_limits=[urdf_limits[n] for n in names],
        tick_ranges=[(data[n]["range_min"], data[n]["range_max"]) for n in names],
        arm_id=arm_id or p.stem,
        source=f"seeded from {p.name} travel ranges + URDF limits",
    )
    cal.notes["lerobot_file"] = str(p)
    cal.notes["lerobot_drive_modes"] = {n: data[n].get("drive_mode") for n in names}
    cal.notes["homing_offset_note"] = (
        "not applied — lerobot writes it to servo EEPROM, so raw ticks "
        "already include it"
    )
    return cal
