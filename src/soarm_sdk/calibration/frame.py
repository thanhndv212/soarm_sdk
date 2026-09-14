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
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..conversions import RADS_PER_TICK, TICKS_PER_RAD

__all__ = [
    "JointCalibration",
    "RobotCalibration",
    "rezero_from_pose",
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
    #: Where this joint's zero came from. Decides what ``span_ratio`` implies:
    #: a zero *derived from* the URDF's limits is only as good as they are, so
    #: a span mismatch impeaches it. A zero pinned to a physically verified
    #: pose does not depend on those limits at all, and the same mismatch then
    #: says only that the URDF is conservative about travel.
    #: One of ``travel_and_urdf_limits``, ``reference_pose``, ``unknown``.
    zero_source: str = "unknown"

    def to_rad(self, ticks: float) -> float:
        return self.direction_sign * (ticks - self.zero_offset_ticks) * RADS_PER_TICK

    def to_ticks(self, rad: float) -> float:
        return self.zero_offset_ticks + self.direction_sign * rad * TICKS_PER_RAD

    def shifted_by(self, delta_rad: float) -> "JointCalibration":
        """Copy of this joint whose reported angle moves by *delta_rad*.

        Moves the zero, not the reading: the servo still reports the same
        ticks, and this changes what those ticks are taken to mean. Used to
        dial the model onto an arm the operator can see — nudge until the
        rendered member lies where the real one does, then save.

        The shift is in URDF radians and signed in the URDF's frame, so the
        caller does not have to think about ``direction_sign``; it is folded
        in here, the same way :meth:`to_ticks` folds it in.
        """
        return replace(
            self,
            zero_offset_ticks=self.zero_offset_ticks
            - self.direction_sign * delta_rad * TICKS_PER_RAD,
            zero_source="manual_nudge",
        )

    def with_direction_flipped(self) -> "JointCalibration":
        """Copy of this joint whose reported angle increases the other way.

        The fix when the model turns *opposite* to the member: no zero can
        repair a wrong sign, because the sign is not an offset.

        The zero *tick* is kept — which tick reads zero does not move — but
        any zero solved for under the old sign is now wrong:
        :func:`rezero_from_pose` computes ``zero = ticks - sign * rad``, so
        the sign is folded into it. The provenance is therefore dropped
        rather than carried, and the joint has to be re-pinned against a
        pose before the calibration can be accepted again.
        """
        return replace(
            self,
            direction_sign=-self.direction_sign,
            zero_source="manual_sign_flip",
        )

    def with_travel(self, tick_min: int, tick_max: int) -> "JointCalibration":
        """Copy carrying freshly measured hard stops.

        The travel is a fact about the mechanism and independent of the zero,
        so nothing else moves: the same ticks still mean the same angles.
        """
        lo, hi = int(min(tick_min, tick_max)), int(max(tick_min, tick_max))
        return replace(self, tick_min=lo, tick_max=hi)

    def with_claim_withdrawn(self) -> "JointCalibration":
        """Copy whose zero stops claiming a reference-pose witness.

        For a joint labelled ``reference_pose`` that no recorded pose
        actually constrains — the claim is false, and the only honest repair
        is to drop it. Becomes ``unknown`` rather than
        ``travel_and_urdf_limits``: how the zero was really derived is not
        recoverable from the file, and ``unknown`` is treated as
        limits-derived by :attr:`suspect`, which is the conservative reading.

        Downgrade-only by construction. It can remove a claim and never add
        one, so it cannot be turned into a way to launder a zero into looking
        pose-anchored.
        """
        if self.zero_source != "reference_pose":
            return self
        return replace(self, zero_source="unknown")

    @property
    def span_mismatch(self) -> bool:
        """True when measured travel and the URDF's limits disagree materially.

        A fact about the two sources, independent of how the zero was found.
        """
        return abs(self.span_ratio - 1.0) > SPAN_RATIO_TOLERANCE

    @property
    def suspect(self) -> bool:
        """True when the span mismatch actually impeaches this joint's zero.

        Only when the zero was inferred from the URDF's limits. A zero pinned
        to a verified pose is unaffected by them being wrong, so the same
        mismatch is informational there rather than disqualifying — see
        :func:`rezero_from_pose`. ``unknown`` provenance is treated as
        limits-derived, since that is what every calibration written before
        this field existed was.
        """
        return self.span_mismatch and self.zero_source != "reference_pose"

    @property
    def reachable_rad(self) -> Tuple[float, float]:
        """``(lo, hi)`` in URDF radians that this joint can physically reach.

        The measured tick travel — the mechanical hard stops — expressed in
        the URDF's frame. Ordered, because a negative ``direction_sign``
        swaps which tick endpoint is the larger angle.

        This is the *hardware* bound. It is not a model's opinion about the
        joint: whatever a URDF or a config says, the servo stops here.
        """
        a, b = self.to_rad(self.tick_min), self.to_rad(self.tick_max)
        return (a, b) if a <= b else (b, a)


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

    def reachable_limits(self) -> Tuple[List[float], List[float]]:
        """``(lower, upper)`` per joint, in URDF radians, from measured travel.

        The arm's real hard stops. Intersect a model's declared limits with
        this before enforcing them, so a command can never be driven past
        what the mechanism actually allows — see
        :meth:`~soarm_sdk.robot.servo.ServoRobot.connect`.
        """
        pairs = [j.reachable_rad for j in self.joints]
        return [lo for lo, _ in pairs], [hi for _, hi in pairs]

    @property
    def suspect_joints(self) -> List[str]:
        """Joints whose zero is impeached by a travel/URDF span mismatch."""
        return [j.name for j in self.joints if j.suspect]

    @property
    def span_mismatch_joints(self) -> List[str]:
        """Joints whose measured travel disagrees with the URDF, zero aside.

        Worth reporting even when the zero is sound: it means the URDF's
        limits are not the arm's real reach, so plan against
        :meth:`reachable_limits` rather than the model's own numbers.
        """
        return [j.name for j in self.joints if j.span_mismatch]

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


def rezero_from_pose(
    calibration: "RobotCalibration",
    ticks: Sequence[float],
    reference_rad: Optional[Sequence[float]] = None,
    *,
    only: Optional[Sequence[str]] = None,
    source: str = "re-zeroed from a physically held reference pose",
) -> "RobotCalibration":
    """Recompute zero offsets from ticks measured at a *known* configuration.

    :func:`seed_from_travel` infers the zero by matching the ends of measured
    travel to the URDF's joint limits, on the stated assumption that both
    describe the same mechanical hard stops. When they do not — when the URDF
    limits are conservative software limits and the real travel is wider — that
    inference is stretched across the disagreement and every zero lands off by
    a share of it. ``span_ratio`` is what measures the disagreement; anything
    far from 1.0 means the seeded zero cannot be trusted.

    This takes the other route: hold the arm at a configuration you can verify
    physically (a level, a straight edge, a hard stop you trust), read the
    ticks there, and pin the zeros to that. No dependence on the URDF's limits
    at all.

    *reference_rad* defaults to all zeros, which is the URDF's kinematic zero
    and **not** a pose anyone can hold the arm in by eye: there the upper arm
    sits at +76.03 deg and the forearm at +2.21 deg. The pose that *is*
    checkable — upper arm vertical, forearm level — is
    :data:`soarm_sdk.calibration.reference.LEVEL`, at
    ``(0, -0.2438, +0.2823, -0.0881, 0, 0)``. Pass one of
    :data:`~soarm_sdk.calibration.reference.REFERENCE_POSES` rather than
    relying on the default; an earlier version of this docstring claimed the
    default *was* the level pose, and re-zeroing on that claim puts
    ``shoulder_lift`` 13.97 deg and ``elbow_flex`` 16.17 deg out, in opposite
    directions.

    Measured travel is carried over unchanged, so :meth:`reachable_rad` still
    reports the real hard stops, and ``span_ratio`` is preserved as the record
    that they disagree with the URDF.

    *only* restricts the re-zero to the named joints, leaving the rest exactly
    as they were. A reference pose constrains the joints it visibly constrains
    and no others — standing the arm up level says nothing about where
    ``wrist_roll`` is, and pinning its zero to "whatever it happened to read"
    would replace a seeded guess with a differently-wrong one while relabelling
    it ``reference_pose``, i.e. trustworthy. Pass
    :attr:`~soarm_sdk.calibration.reference.ReferencePose.covers`.

    The result is ``validated=False``: pinning the zero to a pose you believe
    in is not the same as confirming it, and the direction signs are inherited
    rather than re-measured.
    """
    n = len(calibration.joints)
    if len(ticks) != n:
        raise ValueError(f"expected {n} tick values, got {len(ticks)}")
    ref = [0.0] * n if reference_rad is None else list(reference_rad)
    if len(ref) != n:
        raise ValueError(f"expected {n} reference angles, got {len(ref)}")
    if only is not None:
        chosen = set(only)
        unknown = chosen - {j.name for j in calibration.joints}
        if unknown:
            raise KeyError(f"not joints of this arm: {', '.join(sorted(unknown))}")
    else:
        chosen = {j.name for j in calibration.joints}

    joints = [
        replace(
            j,
            # to_rad(t) = sign * (t - zero) * RADS_PER_TICK, so pinning
            # to_rad(measured) == reference inverts to this.
            zero_offset_ticks=t - j.direction_sign * q * TICKS_PER_RAD,
            seed_residual_rad=0.0,
            zero_source="reference_pose",
        )
        if j.name in chosen
        else j
        for j, t, q in zip(calibration.joints, ticks, ref)
    ]
    return RobotCalibration(
        joints=joints,
        arm_id=calibration.arm_id,
        validated=False,
        source=source,
        notes=dict(calibration.notes),
    )


#: Per-joint override to the +1 assumed when no direction sign is supplied.
#:
#: A travel range alone cannot establish a sign — see the module docstring —
#: so this is not derived from anything measured here. It exists because a
#: physical direction-sign check on this hardware (2026-09-12, arm
#: ``thanh_arm``) found ``wrist_roll`` turning opposite the URDF's
#: convention while every other checked joint matched at +1. That is a
#: property of the SO-101's assembly/URDF pairing, not of one physical unit,
#: so it is used as the starting assumption for every SO-101 rather than
#: re-discovered per arm — subject to revision if a check on a different
#: unit disagrees.
#:
#: Still just an assumption: :func:`seed_from_travel` always returns
#: ``validated=False``, whether a sign came from here or from the +1
#: fallback, and Step 2's physical check is what actually certifies it.
DEFAULT_DIRECTION_SIGN_OVERRIDES: Dict[str, int] = {"wrist_roll": -1}


def seed_from_travel(
    names: Sequence[str],
    urdf_limits: Sequence[Tuple[float, float]],
    tick_ranges: Sequence[Tuple[int, int]],
    *,
    direction_signs: Optional[Sequence[int]] = None,
    arm_id: str = "unknown",
    source: str = "seeded from travel range + URDF limits",
) -> RobotCalibration:
    """Estimate a calibration from measured travel and URDF joint limits.

    Both describe the same mechanical hard stops, so each end of the range
    gives an estimate of the zero tick. They will not agree exactly — the
    URDF's limits are approximate — and half that disagreement is kept per
    joint as ``seed_residual_rad``.

    *direction_signs* defaults to :data:`DEFAULT_DIRECTION_SIGN_OVERRIDES` —
    +1 for every joint except ``wrist_roll``, which is all a travel range
    alone can offer even with that override; see the module docstring for
    why a range can't establish a sign at all. Pass measured signs once a
    physical check has established them: the sign is not a cosmetic flag on
    top of the same zero, because it decides *which* end of the travel is
    the URDF's lower limit, and so changes the zero the two endpoints agree
    on. Flipping the field alone would leave the joint mirrored about the
    wrong point.

    The returned calibration is ``validated=False`` regardless; supplying
    signs — or matching the override — records what was measured or
    assumed, it does not certify it. A physical check in Step 2 is still
    required before ``mark_validated()``.
    """
    if not (len(names) == len(urdf_limits) == len(tick_ranges)):
        raise ValueError("names, urdf_limits and tick_ranges must be the same length")
    signs = (
        list(direction_signs)
        if direction_signs is not None
        else [DEFAULT_DIRECTION_SIGN_OVERRIDES.get(n, 1) for n in names]
    )
    if len(signs) != len(names):
        raise ValueError("direction_signs must have one entry per joint")
    if any(s not in (1, -1) for s in signs):
        raise ValueError("every direction_sign must be +1 or -1")

    joints: List[JointCalibration] = []
    for name, (u_lo, u_hi), (t_min, t_max), sign in zip(
        names, urdf_limits, tick_ranges, signs
    ):
        if u_hi <= u_lo:
            raise ValueError(f"{name}: URDF upper limit must exceed lower")
        if t_max <= t_min:
            raise ValueError(f"{name}: tick max must exceed min")

        # Each endpoint pins the zero independently. Which URDF limit a tick
        # endpoint corresponds to depends on the sign: at +1 the smallest tick
        # is the lower limit, at -1 it is the upper one.
        at_t_min, at_t_max = (u_lo, u_hi) if sign > 0 else (u_hi, u_lo)
        zero_from_lo = t_min - sign * at_t_min * TICKS_PER_RAD
        zero_from_hi = t_max - sign * at_t_max * TICKS_PER_RAD
        zero = (zero_from_lo + zero_from_hi) / 2.0
        residual = abs(zero_from_lo - zero_from_hi) / 2.0 * RADS_PER_TICK

        measured_span = (t_max - t_min) * RADS_PER_TICK
        joints.append(
            JointCalibration(
                name=name,
                zero_offset_ticks=zero,
                direction_sign=sign,
                tick_min=int(t_min),
                tick_max=int(t_max),
                seed_residual_rad=residual,
                span_ratio=measured_span / (u_hi - u_lo),
                zero_source="travel_and_urdf_limits",
            )
        )

    return RobotCalibration(
        joints=joints,
        arm_id=arm_id,
        validated=False,
        source=source,
        created=datetime.now(timezone.utc).isoformat(),
        notes={
            "direction_signs": (
                "ASSUMED (+1, except wrist_roll -1 — see "
                "DEFAULT_DIRECTION_SIGN_OVERRIDES) — not derivable from a "
                "travel range"
                if direction_signs is None
                else "supplied by the caller from a physical check"
            ),
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
