"""Which joint bounds a consumer should actually obey.

Two sources disagree, and the disagreement is not symmetric.

A URDF's ``<limit>`` is a *model's opinion* about the joint. On the SO-101 it
is conservative and, in places, simply wrong: the arm's measured travel is
wider than the model's on five of its six joints, and ``shoulder_lift``
reaches 26 deg past the model's lower limit. Nothing physical enforces it.

The measured travel is where the mechanism *stops*. It is a fact about the
hardware, and it is the bound a command must never be driven past.

So once the travel has been measured well enough to trust, it should replace
the URDF's opinion rather than be intersected with it — intersecting keeps
the conservative number, which is what made a planned trajectory get clamped
on 55% of its waypoints.

Why this is gated
-----------------
"Well enough to trust" is the whole problem. A single sweep of a joint whose
travel crosses the encoder's 4095/0 wrap measures as ``0..4095`` — the
*encoder's* range, not the joint's. Letting that replace the URDF's limits
would hand a planner a full turn of permission on a joint that stops well
short of one, and it would do so silently.

So the measured travel wins only when the ROM acceptance row has passed:
repeated, non-simulated endpoint measurements that agree to within the
per-arm tolerance. Until then the conservative intersection stands, which is
the behaviour every consumer had before this module existed.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

from .frame import RobotCalibration

__all__ = ["measured_is_trusted", "effective_limits"]


def measured_is_trusted(calibration: RobotCalibration) -> bool:
    """True when the ROM evidence is good enough to override a model's limits.

    Delegates to the acceptance pipeline rather than re-deciding here, so
    "the travel is trustworthy" means exactly one thing across the SDK.
    """
    from .pipeline import CalibrationPipeline, PipelineStage

    try:
        return CalibrationPipeline(calibration).report().stage(PipelineStage.ROM).passed
    except Exception:
        return False


def effective_limits(
    calibration: Optional[RobotCalibration],
    declared: Sequence[Tuple[float, float]],
    *,
    trust_measured: Optional[bool] = None,
) -> List[Tuple[float, float]]:
    """``(lo, hi)`` per joint, in URDF radians, that a consumer may command.

    *declared* is the model's own opinion — a URDF's limits, or a config's —
    in the calibration's joint order.

    With no calibration the declared bounds are all there is. With one whose
    travel is accepted, the measured bounds replace them outright. With one
    whose travel is not accepted, the two are intersected, which is strictly
    the more conservative of the two readings.

    *trust_measured* overrides the gate; pass it only to test a policy, never
    to skip the evidence.
    """
    declared = [(float(lo), float(hi)) for lo, hi in declared]
    if calibration is None:
        return declared
    measured = list(zip(*calibration.reachable_limits()))
    if len(measured) != len(declared):
        raise ValueError(
            f"calibration covers {len(measured)} joints, caller declared "
            f"{len(declared)} — they must describe the same arm"
        )
    trusted = (
        measured_is_trusted(calibration) if trust_measured is None else trust_measured
    )
    if trusted:
        return [(float(lo), float(hi)) for lo, hi in measured]
    return [
        (max(d_lo, m_lo), min(d_hi, m_hi))
        for (d_lo, d_hi), (m_lo, m_hi) in zip(declared, measured)
    ]
