"""Named physical poses a zero can be pinned to.

:func:`~soarm_sdk.calibration.frame.rezero_from_pose` needs two things
that must describe the same instant: ticks read off the servos, and the
joint configuration those ticks are *supposed* to mean. The ticks come
from the bus. The configuration has, until now, come from whatever the
operator typed at the time — the 2026-09-13 re-zero on ``thanh_arm``
recorded its reference only as a sentence in a ``notes`` field.

That is the failure mode this module exists to close. A reference pose is
a claim about geometry ("the upper arm is vertical and the forearm is
level"), and a claim about geometry can be *checked against the URDF*
rather than asserted. Here each pose carries only its configuration; the
prose description is derived from the model by
:func:`~soarm_sdk.kinematics.urdf_fk.member_pitches`, and
``tests/test_reference_poses.py`` asserts the two still agree. A pose
whose description drifts from its numbers fails a test instead of
quietly biasing every zero pinned to it.

The bug that motivated it, and the bug in the fix
-------------------------------------------------
``rezero_from_pose`` documented its all-zeros default as "for the SO-101
that is the upper arm vertical and the forearm horizontal". This module
was written to say that was false — that the URDF zero put the upper arm
at +76.03 deg and the forearm at +2.21 deg — and :data:`LEVEL` was solved
to be the pose the docstring had described.

**That correction was wrong, and the original docstring was right.** Both
numbers came from :data:`~soarm_sdk.kinematics.urdf_fk.MEMBERS`, which
measured a member as the chord between two joint-frame *origins*. On the
SO-101 those origins sit off the upper arm's long axis, so the chord runs
about 14 deg away from the body an operator actually lays a level on.
Measured along the body instead, the URDF zero is upper arm **+89.90 deg**
and forearm **+0.00 deg** — vertical and level, exactly as first written.

So every pose here had been solved to make the wrong line come out
straight, and was off by that link's chord-to-body offset:
``folded_flat`` rendered with its upper arm visibly sloped while
reporting itself level. Both poses are now solved against the body axes,
and ``folded_flat`` comes out at the clean ``(0, -pi/2, +pi/2, 0, 0, 0)``.

**Any zero pinned against the previous values is off by that offset** —
roughly 14 deg on ``shoulder_lift`` and 16 deg on ``elbow_flex``. Re-pin
rather than trusting them.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

__all__ = [
    "ReferencePose",
    "REFERENCE_POSES",
    "JOINT_ORDER",
    "URDF_ZERO",
    "LEVEL",
    "FOLDED_FLAT",
]

#: URDF joint order. Matches ``soarm_tamp.conventions.JOINT_ORDER`` and the
#: order every :class:`~soarm_sdk.calibration.frame.RobotCalibration` stores.
JOINT_ORDER: Tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


@dataclass(frozen=True)
class ReferencePose:
    """A configuration the operator can put the arm into and verify by eye.

    ``q`` is in URDF radians, in :data:`JOINT_ORDER`. ``setup`` says what
    the operator must physically do; ``verify_by`` says how they can tell
    they have done it. Neither states the resulting member angles — those
    come from the URDF via
    :func:`~soarm_sdk.kinematics.urdf_fk.member_pitches`, so they cannot
    disagree with the model.

    ``covers`` names the joints this pose actually pins. A pose that does
    not constrain a joint must not be used to re-zero it: holding the arm
    level says nothing about ``wrist_roll``, and pinning it anyway would
    replace a seeded guess with a differently-wrong one.
    """

    key: str
    label: str
    q: Tuple[float, ...]
    setup: str
    verify_by: str
    covers: Tuple[str, ...]

    def as_cfg(self) -> Dict[str, float]:
        """``{joint_name: radians}``, for a URDF or an FK call."""
        return dict(zip(JOINT_ORDER, self.q))

    def q_for(self, names: Tuple[str, ...]) -> Tuple[float, ...]:
        """This pose's configuration reordered to *names*.

        A calibration stores its joints in its own order; reindexing by
        name rather than trusting position keeps a reordered calibration
        from silently pinning the wrong joint.
        """
        cfg = self.as_cfg()
        missing = [n for n in names if n not in cfg]
        if missing:
            raise KeyError(f"{self.key} has no angle for: {', '.join(missing)}")
        return tuple(cfg[n] for n in names)


#: The URDF's own zero, kept for driving the view to it and for comparison.
#:
#: It is the *same configuration* as :data:`LEVEL` — measured along the
#: member bodies, the model's zero really is upper arm vertical and forearm
#: level. ``covers`` stays empty so that re-zeroing goes through
#: :data:`LEVEL`, which carries the setup and verification prose; this entry
#: exists to name the configuration, not to pin anything to it.
URDF_ZERO = ReferencePose(
    key="urdf_zero",
    label="URDF zero (same configuration as 'level')",
    q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    setup="Nothing to set up — this is the model's zero, shown for comparison.",
    verify_by=(
        "Use 'level' instead — same configuration, with the setup and "
        "verification steps attached."
    ),
    covers=(),
)

#: Upper arm straight up, forearm and gripper axis level, no yaw.
#:
#: This is the URDF's own zero — ``q`` is all zeros, and measured along the
#: member bodies that configuration puts the upper arm at +89.90 deg and the
#: forearm at +0.00 deg. It previously carried
#: ``(0, -0.2438, +0.2823, -0.0881, 0, 0)``, solved to level the joint-origin
#: chords rather than the members; see the module docstring.
LEVEL = ReferencePose(
    key="level",
    label="Upper arm vertical, forearm level",
    q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    setup=(
        "Release torque. Upper arm vertical, forearm horizontal, base square "
        "(no yaw). Support it while reading — it droops ~2 deg once you let go."
    ),
    verify_by=(
        "Level or inclinometer: vertical on the upper arm, level on the "
        "forearm and gripper axis."
    ),
    covers=("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"),
)

#: Upper arm laid flat pointing back, forearm folded flat on top of it.
#:
#: Solved from the URDF against the member **bodies**: both the upper arm
#: and the forearm come out at 0.00 deg elevation, which also makes them
#: mutually parallel — folded face to face and level, which is what the
#: pose's name claims. ``tests/test_reference_poses.py`` asserts both.
#:
#: The configuration is exactly ``shoulder_lift = -pi/2``,
#: ``elbow_flex = +pi/2``, which is a good sign the body axes are the right
#: line to have solved against: the previous chord-derived
#: ``(-1.81458, +1.85311)`` was 14-16 deg away and rendered with the upper
#: arm visibly sloped.
#:
#: Better than :data:`LEVEL` for the two joints it covers, because the
#: operator is not judging "vertical" against gravity by eye — the links
#: resting face to face is a mechanical constraint that repeats, and only
#: the pair's shared pitch has to be levelled.
#:
#: Now inside the URDF's limits (``elbow_flex`` +90.0 deg against a +96.8
#: ceiling), so the mirror no longer renders it self-intersecting — the old
#: value overshot at +106.2 deg and the dashboard had to explain the
#: intersection away.
FOLDED_FLAT = ReferencePose(
    key="folded_flat",
    label="Upper arm flat, forearm folded flat on top",
    q=(0.0, -1.570796, 1.570796, 0.0, 0.0, 0.0),
    setup=(
        "Release torque. Lay the upper arm flat, fold the forearm back onto "
        "it, base square (no yaw)."
    ),
    verify_by=(
        "The links resting face to face is a mechanical stop, not a "
        "judgement. Then one level across the pair."
    ),
    covers=("shoulder_lift", "elbow_flex"),
)

#: Every pose the dashboard offers, keyed by :attr:`ReferencePose.key`.
REFERENCE_POSES: Dict[str, ReferencePose] = {
    p.key: p for p in (FOLDED_FLAT, LEVEL, URDF_ZERO)
}


def get(key: str) -> Optional[ReferencePose]:
    """Look up a reference pose by key, or ``None``."""
    return REFERENCE_POSES.get(key)
