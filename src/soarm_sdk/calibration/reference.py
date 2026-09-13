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

The bug that motivated it
-------------------------
``rezero_from_pose`` documented its all-zeros default as "for the SO-101
that is the upper arm vertical and the forearm horizontal". It is not. At
the URDF's zero the upper arm sits at **+76.03 deg** and the forearm at
**+2.21 deg**; the pose actually described is :data:`LEVEL`, whose
configuration is ``(0, -0.2438, +0.2823, -0.0881, 0, 0)``. Re-zeroing
against the default while holding the arm in the documented pose would
have pushed ``shoulder_lift`` 13.97 deg and ``elbow_flex`` 16.17 deg off
true — in opposite directions, which is exactly the "both links bend the
wrong way" signature.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

__all__ = ["ReferencePose", "REFERENCE_POSES", "JOINT_ORDER", "URDF_ZERO", "LEVEL"]

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


#: The URDF's own zero. Not a pose anyone can hold the arm in by eye — it
#: is included so the view can be driven to it and so the docstring bug
#: above stays visible as a comparison, not because it makes a good
#: reference. ``covers`` is empty: nothing here is independently checkable.
URDF_ZERO = ReferencePose(
    key="urdf_zero",
    label="URDF zero (not a checkable pose)",
    q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    setup="Nothing to set up — this is the model's zero, shown for comparison.",
    verify_by=(
        "Nothing. The upper arm sits at +76 deg and the forearm at +2 deg here, "
        "so there is no straight edge or level that confirms it. Do not re-zero "
        "against this pose."
    ),
    covers=(),
)

#: The pose the arm's zeros are actually pinned to: upper arm straight up,
#: forearm and gripper axis level, no yaw. Verified against the URDF in
#: ``tests/test_reference_poses.py`` — upper arm 90.00 deg, forearm 0.00 deg.
LEVEL = ReferencePose(
    key="level",
    label="Upper arm vertical, forearm level",
    q=(0.0, -0.2438, 0.2823, -0.0881, 0.0, 0.0),
    setup=(
        "Release torque. Stand the upper arm straight up, bring the forearm "
        "horizontal, and square the base so the arm points straight out with "
        "no yaw. Support the arm while reading — it droops about 2 deg under "
        "its own weight once you let go."
    ),
    verify_by=(
        "A spirit level or phone inclinometer: vertical on the upper arm, "
        "level on the forearm and along the gripper axis."
    ),
    covers=("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"),
)

#: Every pose the dashboard offers, keyed by :attr:`ReferencePose.key`.
REFERENCE_POSES: Dict[str, ReferencePose] = {p.key: p for p in (LEVEL, URDF_ZERO)}


def get(key: str) -> Optional[ReferencePose]:
    """Look up a reference pose by key, or ``None``."""
    return REFERENCE_POSES.get(key)
