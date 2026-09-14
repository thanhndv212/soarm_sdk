"""The reference poses must mean what they say, checked against the URDF.

This is the test the bug got past. ``rezero_from_pose`` documented its
all-zeros default as "the upper arm vertical and the forearm horizontal",
and nothing anywhere compared that sentence to the model. It was wrong by
13.97 deg at the shoulder and 16.17 deg at the elbow, and every zero
pinned on the strength of it inherited the error.

So: no prose claim about a pose's geometry without an assertion behind
it. These tests pose the real URDF and measure.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from soarm_sdk.calibration.reference import (
    FOLDED_FLAT,
    JOINT_ORDER,
    LEVEL,
    REFERENCE_POSES,
    URDF_ZERO,
)
from soarm_sdk.kinematics.urdf_fk import URDF_AVAILABLE, load_urdf, member_pitches

# A sibling SO-ARM100/ checkout, the same assumption the dashboard CLI makes.
URDF_PATH = (
    Path(__file__).resolve().parents[2]
    / "SO-ARM100"
    / "Simulation"
    / "SO101"
    / "so101_new_calib.urdf"
)

needs_urdf = pytest.mark.skipif(
    not URDF_AVAILABLE or not URDF_PATH.exists(),
    reason="yourdfpy and a sibling SO-ARM100/ checkout are needed",
)


@pytest.fixture(scope="module")
def urdf():
    loaded = load_urdf(URDF_PATH)
    assert loaded is not None, f"could not load {URDF_PATH}"
    return loaded


# -- the poses describe the geometry they claim ------------------------


@needs_urdf
def test_level_pose_really_is_level(urdf):
    """Upper arm vertical, forearm horizontal — the claim in LEVEL.setup."""
    pitches = member_pitches(urdf, LEVEL.as_cfg())
    assert pitches["upper_arm"] == pytest.approx(90.0, abs=0.15)
    assert pitches["forearm"] == pytest.approx(0.0, abs=0.05)


@needs_urdf
def test_urdf_zero_is_the_level_pose(urdf):
    """The correction to the correction.

    This file used to assert the opposite — that the two were 14 deg and
    16 deg apart — on the strength of ``MEMBERS`` measuring each member as
    the chord between two joint-frame origins. Those origins sit off the
    upper arm's long axis, so the chord is not the line a level lies along.
    Measured along the body, the URDF's own zero *is* upper arm vertical
    and forearm level, exactly as ``rezero_from_pose`` said before anyone
    "fixed" it.
    """
    assert LEVEL.q == URDF_ZERO.q
    zero = member_pitches(urdf, URDF_ZERO.as_cfg())
    assert zero["upper_arm"] == pytest.approx(90.0, abs=0.15)
    assert zero["forearm"] == pytest.approx(0.0, abs=0.05)


@needs_urdf
def test_the_chord_and_the_body_are_not_the_same_line(urdf):
    """Why every pose here was wrong, pinned so it cannot come back.

    Keep both numbers in one place: the chord between joint origins reads
    +76 deg at the URDF zero, the member body reads +90 deg, and the 14 deg
    between them is the whole bug. Anyone reverting ``MEMBERS`` to the
    origin-to-origin form fails here.
    """
    import numpy as np

    urdf.update_cfg({k: 0.0 for k in JOINT_ORDER})
    g = urdf.scene.graph
    a = g["upper_arm_link"][0][:3, 3]
    b = g["lower_arm_link"][0][:3, 3]
    chord = b - a
    chord_pitch = math.degrees(math.asin(chord[2] / float(np.linalg.norm(chord))))

    body_pitch = member_pitches(urdf, {k: 0.0 for k in JOINT_ORDER})["upper_arm"]

    assert chord_pitch == pytest.approx(76.03, abs=0.05)
    assert body_pitch == pytest.approx(90.0, abs=0.15)
    assert abs(body_pitch - chord_pitch) == pytest.approx(13.9, abs=0.2)


@needs_urdf
def test_the_baked_body_axes_still_match_the_meshes(urdf):
    """MEMBERS carries measured constants; re-derive them from the URDF.

    Taken from each shell mesh's oriented bounding box. PCA is not usable
    here — the wrist shell is nearly cubic and its principal axis comes out
    16 deg from the body.
    """
    import numpy as np

    from soarm_sdk.kinematics.urdf_fk import MEMBERS

    urdf.update_cfg({k: 0.0 for k in JOINT_ORDER})
    g = urdf.scene.graph
    shells = {
        "upper_arm_link": "upper_arm_so101_v1",
        "lower_arm_link": "under_arm_so101_v1",
        "wrist_link": "wrist_roll_pitch_so101_v2",
    }
    for _name, link, baked in MEMBERS:
        T_link, _ = g[link]
        node = next(n for n in g.nodes_geometry if shells[link] in n)
        T_node, gname = g[node]
        obb = urdf.scene.geometry[gname].bounding_box_oriented
        R = np.asarray(obb.primitive.transform)[:3, :3]
        axis = R[:, int(np.argmax(np.asarray(obb.primitive.extents)))]
        world = T_node[:3, :3] @ axis
        world /= np.linalg.norm(world)
        derived = T_link[:3, :3].T @ world
        baked_v = np.asarray(baked)
        # Sign is arbitrary out of an OBB; compare the line, not the ray.
        cos = abs(float(np.dot(derived, baked_v)))
        assert math.degrees(math.acos(min(cos, 1.0))) < 0.5, link


# -- structural invariants, no URDF needed -----------------------------


def test_every_pose_covers_only_real_joints():
    for pose in REFERENCE_POSES.values():
        assert set(pose.covers) <= set(JOINT_ORDER), pose.key


def test_every_pose_has_one_angle_per_joint():
    for pose in REFERENCE_POSES.values():
        assert len(pose.q) == len(JOINT_ORDER), pose.key


def test_urdf_zero_covers_nothing():
    """It is not verifiable by eye, so it must not be offered as a re-zero."""
    assert URDF_ZERO.covers == ()


def test_level_pose_does_not_claim_wrist_roll_or_gripper():
    """Standing the arm up level says nothing about roll or jaw opening."""
    assert "wrist_roll" not in LEVEL.covers
    assert "gripper" not in LEVEL.covers


def test_q_for_reorders_by_name_not_position():
    reversed_order = tuple(reversed(JOINT_ORDER))
    got = LEVEL.q_for(reversed_order)
    assert got == tuple(reversed(LEVEL.q))


def test_q_for_rejects_an_unknown_joint():
    with pytest.raises(KeyError):
        LEVEL.q_for(("shoulder_pan", "not_a_joint"))


# -- the folded-flat pose ----------------------------------------------


@needs_urdf
def test_folded_flat_really_is_flat_and_folded(urdf):
    """Both member bodies level, and parallel to each other.

    Measured along the bodies, not the joint-origin chords: the previous
    values levelled the chords and left the upper arm visibly sloped, which
    is what the operator sees and what a level on it reads.
    """
    pitches = member_pitches(urdf, FOLDED_FLAT.as_cfg())
    assert pitches["upper_arm"] == pytest.approx(0.0, abs=0.05)
    assert pitches["forearm"] == pytest.approx(0.0, abs=0.05)
    # Both level means both parallel, which is what "folded flat" claims.
    assert abs(pitches["upper_arm"] - pitches["forearm"]) < 0.05


def test_folded_flat_is_a_right_angle_at_each_joint():
    """Solved, not typed — and it landed exactly on -pi/2, +pi/2.

    A good sign the body axes are the right line to have solved against.
    The chord-derived values were -1.81458 and +1.85311.
    """
    cfg = FOLDED_FLAT.as_cfg()
    assert cfg["shoulder_lift"] == pytest.approx(-math.pi / 2, abs=1e-5)
    assert cfg["elbow_flex"] == pytest.approx(math.pi / 2, abs=1e-5)


def test_folded_flat_is_now_inside_the_urdf_limits():
    """It used to overshoot the elbow ceiling by 9.4 deg and self-intersect.

    That was a symptom of the wrong solve, not of conservative limits.
    """
    cfg = FOLDED_FLAT.as_cfg()
    assert abs(cfg["elbow_flex"]) < 1.69
    assert abs(cfg["shoulder_lift"]) < 1.74533


def test_folded_flat_covers_only_the_two_pitch_joints():
    """It says nothing about yaw, wrist pitch, roll or the jaw."""
    assert FOLDED_FLAT.covers == ("shoulder_lift", "elbow_flex")
