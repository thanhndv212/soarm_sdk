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
    assert pitches["upper_arm"] == pytest.approx(90.0, abs=0.05)
    assert pitches["forearm"] == pytest.approx(0.0, abs=0.05)


@needs_urdf
def test_urdf_zero_is_not_the_level_pose(urdf):
    """The bug, pinned so it cannot come back as a 'simplification'.

    Someone reading ``rezero_from_pose``'s old docstring would conclude
    these two are the same pose and drop LEVEL's awkward constants. They
    are 14 deg and 16 deg apart.
    """
    zero = member_pitches(urdf, URDF_ZERO.as_cfg())
    assert zero["upper_arm"] == pytest.approx(76.03, abs=0.05)
    assert zero["forearm"] == pytest.approx(2.21, abs=0.05)

    level = member_pitches(urdf, LEVEL.as_cfg())
    assert abs(level["upper_arm"] - zero["upper_arm"]) > 10.0
    assert abs(level["forearm"] - zero["forearm"]) > 1.0


@needs_urdf
def test_level_pose_error_if_taken_as_urdf_zero(urdf):
    """The size of the mistake, in the units the calibration is written in.

    Re-zeroing against all-zeros while the arm is physically in LEVEL puts
    shoulder_lift and elbow_flex out by these amounts, in *opposite*
    directions — which is why the symptom is the two links folding into
    each other rather than the whole arm being rotated.
    """
    cfg = LEVEL.as_cfg()
    assert math.degrees(cfg["shoulder_lift"]) == pytest.approx(-13.97, abs=0.05)
    assert math.degrees(cfg["elbow_flex"]) == pytest.approx(16.17, abs=0.05)
    assert cfg["shoulder_lift"] * cfg["elbow_flex"] < 0


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
