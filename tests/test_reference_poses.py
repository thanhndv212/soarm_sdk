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


# -- the folded-flat pose ----------------------------------------------


@needs_urdf
def test_folded_flat_really_is_flat_and_folded(urdf):
    """Both links level, and the forearm lying back along the upper arm."""
    import numpy as np

    cfg = FOLDED_FLAT.as_cfg()
    urdf.update_cfg({k: float(v) for k, v in cfg.items()})
    pts = {
        lk: urdf.scene.graph[lk][0][:3, 3]
        for lk in ("upper_arm_link", "lower_arm_link", "wrist_link")
    }
    upper = pts["lower_arm_link"] - pts["upper_arm_link"]
    fore = pts["wrist_link"] - pts["lower_arm_link"]

    # Flat: both headings on the horizontal, 180 deg apart.
    h_up = math.degrees(math.atan2(upper[2], upper[0]))
    h_fo = math.degrees(math.atan2(fore[2], fore[0]))
    assert abs(h_up) == pytest.approx(180.0, abs=0.05)
    assert h_fo == pytest.approx(0.0, abs=0.05)

    # Folded: antiparallel, and the axes at the same height.
    cos = float(
        np.dot(upper, fore) / (np.linalg.norm(upper) * np.linalg.norm(fore))
    )
    assert math.degrees(math.acos(cos)) == pytest.approx(180.0, abs=0.1)
    assert abs(pts["wrist_link"][2] - pts["upper_arm_link"][2]) < 1e-3


@needs_urdf
def test_folded_flat_is_outside_the_urdf_elbow_limit(urdf):
    """Stated in the docstring, so assert it rather than trusting the prose.

    The pose is reachable and valid; the URDF's limits are conservative.
    The mirror will render it out of range, which is worth knowing before
    it looks like the re-zero broke something.
    """
    elbow = FOLDED_FLAT.as_cfg()["elbow_flex"]
    assert elbow > 1.69, "URDF ceiling is 1.69 rad"
    assert math.degrees(elbow - 1.69) == pytest.approx(9.4, abs=0.2)


def test_folded_flat_covers_only_the_two_pitch_joints():
    """It says nothing about yaw, wrist pitch, roll or the jaw."""
    assert FOLDED_FLAT.covers == ("shoulder_lift", "elbow_flex")
