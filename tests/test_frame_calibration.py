"""Unit tests for soarm_sdk.frame_calibration.

Pure math and file I/O, no hardware. The fixture data is the real
SO-101 calibration for `thanh_arm` plus the limits from
so101_new_calib.urdf, so these tests exercise the numbers the arm
actually has rather than round ones.
"""

from __future__ import annotations

import json

import pytest

from soarm_sdk.conversions import RADS_PER_TICK
from soarm_sdk.frame_calibration import (
    RobotCalibration,
    seed_from_lerobot,
    seed_from_travel,
)

# so101_new_calib.urdf joint limits, in URDF order.
URDF_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}

# Measured travel from the real lerobot calibration for this arm.
TICK_RANGES = {
    "shoulder_pan": (763, 3188),
    "shoulder_lift": (890, 3271),
    "elbow_flex": (824, 3050),
    "wrist_flex": (776, 2911),
    "wrist_roll": (116, 3929),
    "gripper": (2046, 3512),
}


@pytest.fixture
def seeded() -> RobotCalibration:
    names = list(URDF_LIMITS)
    return seed_from_travel(
        names=names,
        urdf_limits=[URDF_LIMITS[n] for n in names],
        tick_ranges=[TICK_RANGES[n] for n in names],
        arm_id="thanh_arm",
    )


# ---------------------------------------------------------------- round trip


def test_tick_rad_round_trip_is_identity(seeded: RobotCalibration) -> None:
    for j in seeded.joints:
        for ticks in range(j.tick_min, j.tick_max + 1, 37):
            assert j.to_ticks(j.to_rad(ticks)) == pytest.approx(ticks, abs=1e-6)


def test_vector_round_trip(seeded: RobotCalibration) -> None:
    rad = [0.1, -0.2, 0.3, -0.4, 0.5, 0.6]
    assert seeded.ticks_to_rad(seeded.rad_to_ticks(rad)) == pytest.approx(rad)


def test_wrong_length_is_rejected(seeded: RobotCalibration) -> None:
    with pytest.raises(ValueError):
        seeded.ticks_to_rad([0, 0, 0])
    with pytest.raises(ValueError):
        seeded.rad_to_ticks([0.0])


# ------------------------------------------------------------------ seeding


def test_every_urdf_limit_lands_inside_measured_travel(
    seeded: RobotCalibration,
) -> None:
    """The seed must not put a reachable URDF angle outside the hard stops.

    Tolerance is the joint's own seed residual plus a tick, since that
    residual *is* the disagreement between the two sources.
    """
    for j in seeded.joints:
        lo, hi = URDF_LIMITS[j.name]
        tol = j.seed_residual_rad * 1.0 / RADS_PER_TICK + 1.0
        assert j.to_ticks(lo) >= j.tick_min - tol
        assert j.to_ticks(hi) <= j.tick_max + tol


def test_zero_is_range_midpoint_for_symmetric_limits(
    seeded: RobotCalibration,
) -> None:
    """A symmetric URDF range puts the seeded zero at the travel midpoint.

    Worth pinning down: it means the seeded frame and lerobot's own
    normalization agree exactly on those joints, so a migration from one to
    the other moves nothing. Only the asymmetric joints (wrist_roll,
    gripper) shift.
    """
    for j in seeded.joints:
        lo, hi = URDF_LIMITS[j.name]
        if abs(lo + hi) < 1e-6:
            midpoint = (j.tick_min + j.tick_max) / 2
            assert j.zero_offset_ticks == pytest.approx(midpoint, abs=1e-6)


def test_asymmetric_joints_do_not_sit_at_the_midpoint(
    seeded: RobotCalibration,
) -> None:
    by = {j.name: j for j in seeded.joints}
    for name in ("wrist_roll", "gripper"):
        j = by[name]
        midpoint = (j.tick_min + j.tick_max) / 2
        assert abs(j.zero_offset_ticks - midpoint) > 1.0


def test_seed_residual_is_small_for_the_arm_joints(
    seeded: RobotCalibration,
) -> None:
    """The four joints that carry the end effector must seed tightly.

    These drive Cartesian accuracy, so a loose seed here would show up as a
    grasp that misses. wrist_roll and the gripper are allowed to be looser
    and are checked separately.
    """
    by = {j.name: j for j in seeded.joints}
    for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"):
        assert by[name].seed_residual_rad < 0.10


def test_gripper_span_is_flagged_as_suspect(seeded: RobotCalibration) -> None:
    """The gripper's measured travel exceeds the URDF's by ~17%.

    The gearing is fixed, so this is a disagreement between sources, not a
    conversion error — and it must surface rather than be averaged away.
    """
    by = {j.name: j for j in seeded.joints}
    assert by["gripper"].suspect
    assert "gripper" in seeded.suspect_joints
    for name in ("elbow_flex", "wrist_flex"):
        assert not by[name].suspect


def test_direction_signs_are_assumed_and_flagged(seeded: RobotCalibration) -> None:
    assert seeded.direction_signs == [1] * 6
    assert seeded.validated is False
    assert "ASSUMED" in seeded.notes["direction_signs"]


def test_seeding_rejects_degenerate_input() -> None:
    with pytest.raises(ValueError):
        seed_from_travel(["j"], [(1.0, -1.0)], [(0, 100)])
    with pytest.raises(ValueError):
        seed_from_travel(["j"], [(-1.0, 1.0)], [(100, 100)])
    with pytest.raises(ValueError):
        seed_from_travel(["a", "b"], [(-1.0, 1.0)], [(0, 100)])


# --------------------------------------------------------------- lerobot io


def test_seed_from_lerobot_ignores_homing_offset(tmp_path) -> None:
    """homing_offset lives in servo EEPROM; applying it again would double it.

    Two files identical but for homing_offset must seed identically.
    """
    def write(name: str, offset: int):
        p = tmp_path / f"{name}.json"
        p.write_text(
            json.dumps(
                {
                    n: {
                        "id": i + 1,
                        "drive_mode": 0,
                        "homing_offset": offset,
                        "range_min": TICK_RANGES[n][0],
                        "range_max": TICK_RANGES[n][1],
                    }
                    for i, n in enumerate(URDF_LIMITS)
                }
            )
        )
        return p

    a = seed_from_lerobot(write("a", 0), URDF_LIMITS)
    b = seed_from_lerobot(write("b", -1528), URDF_LIMITS)
    assert a.zero_offsets == pytest.approx(b.zero_offsets)


def test_seed_from_lerobot_reports_a_missing_joint(tmp_path) -> None:
    p = tmp_path / "partial.json"
    p.write_text(json.dumps({"shoulder_pan": {"range_min": 0, "range_max": 100}}))
    with pytest.raises(KeyError, match="shoulder_lift"):
        seed_from_lerobot(p, URDF_LIMITS)


def test_joint_order_follows_urdf_limits_not_the_file(tmp_path) -> None:
    p = tmp_path / "shuffled.json"
    shuffled = {
        n: {"range_min": TICK_RANGES[n][0], "range_max": TICK_RANGES[n][1]}
        for n in reversed(list(URDF_LIMITS))
    }
    p.write_text(json.dumps(shuffled))
    assert seed_from_lerobot(p, URDF_LIMITS).names == list(URDF_LIMITS)


# -------------------------------------------------------------- persistence


def test_save_load_round_trip(seeded: RobotCalibration, tmp_path) -> None:
    path = seeded.save(tmp_path / "nested" / "cal.json")
    back = RobotCalibration.load(path)
    assert back.names == seeded.names
    assert back.zero_offsets == pytest.approx(seeded.zero_offsets)
    assert back.direction_signs == seeded.direction_signs
    assert back.validated is False
    assert back.arm_id == "thanh_arm"


def test_mark_validated_records_how(seeded: RobotCalibration) -> None:
    assert seeded.validated is False
    seeded.mark_validated("tape-measure FK check, 4 poses, max error 3 mm")
    assert seeded.validated is True
    assert "tape-measure" in seeded.notes["validated_by"]
    assert "validated_at" in seeded.notes
