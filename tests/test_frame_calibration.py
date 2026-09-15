"""Unit tests for soarm_sdk.calibration.frame.

Pure math and file I/O, no hardware. The fixture data is the real
SO-101 calibration for `thanh_arm` plus the limits from
so101_new_calib.urdf, so these tests exercise the numbers the arm
actually has rather than round ones.
"""

from __future__ import annotations

import pytest

from soarm_sdk.conversions import RADS_PER_TICK
from soarm_sdk.calibration.frame import (
    JointCalibration,
    RobotCalibration,
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
    """wrist_roll is -1 by default, not +1 — see DEFAULT_DIRECTION_SIGN_OVERRIDES."""
    by_name = dict(zip(seeded.names, seeded.direction_signs))
    assert by_name == {
        "shoulder_pan": 1, "shoulder_lift": 1, "elbow_flex": 1,
        "wrist_flex": 1, "wrist_roll": -1, "gripper": 1,
    }
    assert seeded.validated is False
    assert "ASSUMED" in seeded.notes["direction_signs"]


def test_wrist_roll_defaults_to_a_flipped_sign():
    """2026-09-12 physical check on thanh_arm found it turning opposite the
    URDF's convention. A travel range still can't establish this by itself —
    it is an assumption, not a measurement, and validated stays False."""
    from soarm_sdk.calibration.frame import (
        DEFAULT_DIRECTION_SIGN_OVERRIDES,
        seed_from_travel,
    )

    assert DEFAULT_DIRECTION_SIGN_OVERRIDES == {"wrist_roll": -1}
    j = seed_from_travel(
        ["wrist_roll"], [(-2.74385, 2.84121)], [(102, 3993)]
    ).joints[0]
    assert j.direction_sign == -1


def test_every_other_joint_still_defaults_to_positive():
    from soarm_sdk.calibration.frame import seed_from_travel

    for name in ("shoulder_pan", "shoulder_lift", "elbow_flex",
                 "wrist_flex", "gripper"):
        j = seed_from_travel([name], [(-1.0, 1.0)], [(1000, 3000)]).joints[0]
        assert j.direction_sign == 1


def test_an_explicit_sign_overrides_the_wrist_roll_default():
    from soarm_sdk.calibration.frame import seed_from_travel

    j = seed_from_travel(
        ["wrist_roll"], [(-2.74385, 2.84121)], [(102, 3993)],
        direction_signs=[1],
    ).joints[0]
    assert j.direction_sign == 1


def test_seeding_rejects_degenerate_input() -> None:
    with pytest.raises(ValueError):
        seed_from_travel(["j"], [(1.0, -1.0)], [(0, 100)])
    with pytest.raises(ValueError):
        seed_from_travel(["j"], [(-1.0, 1.0)], [(100, 100)])
    with pytest.raises(ValueError):
        seed_from_travel(["a", "b"], [(-1.0, 1.0)], [(0, 100)])


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


def test_negative_sign_maps_the_other_endpoint_to_the_lower_limit():
    """A flipped joint's zero is not the +1 zero with a sign stuck on it.

    The sign decides which end of the measured travel is the URDF's lower
    limit, so it changes the zero the two endpoints agree on. Getting this
    wrong mirrors the joint about the wrong point — a plausible-looking
    calibration that is wrong everywhere except by coincidence.
    """
    from soarm_sdk.calibration.frame import seed_from_travel

    limits, ticks = [(-1.0, 1.0)], [(1000, 3000)]
    pos = seed_from_travel(["j"], limits, ticks).joints[0]
    neg = seed_from_travel(["j"], limits, ticks, direction_signs=[-1]).joints[0]

    assert pos.direction_sign == 1 and neg.direction_sign == -1
    # Symmetric limits and travel put both zeros at the travel midpoint here,
    # but the endpoints they map to are swapped.
    assert pos.to_rad(1000) == pytest.approx(-neg.to_rad(1000), abs=1e-9)


def test_negative_sign_zero_differs_when_limits_are_asymmetric():
    from soarm_sdk.calibration.frame import seed_from_travel

    # This arm's real numbers (asymmetric URDF limits) under a generic
    # joint name, so the test exercises the sign argument rather than
    # DEFAULT_DIRECTION_SIGN_OVERRIDES, which now defaults "wrist_roll" to
    # the same -1 the "neg" case asks for explicitly.
    limits, ticks = [(-2.74385, 2.84121)], [(102, 3993)]
    pos = seed_from_travel(["j"], limits, ticks, direction_signs=[1]).joints[0]
    neg = seed_from_travel(["j"], limits, ticks, direction_signs=[-1]).joints[0]

    assert pos.zero_offset_ticks != pytest.approx(neg.zero_offset_ticks)
    # Each end of the travel must still land on a URDF limit, swapped over.
    assert neg.to_rad(3993) == pytest.approx(-2.74385, abs=0.2)
    assert neg.to_rad(102) == pytest.approx(2.84121, abs=0.2)


def test_direction_signs_are_validated():
    from soarm_sdk.calibration.frame import seed_from_travel

    with pytest.raises(ValueError, match="one entry per joint"):
        seed_from_travel(["a"], [(-1.0, 1.0)], [(0, 100)], direction_signs=[1, 1])
    with pytest.raises(ValueError, match=r"\+1 or -1"):
        seed_from_travel(["a"], [(-1.0, 1.0)], [(0, 100)], direction_signs=[0])


def test_rebasing_preserves_the_physical_position_the_zero_meant():
    """+100 to STS_OFS moves every REPORTED position by -100 (recentre.py's
    own finding); the calibration's zero has to move the same way to keep
    pointing at the same physical spot."""
    from soarm_sdk.calibration.frame import JointCalibration

    j = JointCalibration("wrist_roll", 2074.0, 1, 102, 3993,
                          zero_source="manual_sign_flip")
    out = j.rebased_after_offset_change(delta_ticks=-1832.0)
    assert out.zero_offset_ticks == 2074.0 + 1832.0
    assert out.zero_source == "rebased_after_recentre"


def test_rebasing_moves_the_hard_stops_by_the_same_amount():
    from soarm_sdk.calibration.frame import JointCalibration

    j = JointCalibration("a", 2000.0, 1, 500, 3500)
    out = j.rebased_after_offset_change(delta_ticks=200.0)
    assert (out.tick_min, out.tick_max) == (300, 3300)
    assert out.zero_offset_ticks == 1800.0


def test_rebasing_by_zero_is_the_identity_except_provenance():
    from soarm_sdk.calibration.frame import JointCalibration

    j = JointCalibration("a", 2000.0, 1, 500, 3500, zero_source="reference_pose")
    out = j.rebased_after_offset_change(0.0)
    assert (out.zero_offset_ticks, out.tick_min, out.tick_max) == (2000.0, 500, 3500)
    assert out.zero_source == "rebased_after_recentre"


def test_rebasing_does_not_touch_the_direction_sign():
    from soarm_sdk.calibration.frame import JointCalibration

    j = JointCalibration("a", 2000.0, -1, 500, 3500)
    assert j.rebased_after_offset_change(50.0).direction_sign == -1


def test_pinning_to_the_current_tick_makes_it_read_as_zero():
    from soarm_sdk.calibration.frame import JointCalibration

    j = JointCalibration("wrist_roll", 6153.9, 1, 1, 4095, zero_source="manual_nudge")
    out = j.with_zero_pinned_to_current_tick(1418.0)
    assert out.to_rad(1418.0) == 0.0
    assert out.zero_source == "pinned_to_current_tick"


def test_pinning_to_current_tick_never_claims_a_pose():
    from soarm_sdk.calibration.frame import JointCalibration

    j = JointCalibration("a", 2000.0, 1, 0, 4095, zero_source="reference_pose")
    assert j.with_zero_pinned_to_current_tick(500.0).zero_source != "reference_pose"


def test_pinning_to_current_tick_does_not_touch_sign_or_travel():
    from soarm_sdk.calibration.frame import JointCalibration

    j = JointCalibration("a", 2000.0, -1, 100, 3000)
    out = j.with_zero_pinned_to_current_tick(1500.0)
    assert out.direction_sign == -1
    assert (out.tick_min, out.tick_max) == (100, 3000)


# -- EEPROM limit tracking ------------------------------------------------
#
# The gap this closes: wrist_flex's servo capped at 3046 ticks while the
# calibration's travel said 3314, and nothing compared the two. These
# fields let that comparison happen; see hardware.py's connect-time check
# and conventions.phantom_range().


def test_a_calibration_written_before_this_field_existed_loads_fine():
    """Every calibration on disk before this session predates these fields.

    ``RobotCalibration.load`` round-trips through ``JointCalibration(**j)``;
    a JSON object missing the two ``eeprom_*`` keys must fall through to
    their dataclass defaults rather than raising.
    """
    from soarm_sdk.calibration.frame import JointCalibration

    j = JointCalibration(**{
        "name": "wrist_flex", "zero_offset_ticks": 2487.0, "direction_sign": 1,
        "tick_min": 1285, "tick_max": 3314,
    })
    assert j.eeprom_min_ticks is None
    assert j.eeprom_max_ticks is None
    assert j.eeprom_limits_ticks() is None
    assert j.eeprom_recorded_at == ""


def test_with_eeprom_limits_records_ticks_and_a_timestamp():
    j = JointCalibration("wrist_flex", 2487.0, 1, 1285, 3314)
    out = j.with_eeprom_limits(1050, 3546)
    assert out.eeprom_limits_ticks() == (1050, 3546)
    assert out.eeprom_recorded_at != ""
    # travel and zero are untouched — this is a fact about the servo's
    # firmware, independent of what the mechanism measures or means.
    assert (out.tick_min, out.tick_max) == (1285, 3314)
    assert out.zero_offset_ticks == 2487.0


def test_with_eeprom_limits_orders_min_below_max_regardless_of_argument_order():
    j = JointCalibration("a", 2048.0, 1, 0, 4095)
    out = j.with_eeprom_limits(3546, 1050)  # swapped
    assert out.eeprom_limits_ticks() == (1050, 3546)


def test_eeprom_mismatch_is_none_when_never_recorded():
    """A joint with no recorded EEPROM says nothing either way — the same
    gate ``measured_is_trusted`` uses for travel acceptance."""
    j = JointCalibration("a", 2048.0, 1, 0, 4095)
    assert j.eeprom_mismatch(1050, 3546) is None


def test_eeprom_mismatch_is_none_when_the_live_reading_agrees():
    j = JointCalibration("a", 2048.0, 1, 0, 4095).with_eeprom_limits(1050, 3546)
    assert j.eeprom_mismatch(1050, 3546) is None
    # within the rounding tolerance
    assert j.eeprom_mismatch(1051, 3545) is None


def test_eeprom_mismatch_names_the_joint_and_both_readings():
    """The exact bug this exists to catch: wrist_flex recorded 3314 but the
    servo now reports 3046."""
    j = JointCalibration("wrist_flex", 2487.0, 1, 1285, 3314).with_eeprom_limits(1050, 3314)
    msg = j.eeprom_mismatch(1050, 3046)
    assert msg is not None
    assert "wrist_flex" in msg
    assert "3314" in msg and "3046" in msg


def test_eeprom_limits_survive_save_load_round_trip(tmp_path):
    from soarm_sdk.calibration.frame import RobotCalibration

    j = JointCalibration("wrist_flex", 2487.0, 1, 1285, 3546).with_eeprom_limits(1050, 3546)
    cal = RobotCalibration(joints=[j], arm_id="thanh_arm")
    path = cal.save(tmp_path / "cal.json")
    back = RobotCalibration.load(path)
    assert back.joints[0].eeprom_limits_ticks() == (1050, 3546)
    assert back.joints[0].eeprom_recorded_at == j.eeprom_recorded_at
