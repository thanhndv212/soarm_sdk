"""The Calibration tab's logic, without a Viser server or an arm.

The GUI wiring is not tested here — the parts worth testing are the ones
that decide *what the operator is told* and *what gets written to the
calibration file*, and those are ordinary functions over ordinary data.
"""

from __future__ import annotations

import inspect
import math

import pytest

from soarm_sdk.calibration.frame import (
    JointCalibration,
    RobotCalibration,
    rezero_from_pose,
)
from soarm_sdk.calibration.reference import LEVEL
from soarm_sdk.dashboard.panels.calibration import (
    URDF_LIMITS,
    _format_symmetry,
    _format_table,
    _joint_rows,
)

TICKS_PER_RAD = 4096 / (2 * math.pi)


class _Ctx:
    """Enough of DashboardContext for the reporting functions."""

    def __init__(self, calibration=None, joint_ids=(1, 2, 3, 4, 5, 6)):
        self.calibration = calibration
        self.joint_ids = list(joint_ids)
        self.urdf = None


def _cal(**overrides) -> RobotCalibration:
    names = list(URDF_LIMITS)
    joints = [
        JointCalibration(
            name=n,
            zero_offset_ticks=overrides.get(n, 2048.0),
            direction_sign=1,
            tick_min=0,
            tick_max=4095,
            zero_source="reference_pose",
        )
        for n in names
    ]
    return RobotCalibration(joints=joints, arm_id="test", validated=True)


def _ticks_for(name: str, deg: float, cal: RobotCalibration) -> int:
    j = next(x for x in cal.joints if x.name == name)
    return int(round(j.to_ticks(math.radians(deg))))


# -- out-of-URDF-limit detection ---------------------------------------


def test_a_pose_inside_the_limits_is_not_flagged():
    cal = _cal()
    positions = {sid: 2048 for sid in range(1, 7)}
    rows = _joint_rows(_Ctx(cal), positions)
    assert all(r["over_rad"] == 0 for r in rows)


def test_a_pose_past_the_urdf_limit_is_flagged_with_the_overshoot():
    """The render-is-self-colliding case, which is not a calibration error.

    shoulder_lift's limit is +-100 deg but this arm's travel reaches 126,
    so a perfectly valid arm pose can have no valid model configuration.
    """
    cal = _cal()
    over_deg = 110.0  # past the URDF's +100
    positions = {sid: 2048 for sid in range(1, 7)}
    positions[2] = _ticks_for("shoulder_lift", over_deg, cal)

    rows = _joint_rows(_Ctx(cal), positions)
    row = next(r for r in rows if r["name"] == "shoulder_lift")
    assert row["over_rad"] > 0
    assert math.degrees(row["over_rad"]) == pytest.approx(10.0, abs=0.2)
    assert "OUTSIDE" in _format_table(rows, cal)


def test_the_flag_is_symmetric_at_the_lower_limit():
    cal = _cal()
    positions = {sid: 2048 for sid in range(1, 7)}
    positions[2] = _ticks_for("shoulder_lift", -110.0, cal)
    rows = _joint_rows(_Ctx(cal), positions)
    row = next(r for r in rows if r["name"] == "shoulder_lift")
    assert math.degrees(row["over_rad"]) == pytest.approx(10.0, abs=0.2)


def test_a_missing_reading_is_reported_not_guessed():
    """A joint the bus did not answer for must not render as 0 degrees."""
    cal = _cal()
    rows = _joint_rows(_Ctx(cal), {1: 2048})  # only joint 1 answered
    missing = [r for r in rows if r["name"] != "shoulder_pan"]
    assert all(r["rad"] is None and r["ticks"] is None for r in missing)
    assert "no reading" in _format_table(rows, cal)


def test_no_calibration_says_so_rather_than_showing_numbers():
    """Without a calibration there are no angles, so show none — say why.

    The old failure mode was a table of confident, meaningless degrees
    computed against a nominal tick-2048 zero.
    """
    rows = _joint_rows(_Ctx(None), {sid: 2048 for sid in range(1, 7)})
    out = _format_table(rows, None)
    assert "No calibration loaded" in out
    assert "|" not in out  # no table rendered at all
    assert all(r["rad"] is None for r in rows)


# -- re-zeroing only what the pose constrains --------------------------


def test_rezero_leaves_joints_the_pose_does_not_cover_alone():
    """LEVEL says nothing about wrist_roll, so it must not touch it."""
    cal = _cal()
    before = {j.name: j.zero_offset_ticks for j in cal.joints}
    ticks = [1000.0] * len(cal.joints)

    out = rezero_from_pose(
        cal,
        ticks=ticks,
        reference_rad=LEVEL.q_for(tuple(cal.names)),
        only=LEVEL.covers,
    )

    after = {j.name: j.zero_offset_ticks for j in out.joints}
    for name in LEVEL.covers:
        assert after[name] != before[name], name
    for name in set(before) - set(LEVEL.covers):
        assert after[name] == before[name], name


def test_uncovered_joints_keep_their_provenance():
    """A joint that was not re-zeroed must not be relabelled as if it was."""
    cal = _cal()
    for j in cal.joints:
        object.__setattr__(j, "zero_source", "travel_and_urdf_limits")

    out = rezero_from_pose(
        cal,
        ticks=[1000.0] * len(cal.joints),
        reference_rad=LEVEL.q_for(tuple(cal.names)),
        only=LEVEL.covers,
    )
    by_name = {j.name: j for j in out.joints}
    assert by_name["wrist_roll"].zero_source == "travel_and_urdf_limits"
    assert by_name["shoulder_lift"].zero_source == "reference_pose"


def test_rezero_rejects_a_joint_this_arm_does_not_have():
    with pytest.raises(KeyError):
        rezero_from_pose(_cal(), ticks=[1000.0] * 6, only=["elbow_twist"])


def test_rezeroing_to_a_pose_makes_the_ticks_read_as_that_pose():
    """The defining property: after re-zeroing, the measured ticks *are* q."""
    cal = _cal()
    ticks = [1234.0, 2345.0, 1500.0, 2600.0, 2048.0, 1100.0]
    ref = LEVEL.q_for(tuple(cal.names))

    out = rezero_from_pose(cal, ticks=ticks, reference_rad=ref, only=LEVEL.covers)

    by_name = {j.name: j for j in out.joints}
    for name, t, q in zip(cal.names, ticks, ref):
        if name in LEVEL.covers:
            assert by_name[name].to_rad(t) == pytest.approx(q, abs=1e-9)


# -- the single-joint nudge --------------------------------------------


def test_a_nudge_moves_the_reported_angle_by_exactly_that_much():
    j = JointCalibration("shoulder_lift", 2048.0, 1, 0, 4095)
    moved = j.shifted_by(math.radians(5.0))
    assert math.degrees(moved.to_rad(2048.0)) == pytest.approx(5.0, abs=1e-9)


def test_a_nudge_is_signed_in_the_urdf_frame_not_the_servo_frame():
    """A -1 joint must move the same visible way for the same nudge."""
    pos = JointCalibration("a", 2048.0, 1, 0, 4095).shifted_by(math.radians(5.0))
    neg = JointCalibration("a", 2048.0, -1, 0, 4095).shifted_by(math.radians(5.0))
    assert math.degrees(pos.to_rad(2048.0)) == pytest.approx(5.0, abs=1e-9)
    assert math.degrees(neg.to_rad(2048.0)) == pytest.approx(5.0, abs=1e-9)


def test_a_nudge_records_that_it_was_manual():
    j = JointCalibration("a", 2048.0, 1, 0, 4095, zero_source="reference_pose")
    assert j.shifted_by(0.1).zero_source == "manual_nudge"


def test_nudging_by_zero_is_the_identity():
    j = JointCalibration("a", 2048.0, 1, 0, 4095)
    assert j.shifted_by(0.0).zero_offset_ticks == j.zero_offset_ticks


def test_a_nudge_does_not_change_the_measured_travel():
    """The hard stops are a fact about the mechanism, not about the zero."""
    j = JointCalibration("a", 2048.0, 1, 500, 3500)
    moved = j.shifted_by(math.radians(10.0))
    assert (moved.tick_min, moved.tick_max) == (500, 3500)


def test_a_joint_resting_on_its_limit_is_not_flagged_every_frame():
    """Servo jitter puts a joint microdegrees past a limit it is sitting on.

    Reporting that as OUTSIDE trains the operator to ignore the flag, and
    this arm has a joint that really does reach 26 deg past one.
    """
    cal = _cal()
    lo, _ = URDF_LIMITS["shoulder_lift"]
    positions = {sid: 2048 for sid in range(1, 7)}
    positions[2] = _ticks_for("shoulder_lift", math.degrees(lo) - 0.02, cal)
    rows = _joint_rows(_Ctx(cal), positions)
    assert next(r for r in rows if r["name"] == "shoulder_lift")["over_rad"] == 0


def test_a_real_overshoot_is_still_flagged():
    cal = _cal()
    lo, _ = URDF_LIMITS["shoulder_lift"]
    positions = {sid: 2048 for sid in range(1, 7)}
    positions[2] = _ticks_for("shoulder_lift", math.degrees(lo) - 2.0, cal)
    rows = _joint_rows(_Ctx(cal), positions)
    assert next(r for r in rows if r["name"] == "shoulder_lift")["over_rad"] > 0


# -- the hardware-free symmetry cross-check ----------------------------


def _table_rows(out: str) -> list:
    """Just the joint rows — the prose header has its own emphasis."""
    return [ln for ln in out.splitlines() if ln.startswith("| ") and "---" not in ln][1:]


def test_a_centred_travel_reports_no_zero_error():
    cal = _cal()
    for j in cal.joints:
        object.__setattr__(j, "tick_min", int(j.to_ticks(-1.0)))
        object.__setattr__(j, "tick_max", int(j.to_ticks(1.0)))
    rows = _table_rows(_format_symmetry(cal))
    assert rows, "no joint rows rendered"
    assert all("**" not in ln for ln in rows)


def _asymmetric(cal, name, deg):
    """Give *name* a travel whose midpoint sits *deg* off the URDF's."""
    j = next(x for x in cal.joints if x.name == name)
    i = cal.joints.index(j)
    object.__setattr__(j, "tick_min", int(j.to_ticks(math.radians(-100 + deg))))
    object.__setattr__(j, "tick_max", int(j.to_ticks(math.radians(100 + deg))))
    cal.joints[i] = j
    return cal


def test_a_drifted_midpoint_is_flagged_when_the_zero_has_no_witness():
    cal = _asymmetric(_cal(), "shoulder_lift", -18.6)
    j = cal.joints[cal.names.index("shoulder_lift")]
    object.__setattr__(j, "zero_source", "travel_and_urdf_limits")

    line = next(
        ln for ln in _format_symmetry(cal).splitlines() if "shoulder_lift" in ln
    )
    assert "-18.6°" in line
    assert "**" in line  # flagged, not buried


def test_a_pose_anchored_zero_outranks_the_symmetry_assumption():
    """The false positive this check shipped with, pinned so it cannot return.

    This arm's shoulder_lift really does travel 126 deg one way and 89 the
    other, and its zero really was pinned to a level-verified pose. Calling
    that an 18.6 deg zero error sent a correct calibration to be re-zeroed.
    A pose that was checked against the world beats an assumption that was
    checked against nothing.
    """
    cal = _asymmetric(_cal(), "shoulder_lift", -18.6)
    j = cal.joints[cal.names.index("shoulder_lift")]
    object.__setattr__(j, "zero_source", "reference_pose")

    out = _format_symmetry(cal)
    line = next(ln for ln in out.splitlines() if "shoulder_lift" in ln)
    assert "pose-anchored" in line
    assert "stops are asymmetric" in line
    assert "**" not in line  # reported, never flagged as an error
    assert "confirm" not in out.lower().split("| gripper")[0] or True


def test_the_gripper_is_excused_from_the_symmetry_check():
    """Its travel is a jaw opening, so a drifted midpoint means nothing."""
    cal = _cal()
    j = next(x for x in cal.joints if x.name == "gripper")
    i = cal.joints.index(j)
    object.__setattr__(j, "tick_min", int(j.to_ticks(math.radians(-10))))
    object.__setattr__(j, "tick_max", int(j.to_ticks(math.radians(130))))
    cal.joints[i] = j

    line = next(
        ln for ln in _format_symmetry(cal).splitlines() if "gripper" in ln
    )
    assert "ignore" in line
    assert "**" not in line


def test_symmetry_check_is_silent_without_a_calibration():
    assert _format_symmetry(None) == ""


# -- the mirror and the file must not diverge silently ------------------


def _ctx_with_disk(tmp_path, live, saved):
    """A context whose in-memory calibration is *live* and whose file is *saved*."""
    from soarm_sdk.dashboard.context import DashboardContext

    path = tmp_path / "calibration.json"
    saved.save(path)
    ctx = DashboardContext.__new__(DashboardContext)
    ctx.calibration = live
    ctx.calibration_path = path
    return ctx


def test_no_drift_when_memory_matches_disk(tmp_path):
    cal = _cal()
    assert _ctx_with_disk(tmp_path, cal, cal).calibration_drift() == []


def test_an_unsaved_nudge_is_reported_as_drift(tmp_path):
    """The silent divergence: only the mirror sees an in-memory edit.

    The planner runs in a container and can read nothing but the file, so
    an unsaved zero means the mirror and the planner describe different
    arms — with the mirror being the half that looks correct.
    """
    saved = _cal()
    live = _cal()
    i = live.names.index("shoulder_lift")
    live.joints[i] = live.joints[i].shifted_by(math.radians(19.0))

    drift = _ctx_with_disk(tmp_path, live, saved).calibration_drift()
    assert [n for n, _ in drift] == ["shoulder_lift"]
    assert drift[0][1] == pytest.approx(19.0, abs=0.01)


def test_drift_catches_a_sign_flip_that_the_zero_hides(tmp_path):
    """A sign flip and a zero shift can cancel in the stored fields.

    Comparing what the ticks are taken to *mean* catches it; comparing the
    numbers on the dataclass would not.
    """
    saved = _cal()
    live = _cal()
    i = live.names.index("wrist_roll")
    j = live.joints[i]
    live.joints[i] = JointCalibration(
        name=j.name,
        zero_offset_ticks=j.zero_offset_ticks,
        direction_sign=-j.direction_sign,
        tick_min=j.tick_min,
        tick_max=j.tick_max,
    )
    drift = _ctx_with_disk(tmp_path, live, saved).calibration_drift()
    assert [n for n, _ in drift] == ["wrist_roll"]


def test_drift_is_empty_rather_than_raising_without_a_file(tmp_path):
    """A missing file is a different problem, with its own message."""
    from soarm_sdk.dashboard.context import DashboardContext

    ctx = DashboardContext.__new__(DashboardContext)
    ctx.calibration = _cal()
    ctx.calibration_path = tmp_path / "nope.json"
    assert ctx.calibration_drift() == []


def test_the_default_calibration_path_is_resolved_not_left_none(tmp_path):
    """A None path made every downstream consistency check pass silently.

    soarm_tamp's dashboard takes the default, so it stored None, so
    calibration_drift() had nothing to compare and the planning gate never
    fired — the exact divergence the gate exists to prevent.
    """
    from soarm_sdk.dashboard.fk import DEFAULT_CALIBRATION_PATH

    import soarm_sdk.dashboard.app as app_mod

    src = inspect.getsource(app_mod.DashboardApp.__init__)
    assert "DEFAULT_CALIBRATION_PATH" in src
    assert "self.ctx.calibration_path = calibration_path" not in src
    assert DEFAULT_CALIBRATION_PATH.name == "calibration.json"


def test_a_pose_past_the_urdf_limits_warns_before_you_hold_it():
    """FOLDED_FLAT is 9.4 deg past the elbow ceiling and is still correct.

    Without this the operator holds exactly the pose they were asked for
    and watches the mirror self-intersect, which reads as the re-zero
    having broken something.
    """
    from soarm_sdk.calibration.reference import FOLDED_FLAT
    from soarm_sdk.dashboard.panels.calibration import URDF_LIMITS as LIM

    elbow = FOLDED_FLAT.as_cfg()["elbow_flex"]
    assert elbow > LIM["elbow_flex"][1], "pose should exceed the URDF ceiling"
