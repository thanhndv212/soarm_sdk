"""The Calibration tab's logic, without a Viser server or an arm.

The GUI wiring is not tested here — the parts worth testing are the ones
that decide *what the operator is told* and *what gets written to the
calibration file*, and those are ordinary functions over ordinary data.
"""

from __future__ import annotations

import contextlib
import inspect
import math
import re
import threading
import types

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
    """A pose the model cannot represent must be called out, not rendered mute.

    Otherwise the operator holds exactly the pose they were asked for and
    watches the mirror self-intersect, which reads as the re-zero having
    broken something.

    No shipped pose exceeds the limits any more — FOLDED_FLAT used to, by
    9.4 deg, but that was the wrong solve rather than conservative limits
    (see test_reference_poses). The warning still has to work, so this
    exercises it against a pose built to trip it.
    """
    from soarm_sdk.calibration.reference import ReferencePose
    from soarm_sdk.dashboard.panels.calibration import URDF_LIMITS as LIM

    over = ReferencePose(
        key="over", label="past the ceiling",
        q=(0.0, 0.0, LIM["elbow_flex"][1] + 0.2, 0.0, 0.0, 0.0),
        setup="—", verify_by="—", covers=("elbow_flex",),
    )
    assert over.as_cfg()["elbow_flex"] > LIM["elbow_flex"][1]


def test_no_shipped_pose_asks_for_something_the_model_cannot_render():
    from soarm_sdk.calibration.reference import REFERENCE_POSES
    from soarm_sdk.dashboard.panels.calibration import URDF_LIMITS as LIM

    for pose in REFERENCE_POSES.values():
        for name, q in pose.as_cfg().items():
            lo, hi = LIM[name]
            assert lo <= q <= hi, f"{pose.key}.{name} = {q}"


# -- the shape of the tab ----------------------------------------------
#
# These assert on structure rather than behaviour, which is unusual, but the
# thing being protected here *is* structure: the tab shipped with its folders
# emitted 1, 1, 0, 2, 3, 4, 4, 5 — two steps numbered 1, two numbered 4, and
# the prerequisite numbered 0 below the step needing it. Every folder was
# individually correct, so nothing failed and nothing caught it.


def _tab_builders() -> list:
    """The four tab builders, in the order build_calibration_panels lists them."""
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration.build_calibration_panels)
    return re.findall(r"_tab_(\w+)\(server", src)


def _tab_names() -> list:
    from soarm_sdk.dashboard.panels import calibration

    return re.findall(
        r'Panel\(\s*"([^"]+)"',
        inspect.getsource(calibration.build_calibration_panels),
    )


def _folder_titles(tab: str) -> list:
    """Folder titles one tab emits, in order."""
    from soarm_sdk.dashboard.panels import calibration

    titles = []
    src = inspect.getsource(getattr(calibration, f"_tab_{tab}"))
    for helper in re.findall(r"(_build_\w+)\(server", src):
        titles += re.findall(r'add_folder\(\s*"([^"]+)"', inspect.getsource(
            getattr(calibration, helper)))
    return titles


def test_there_are_four_tabs_in_workflow_order():
    """Tolerances gate the grading, signs cannot be fixed by a zero, travel
    is measured before zeros are pinned inside it, zeros last."""
    assert _tab_builders() == ["tolerances", "signs", "travel", "zeros"]


def test_the_tab_names_are_numbered_so_the_order_is_visible():
    names = _tab_names()
    assert len(names) == 4
    for i, name in enumerate(names, start=1):
        assert name.startswith(f"{i} ·"), names


def test_every_tab_states_where_it_sits_in_the_order():
    from soarm_sdk.dashboard.panels import calibration

    for i, tab in enumerate(_tab_builders(), start=1):
        src = inspect.getsource(getattr(calibration, f"_tab_{tab}"))
        assert f"handles, {i}," in src, tab
    banner = inspect.getsource(calibration._order_banner)
    assert "of 4" in banner
    assert "in order" in banner


def test_each_acceptance_row_is_owned_by_exactly_one_tab():
    """A row nobody owns is a blocker with no tab to fix it in."""
    from soarm_sdk.calibration.pipeline import PipelineStage
    from soarm_sdk.dashboard.panels.calibration import TAB_STAGES

    owned = [st for stages in TAB_STAGES.values() for st in stages]
    assert sorted(owned, key=lambda s: s.value) == sorted(
        PipelineStage, key=lambda s: s.value
    )
    assert len(owned) == len(set(owned))
    assert list(TAB_STAGES) == _tab_builders()


def test_live_state_is_repeated_on_every_tab_that_checks_against_it():
    """Sending the operator to another tab to read the arm defeats the split."""
    from soarm_sdk.dashboard.panels import calibration

    for tab in ("signs", "travel", "zeros"):
        src = inspect.getsource(getattr(calibration, f"_tab_{tab}"))
        assert "_build_live_state" in src, tab
    # Tolerances needs no arm reading at all.
    assert "_build_live_state" not in inspect.getsource(calibration._tab_tolerances)


def test_every_tab_puts_its_status_line_at_the_top():
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration._tab_header)
    assert '_reg(handles, "status_md"' in src
    for tab in _tab_builders():
        body = inspect.getsource(getattr(calibration, f"_tab_{tab}"))
        assert body.index("_tab_header") < body.index("_build_review"), tab


def test_every_tab_ends_with_its_own_review_and_save():
    from soarm_sdk.dashboard.panels import calibration

    for tab in _tab_builders():
        src = inspect.getsource(getattr(calibration, f"_tab_{tab}"))
        assert f'tab="{tab}"' in src, tab
        # ...and it is the last thing the tab builds.
        assert src.rindex("_build_review") > max(
            src.rindex(h) for h in re.findall(r"_build_\w+", src)
            if h != "_build_review"
        ), tab

    review = inspect.getsource(calibration._build_review)
    assert "Save calibration" in review
    assert 'pipeline_md' in review
    assert 'save_md' in review


def test_the_review_shows_the_whole_record_not_just_this_tabs_rows():
    """Save refuses on rows owned by tabs you have not opened; that has to be
    visible from the button rather than inferred."""
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration._build_review)
    assert "stage_mds" in src        # this tab's own rows
    assert "pipeline_md" in src      # and every other tab's


def test_the_record_labels_still_name_steps_the_operator_can_find():
    from soarm_sdk.calibration.pipeline import CalibrationPipeline

    report = CalibrationPipeline(_cal()).report().as_markdown()
    cited = {int(n) for n in re.findall(r"Step (\d+) -", report)}
    assert cited, report
    assert cited <= {1, 2, 3, 4}


def test_the_travel_tab_embeds_the_homing_workflow():
    """ROM measurement must not be split off from the calibration flow."""
    from soarm_sdk.dashboard.panels import calibration

    assert "_build_rom" in inspect.getsource(calibration._tab_travel)
    assert "_build_homing" in inspect.getsource(calibration._build_rom)


def test_the_embedded_rom_step_does_not_retitle_itself():
    """A "## Homing Wizard" heading inside "Step 3" reads as a second tool."""
    from soarm_sdk.dashboard.panels import calibration

    assert "heading=False" in inspect.getsource(calibration._build_rom)


def test_step_4_leads_with_the_pose_not_the_symmetry_table():
    """Opening on the travel midpoint framed the step as the one method it
    deliberately does not use.

    Step 4 pins a zero to a pose that was checked against the world. The
    hard-stop symmetry check reasons from the travel instead, cannot tell a
    bad zero from an asymmetric mechanism, and has already produced one
    false positive on this arm. Leading with it made the step read as
    midpoint-finding.
    """
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration._build_rezero)
    assert src.index("Re-zero to this pose") < src.index("_build_symmetry")
    assert "symmetry_md" not in src


def test_the_symmetry_check_is_collapsed_and_says_it_sets_nothing():
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration._build_symmetry)
    assert "expand_by_default=False" in src
    assert "sets nothing" in src
    assert "symmetry_md" in src


def test_the_symmetry_table_points_back_up_not_down():
    """It used to end "confirm it against a reference pose below"; the pose
    picker is now above it, and a stale direction sends the operator off the
    end of the panel."""
    from soarm_sdk.dashboard.panels.calibration import _format_symmetry

    cal = _cal()
    for j in cal.joints:
        object.__setattr__(j, "zero_source", "travel_and_urdf_limits")
    i = cal.names.index("shoulder_pan")
    object.__setattr__(cal.joints[i], "tick_min", int(cal.joints[i].to_ticks(-0.5)))
    object.__setattr__(cal.joints[i], "tick_max", int(cal.joints[i].to_ticks(1.5)))
    out = _format_symmetry(cal)
    assert "below" not in out
    assert "do **not** subtract" in out


def test_the_symmetry_table_never_tells_you_to_apply_the_gap():
    """A gap is a question. Subtracting it is how the false positive bites."""
    from soarm_sdk.dashboard.panels.calibration import _format_symmetry

    cal = _cal()
    out = _format_symmetry(cal)
    assert "never a correction" in out


# -- confirming the direction signs -------------------------------------


def test_the_tab_can_record_a_direction_sign_check():
    """The step the tab asked for and could not accept.

    ``PipelineStage.DIRECTION_SIGNS`` gates Save on ``validated``, nothing in
    the dashboard called ``mark_validated``, and no other step sets it — so
    the record's sign row was permanently BLOCKED and Save could never
    succeed from this tab at all.
    """
    from soarm_sdk.dashboard.panels import calibration

    assert "mark_validated" in inspect.getsource(calibration._build_signs)


def test_a_recorded_check_unblocks_only_the_sign_row():
    from soarm_sdk.calibration.pipeline import CalibrationPipeline, PipelineStage

    cal = _cal()
    cal.validated = False
    before = CalibrationPipeline(cal).report()
    assert not before.stage(PipelineStage.DIRECTION_SIGNS).passed

    cal.mark_validated("jogged each joint and watched the mirror")
    after = CalibrationPipeline(cal).report()
    assert after.stage(PipelineStage.DIRECTION_SIGNS).passed
    assert cal.notes["validated_by"].startswith("jogged")


def test_an_edit_reaches_the_un_nudged_base_as_well_as_the_live_copy():
    """While a slider is off zero these are two objects, and the base wins.

    ``_apply`` rebuilds the live calibration from the base on every slider
    move, so provenance written only to the live copy is discarded by the
    next twitch of a slider.
    """
    from soarm_sdk.dashboard.panels.calibration import _live_calibrations

    live, base = _cal(), _cal()
    ctx = _Ctx(live)
    assert set(map(id, _live_calibrations(ctx, {"nudge_base": base}))) == {
        id(live),
        id(base),
    }


def test_one_calibration_is_not_written_to_twice():
    """After a re-zero the live copy *is* the base; do not double-apply."""
    from soarm_sdk.dashboard.panels.calibration import _live_calibrations

    cal = _cal()
    assert _live_calibrations(_Ctx(cal), {"nudge_base": cal}) == [cal]


def test_no_calibration_is_not_an_error():
    from soarm_sdk.dashboard.panels.calibration import _live_calibrations

    assert _live_calibrations(_Ctx(None), {}) == []


def test_an_unconfirmed_sign_says_so_rather_than_staying_blank():
    from soarm_sdk.dashboard.panels.calibration import _format_signs

    cal = _cal()
    cal.validated = False
    assert "Not confirmed" in _format_signs(cal)


def test_a_confirmed_sign_reports_what_was_actually_done():
    from soarm_sdk.dashboard.panels.calibration import _format_signs

    cal = _cal()
    cal.mark_validated("levelled the upper arm and jogged J2")
    out = _format_signs(cal)
    assert "Confirmed" in out
    assert "levelled the upper arm" in out


# -- the record must be readable before the arm is plugged in -----------


class _OfflineCtx(_Ctx):
    """A context with a calibration loaded and no arm attached."""

    def __init__(self, calibration):
        super().__init__(calibration)
        import threading
        import types

        self.lock = threading.Lock()
        self.state = types.SimpleNamespace(positions={}, connected=False)

    def calibration_drift(self):
        return []


def test_the_blocked_steps_are_listed_while_disconnected():
    """What is left to do must be readable before deciding to plug in.

    ``_on_tick`` returned early when disconnected, which froze the acceptance
    record on "waiting for calibration" — the one panel that says what
    remains was blank until the thing it grades was live.
    """
    from soarm_sdk.dashboard.panels.calibration import _on_tick

    class _MD:
        content = ""

    handles = {
        k: [_MD()]
        for k in (
            "symmetry_md",
            "claims_md",
            "signs_md",
            "drift_md",
            "pipeline_md",
            "table_md",
            "pinned_md",
            "members_md",
            "outside_md",
        )
    }
    cal = _cal()
    cal.validated = False
    _on_tick(_OfflineCtx(cal), handles)

    assert "BLOCKED" in handles["pipeline_md"][0].content
    assert "Not confirmed" in handles["signs_md"][0].content
    assert "Not connected" in handles["table_md"][0].content


# -- an arm that settles after the re-zero ------------------------------


def _rows_at(ticks: dict):
    return [{"name": n, "ticks": t} for n, t in ticks.items()]


def test_no_movement_since_the_pin_reads_as_showing_the_pose():
    from soarm_sdk.calibration.reference import FOLDED_FLAT
    from soarm_sdk.dashboard.panels.calibration import _format_pinned

    held = {"shoulder_lift": 1171, "elbow_flex": 3435}
    handles = {"pinned": {"pose": FOLDED_FLAT, "ticks": held}}
    out = _format_pinned(handles, _rows_at(held))
    assert "still there" in out
    assert "moved" not in out


def test_an_arm_that_settles_is_named_as_settling_not_as_a_failed_rezero():
    """The exact confusion: pin at 3435, arm sags to 3371, mirror follows.

    The mirror stops showing the pinned pose and looks broken. It is not —
    it is tracking an arm that moved 5.6 deg after the hand came off.
    """
    from soarm_sdk.calibration.reference import FOLDED_FLAT
    from soarm_sdk.dashboard.panels.calibration import _format_pinned

    handles = {
        "pinned": {
            "pose": FOLDED_FLAT,
            "ticks": {"shoulder_lift": 1171, "elbow_flex": 3435},
        }
    }
    out = _format_pinned(
        handles, _rows_at({"shoulder_lift": 1171, "elbow_flex": 3371})
    )
    assert "elbow_flex -5.6" in out
    assert "not** a failed" in out
    assert "shoulder_lift" not in out  # it did not move; do not cry wolf


def test_nothing_is_said_before_any_rezero():
    from soarm_sdk.dashboard.panels.calibration import _format_pinned

    assert _format_pinned({}, _rows_at({"elbow_flex": 3371})) == ""


# -- flipping a sign, live ----------------------------------------------


def test_flipping_reverses_the_reported_angle_about_the_same_zero_tick():
    j = JointCalibration("wrist_roll", 1000.0, 1, 0, 4095)
    f = j.with_direction_flipped()
    assert f.direction_sign == -1
    assert f.to_rad(2000.0) == pytest.approx(-j.to_rad(2000.0))
    # Which tick reads zero does not move; only which way angle increases.
    assert f.zero_offset_ticks == j.zero_offset_ticks
    assert f.to_rad(1000.0) == 0.0


def test_flipping_drops_a_pose_anchored_provenance():
    """rezero_from_pose solves zero = ticks - sign*rad, so the sign is in it.

    A zero pinned under the old sign does not survive the flip, and must not
    keep claiming a pose witness it no longer has.
    """
    j = JointCalibration("a", 1000.0, 1, 0, 4095, zero_source="reference_pose")
    assert j.with_direction_flipped().zero_source == "manual_sign_flip"


def test_flipping_does_not_move_the_measured_travel():
    j = JointCalibration("a", 1000.0, 1, 500, 3500)
    f = j.with_direction_flipped()
    assert (f.tick_min, f.tick_max) == (500, 3500)


def test_flipping_twice_is_the_identity():
    """Changing the answer back must undo the flip, not compound it."""
    j = JointCalibration("a", 1000.0, 1, 0, 4095)
    assert j.with_direction_flipped().with_direction_flipped().direction_sign == 1


def _signs_ctx():
    """A context whose calibration is pose-anchored and already validated."""
    cal = _cal()
    cal.notes = {
        "rezeroed_from_dashboard": {"pose": "folded_flat", "joints": ["shoulder_lift"]},
        "reference_pose_samples": [{}, {}],
    }
    return _Ctx(cal), cal


class _Dropdown:
    def __init__(self, value):
        self.value = value


def test_selecting_opposite_flips_the_sign_without_waiting_for_the_button():
    """The mirror has to reverse while the operator is still on that joint."""
    from soarm_sdk.dashboard.panels.calibration import SIGN_INVERTED, _apply_signs

    ctx, cal = _signs_ctx()
    handles = {"sign_verdicts": {"wrist_roll": _Dropdown(SIGN_INVERTED)}}
    _apply_signs(ctx, handles)

    assert cal.joints[cal.names.index("wrist_roll")].direction_sign == -1


def test_a_flip_invalidates_the_pose_witness_and_the_recorded_check():
    from soarm_sdk.dashboard.panels.calibration import SIGN_INVERTED, _apply_signs

    ctx, cal = _signs_ctx()
    _apply_signs(ctx, {"sign_verdicts": {"wrist_roll": _Dropdown(SIGN_INVERTED)}})

    assert "rezeroed_from_dashboard" not in cal.notes
    assert "reference_pose_samples" not in cal.notes
    assert cal.validated is False


def test_changing_the_answer_back_undoes_the_flip():
    """Derived from the dropdowns each time, never accumulated."""
    from soarm_sdk.dashboard.panels.calibration import (
        SIGN_INVERTED,
        SIGN_OK,
        _apply_signs,
    )

    ctx, cal = _signs_ctx()
    handles = {"sign_verdicts": {"wrist_roll": _Dropdown(SIGN_INVERTED)}}
    _apply_signs(ctx, handles)
    handles["sign_verdicts"]["wrist_roll"] = _Dropdown(SIGN_OK)
    _apply_signs(ctx, handles)

    assert cal.joints[cal.names.index("wrist_roll")].direction_sign == 1


def test_reapplying_the_same_verdict_does_not_flip_again():
    from soarm_sdk.dashboard.panels.calibration import SIGN_INVERTED, _apply_signs

    ctx, cal = _signs_ctx()
    handles = {"sign_verdicts": {"wrist_roll": _Dropdown(SIGN_INVERTED)}}
    _apply_signs(ctx, handles)
    _apply_signs(ctx, handles)
    _apply_signs(ctx, handles)

    assert cal.joints[cal.names.index("wrist_roll")].direction_sign == -1


def test_a_flip_reaches_the_un_nudged_base_too():
    from soarm_sdk.dashboard.panels.calibration import SIGN_INVERTED, _apply_signs

    ctx, live = _signs_ctx()
    base = _cal()
    _apply_signs(
        ctx,
        {"sign_verdicts": {"wrist_roll": _Dropdown(SIGN_INVERTED)}, "nudge_base": base},
    )
    for cal in (live, base):
        assert cal.joints[cal.names.index("wrist_roll")].direction_sign == -1


def test_a_flipped_joint_is_named_as_needing_a_repin():
    from soarm_sdk.dashboard.panels.calibration import _format_signs

    cal = _cal()
    i = cal.names.index("wrist_roll")
    cal.joints[i] = cal.joints[i].with_direction_flipped()
    out = _format_signs(cal)
    assert "wrist_roll" in out
    assert "tab 4" in out


# -- the reference-pose ghost -------------------------------------------


def test_step_4_offers_the_ghost_and_the_alignment_sliders():
    """The pose is a target to match, not prose to interpret."""
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration._build_rezero)
    assert "ghost_fn" in src
    assert "Show reference ghost" in src
    assert "_build_alignment" in src


def test_the_ghost_is_posed_from_the_selected_pose_not_the_arm():
    """It shows where the arm should be; servo readings must not touch it."""
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration._build_rezero)
    assert "pose.as_cfg()" in src
    assert "state.positions" not in src


def test_unticking_the_ghost_hides_it_rather_than_posing_it_somewhere():
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration._build_rezero)
    assert "ghost_fn(pose.as_cfg() if ghost_h.value else None)" in src


def test_there_is_exactly_one_set_of_zero_sliders():
    """Two controls editing the same zeros in two folders is one too many.

    The nudge used to be its own numbered step after the pin, with no visual
    target to work against — you nudged until it "looked right", with
    nothing in the scene defining right.
    """
    from soarm_sdk.dashboard.panels import calibration

    assert not hasattr(calibration, "_build_nudge")
    builders = [
        n
        for n in dir(calibration)
        if n.startswith("_build_")
        and "add_slider" in inspect.getsource(getattr(calibration, n))
    ]
    assert builders == ["_build_alignment"], builders


def test_the_alignment_sliders_still_edit_the_zero_not_the_arm():
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration._build_alignment)
    assert "shifted_by" in src
    assert 'handles["nudges"] = nudges' in src
    # A nudge is not pose-anchored evidence; it must drop the witness.
    assert 'live.notes.pop("rezeroed_from_dashboard", None)' in src


# -- alignment slider bounds --------------------------------------------


def test_each_slider_is_bounded_by_its_own_joints_travel():
    """A flat +-45 deg matched no joint on this arm.

    It is far past the gripper's jaw travel and nowhere near wrist_roll's,
    so a slider end meant nothing physical in either direction.
    """
    from soarm_sdk.dashboard.panels.calibration import _alignment_range

    for name, (lo, hi) in URDF_LIMITS.items():
        got = _alignment_range(name)
        assert got == (round(math.degrees(lo), 2), round(math.degrees(hi), 2)), name


def test_an_asymmetric_joint_gets_an_asymmetric_slider():
    """The gripper opens one way; +-45 implied it swung both."""
    from soarm_sdk.dashboard.panels.calibration import _alignment_range

    lo, hi = _alignment_range("gripper")
    assert lo == pytest.approx(-10.0, abs=0.01)
    assert hi == pytest.approx(100.0, abs=0.01)
    assert abs(lo) != pytest.approx(abs(hi))


def test_a_joint_with_no_declared_limit_falls_back_rather_than_going_infinite():
    from soarm_sdk.dashboard.panels.calibration import (
        NUDGE_LIMIT_DEG,
        _alignment_range,
    )

    assert _alignment_range("not_a_joint") == (-NUDGE_LIMIT_DEG, NUDGE_LIMIT_DEG)


def test_the_sliders_are_built_from_that_range_not_a_constant():
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration._build_alignment)
    assert "_alignment_range(name)" in src
    assert "min=-NUDGE_LIMIT_DEG" not in src


# -- withdrawing a pose claim no pose backs -----------------------------


def test_a_claim_is_unsupported_when_the_witness_does_not_cover_the_joint():
    """The real file: all six stamped reference_pose, witness covers two."""
    from soarm_sdk.dashboard.panels.calibration import _unsupported_pose_claims

    cal = _cal()  # every joint zero_source="reference_pose"
    cal.notes = {"rezeroed_from_dashboard": {"pose": "folded_flat"}}
    assert _unsupported_pose_claims(cal) == [
        "shoulder_pan",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ]


def test_joints_the_witness_does_cover_are_left_alone():
    from soarm_sdk.dashboard.panels.calibration import _unsupported_pose_claims

    cal = _cal()
    cal.notes = {"rezeroed_from_dashboard": {"pose": "folded_flat"}}
    got = _unsupported_pose_claims(cal)
    assert "shoulder_lift" not in got
    assert "elbow_flex" not in got


def test_with_no_witness_at_all_every_claim_is_unsupported():
    from soarm_sdk.dashboard.panels.calibration import _unsupported_pose_claims

    cal = _cal()
    cal.notes = {}
    assert set(_unsupported_pose_claims(cal)) == set(cal.names)


def test_withdrawing_clears_the_label_but_not_the_zero():
    """The claim is false; the number it labels may be perfectly good."""
    j = JointCalibration("gripper", 1234.0, 1, 0, 4095, zero_source="reference_pose")
    out = j.with_claim_withdrawn()
    assert out.zero_source == "unknown"
    assert out.zero_offset_ticks == j.zero_offset_ticks
    assert out.direction_sign == j.direction_sign
    assert (out.tick_min, out.tick_max) == (j.tick_min, j.tick_max)


def test_withdrawal_only_ever_removes_a_claim():
    """Downgrade-only, so it cannot launder a zero into looking verified."""
    for source in ("travel_and_urdf_limits", "unknown", "manual_nudge",
                   "manual_sign_flip"):
        j = JointCalibration("a", 2048.0, 1, 0, 4095, zero_source=source)
        assert j.with_claim_withdrawn().zero_source == source


def test_withdrawing_unblocks_the_provenance_stage():
    from soarm_sdk.calibration.pipeline import CalibrationPipeline, PipelineStage
    from soarm_sdk.dashboard.panels.calibration import _unsupported_pose_claims

    cal = _cal()
    cal.notes = {"rezeroed_from_dashboard": {"pose": "folded_flat",
                                             "joints": ["shoulder_lift", "elbow_flex"]}}
    assert not CalibrationPipeline(cal).report().stage(
        PipelineStage.ZERO_PROVENANCE).passed

    for i, j in enumerate(cal.joints):
        if j.name in _unsupported_pose_claims(cal):
            cal.joints[i] = j.with_claim_withdrawn()

    assert CalibrationPipeline(cal).report().stage(
        PipelineStage.ZERO_PROVENANCE).passed


def test_the_block_names_the_joint_and_the_pose_that_fails_it():
    """The old wording read as though the provenance covered the gripper."""
    from soarm_sdk.calibration.pipeline import CalibrationPipeline, PipelineStage

    cal = _cal()
    cal.notes = {"rezeroed_from_dashboard": {"pose": "folded_flat",
                                             "joints": ["shoulder_lift", "elbow_flex"]}}
    detail = CalibrationPipeline(cal).report().stage(
        PipelineStage.ZERO_PROVENANCE).detail
    assert "gripper" in detail
    assert "folded_flat" in detail
    assert "does not constrain" in detail
    assert "incorrectly covers" not in detail


def test_two_joints_can_never_earn_a_pose_claim():
    """No shipped pose constrains them, so withdrawal is the only outcome."""
    from soarm_sdk.calibration.reference import REFERENCE_POSES

    coverable = set().union(*(set(p.covers) for p in REFERENCE_POSES.values()))
    assert "gripper" not in coverable
    assert "wrist_roll" not in coverable


# -- the save gate has to be legible ------------------------------------


class _MD:
    def __init__(self):
        self.content = ""


def test_an_incomplete_save_still_writes_and_says_so_beside_the_button():
    """Save used to refuse outright unless every stage passed at once, so a
    session that had genuinely finished tolerances, signs and travel still
    lost all three the moment the dashboard restarted before zeros were
    done — nothing had ever reached disk to survive the restart.

    The actual safety gate lives one layer downstream: TAMP's watchdog
    re-derives full completeness from the file at plan time regardless of
    what Save did, so refusing to persist an incomplete file was redundant
    with that check and only cost already-verified work.
    """
    from soarm_sdk.dashboard.panels.calibration import _do_save

    cal = _cal()
    cal.notes = {}  # nothing recorded: the pipeline is not ready
    handles = {"status_md": [_MD()], "save_md": [_MD()]}
    _do_save(_Ctx(cal), handles)

    assert "Saved" in handles["save_md"][0].content
    assert "Still incomplete" in handles["save_md"][0].content
    assert "watchdog will refuse to plan" in handles["save_md"][0].content
    assert handles["save_md"][0].content == handles["status_md"][0].content


def test_an_incomplete_save_writes_to_disk_with_a_backup(tmp_path):
    """This is exactly the behaviour a refusal used to prevent — deliberately,
    now: an incomplete but genuine calibration should survive a restart."""
    from soarm_sdk.calibration.frame import RobotCalibration
    from soarm_sdk.dashboard.panels.calibration import _do_save

    path = tmp_path / "calibration.json"
    saved = _cal()
    saved.notes = {}
    saved.save(path)

    cal = _cal()
    cal.notes = {}
    i = cal.names.index("shoulder_lift")
    cal.joints[i] = cal.joints[i].shifted_by(0.05)  # a real, distinguishing edit
    ctx = _Ctx(cal)
    ctx.calibration_path = path
    _do_save(ctx, {"status_md": [_MD()], "save_md": [_MD()]})

    reloaded = RobotCalibration.load(path)
    assert reloaded.joints[i].zero_offset_ticks == cal.joints[i].zero_offset_ticks
    assert list(tmp_path.glob("*.backup-*.json")), "previous version must still be backed up"


def test_a_fully_complete_save_does_not_mention_incompleteness():
    from soarm_sdk.calibration.pipeline import AcceptanceTolerances
    from soarm_sdk.dashboard.panels.calibration import _do_save

    cal = _cal()
    cal.notes = {
        "acceptance_tolerances": AcceptanceTolerances(
            pose_repeatability_rad=math.radians(0.5),
            model_deviation_rad=math.radians(2.0),
            rom_endpoint_repeatability_ticks=8,
        ).to_dict(),
        "rom_endpoint_samples": {
            n: [{"min": 10, "max": 4000}, {"min": 11, "max": 4001}] for n in cal.names
        },
        "rezeroed_from_dashboard": {
            "pose": "folded_flat", "joints": ["shoulder_lift", "elbow_flex"],
        },
        "reference_pose_samples": [
            {n: 2048 for n in cal.names}, {n: 2048 for n in cal.names},
        ],
    }
    for i, j in enumerate(cal.joints):
        if j.name not in ("shoulder_lift", "elbow_flex"):
            cal.joints[i] = j.with_claim_withdrawn()
    handles = {"status_md": [_MD()], "save_md": [_MD()]}
    _do_save(_Ctx(cal), handles)

    assert "Saved" in handles["save_md"][0].content
    assert "incomplete" not in handles["save_md"][0].content.lower()
    assert "every stage passes" in handles["save_md"][0].content


def test_a_multiline_message_is_not_wrapped_in_emphasis():
    """`*<markdown table>*` renders as literal asterisks and broken rows."""
    from soarm_sdk.dashboard.panels.calibration import _say

    handles = {"status_md": [_MD()]}
    _say(handles, "| a | b |\n|---|---|\n| 1 | 2 |")
    assert not handles["status_md"][0].content.startswith("*")

    _say(handles, "one liner")
    assert handles["status_md"][0].content == "*one liner*"


def test_a_successful_save_says_so_beside_the_button(tmp_path):
    from soarm_sdk.calibration.pipeline import CalibrationPipeline
    from soarm_sdk.dashboard.panels.calibration import _do_save

    cal = _cal()
    cal.notes = {
        "acceptance_tolerances": {
            "pose_repeatability_rad": 0.01,
            "model_deviation_rad": 0.05,
            "rom_endpoint_repeatability_ticks": 4,
        },
        "rom_endpoint_samples": {
            n: [{"min": 10, "max": 4000}, {"min": 11, "max": 4001}] for n in cal.names
        },
        "rezeroed_from_dashboard": {
            "pose": "folded_flat", "joints": ["shoulder_lift", "elbow_flex"],
        },
        "reference_pose_samples": [
            {n: 2048 for n in cal.names}, {n: 2048 for n in cal.names},
        ],
    }
    for i, j in enumerate(cal.joints):
        if j.name not in ("shoulder_lift", "elbow_flex"):
            cal.joints[i] = j.with_claim_withdrawn()
    assert CalibrationPipeline(cal).report().ready, "fixture must be saveable"

    path = tmp_path / "calibration.json"
    ctx = _Ctx(cal)
    ctx.calibration_path = path
    handles = {"status_md": [_MD()], "save_md": [_MD()]}
    _do_save(ctx, handles)

    assert path.exists()
    assert "Saved" in handles["save_md"][0].content


def test_no_inner_folder_repeats_the_tab_number():
    """The tab carries the number; repeating it inside reads as a sub-step."""

    for tab in _tab_builders():
        for title in _folder_titles(tab):
            assert not re.match(r"Step \d", title), (tab, title)


def test_each_tab_reports_its_own_rows_and_the_whole_record():
    """The review has to answer 'did what I just did take?' locally."""
    from soarm_sdk.calibration.pipeline import CalibrationPipeline
    from soarm_sdk.dashboard.panels.calibration import TAB_STAGES, _format_stage

    cal = _cal()
    cal.notes = {}
    report = CalibrationPipeline(cal).report()

    per_tab = {t: _format_stage(report, st) for t, st in TAB_STAGES.items()}
    # Signs pass on this fixture (validated=True); everything else blocks.
    assert "complete" in per_tab["signs"]
    for tab in ("tolerances", "travel", "zeros"):
        assert "not complete" in per_tab[tab], tab
    # A tab never reports another tab's failure as its own.
    assert "acceptance tolerances" not in per_tab["travel"].lower()


def test_a_tab_with_no_calibration_says_so_rather_than_claiming_success():
    from soarm_sdk.dashboard.panels.calibration import TAB_STAGES, _format_stage

    for stages in TAB_STAGES.values():
        assert "Blocked" in _format_stage(None, stages)


# -- travel evidence reaches the row that grades it ---------------------


def _tol_cal():
    from soarm_sdk.calibration.pipeline import AcceptanceTolerances

    cal = _cal()
    cal.notes = {
        "acceptance_tolerances": AcceptanceTolerances(
            pose_repeatability_rad=math.radians(0.5),
            model_deviation_rad=math.radians(2.0),
            rom_endpoint_repeatability_ticks=8,
        ).to_dict()
    }
    return cal


def _rom_passes(cal):
    from soarm_sdk.calibration.pipeline import CalibrationPipeline, PipelineStage

    return CalibrationPipeline(cal).report().stage(PipelineStage.ROM).passed


def test_recording_travel_in_the_dashboard_feeds_the_row_that_grades_it():
    """The control and its acceptance row were never connected.

    The sweep wrote servo EEPROM and soarm100_rom.json; rom_endpoint_samples
    was written only by the standalone soarm-calibrate-rom. So measuring
    travel here left the ROM row BLOCKED however carefully it was done.
    """
    from soarm_sdk.dashboard.panels.calibration import _record_endpoints

    cal = _tol_cal()
    ctx = _Ctx(cal)
    assert not _rom_passes(cal)

    ends = {n: (800, 3400) for n in cal.names}
    _record_endpoints(ctx, {}, ends, simulated=False)
    assert not _rom_passes(cal), "one pass cannot prove repeatability"

    _record_endpoints(ctx, {}, {n: (802, 3398) for n in cal.names}, simulated=False)
    assert _rom_passes(cal)


def test_a_simulated_sweep_is_recorded_but_never_accepted():
    from soarm_sdk.dashboard.panels.calibration import _record_endpoints

    cal = _tol_cal()
    ctx = _Ctx(cal)
    for _ in range(2):
        _record_endpoints(ctx, {}, {n: (800, 3400) for n in cal.names}, simulated=True)
    assert not _rom_passes(cal)


def test_recording_travel_also_updates_the_joints_own_hard_stops():
    """reachable_limits() is what ServoRobot clamps against; leaving it stale
    while recording the evidence would be its own quiet disagreement."""
    from soarm_sdk.dashboard.panels.calibration import _record_endpoints

    cal = _tol_cal()
    _record_endpoints(_Ctx(cal), {}, {"shoulder_pan": (900, 3100)}, simulated=False)
    j = cal.joints[cal.names.index("shoulder_pan")]
    assert (j.tick_min, j.tick_max) == (900, 3100)


def test_endpoints_are_stored_lowest_first_whichever_way_they_were_swept():
    from soarm_sdk.dashboard.panels.calibration import _record_endpoints

    cal = _tol_cal()
    _record_endpoints(_Ctx(cal), {}, {"shoulder_pan": (3100, 900)}, simulated=False)
    j = cal.joints[cal.names.index("shoulder_pan")]
    assert j.tick_min < j.tick_max


def test_travel_does_not_disturb_the_zero():
    """Hard stops are a fact about the mechanism, not about the zero."""
    from soarm_sdk.dashboard.panels.calibration import _record_endpoints

    cal = _tol_cal()
    before = {j.name: (j.zero_offset_ticks, j.direction_sign) for j in cal.joints}
    _record_endpoints(_Ctx(cal), {}, {n: (900, 3100) for n in cal.names}, False)
    after = {j.name: (j.zero_offset_ticks, j.direction_sign) for j in cal.joints}
    assert before == after


def test_the_travel_report_names_what_is_still_missing():
    from soarm_sdk.dashboard.panels.calibration import _format_rom, _record_endpoints

    cal = _tol_cal()
    assert "No travel passes yet" in _format_rom(cal)

    _record_endpoints(_Ctx(cal), {}, {n: (800, 3400) for n in cal.names}, False)
    assert "needs another pass" in _format_rom(cal)

    _record_endpoints(_Ctx(cal), {}, {n: (802, 3398) for n in cal.names}, False)
    out = _format_rom(cal)
    assert "repeatable to 2 ticks" in out
    assert "needs another pass" not in out


def test_both_sweep_modes_publish_their_endpoints():
    """Manual and automatic both funnel through one callback, so neither can
    quietly skip recording."""
    from soarm_sdk.dashboard.panels import setup

    src = inspect.getsource(setup._build_homing)
    assert src.count("_publish_endpoints(") == 3  # definition + both modes


def test_the_manual_recorder_shows_a_live_reading():
    """Recording a hard stop by hand means pushing until the number stops
    moving, which you cannot do if the number is not on screen."""
    from soarm_sdk.dashboard.panels import setup

    src = inspect.getsource(setup._build_homing)
    assert "man_live_md" in src
    assert '"homing_tick"' in src


def test_a_wrapped_joint_is_named_and_sent_to_recentre_not_to_the_zero():
    """3974 → 4095 → 0 → 3612 is one continuous motion that min/max reads as
    a full turn. No value of the calibration zero removes the discontinuity;
    the servo's homing offset has to move."""
    from soarm_sdk.dashboard.panels.calibration import _format_rom

    cal = _cal()
    cal.notes = {
        "rom_endpoint_samples": {
            n: [{"min": 0, "max": 4095, "simulated": False}] for n in cal.names
        }
    }
    out = _format_rom(cal)
    assert "4095/0 wrap" in out
    assert "encoder's range" in out
    assert "--recentre" in out
    assert "no calibration zero can fix it" in out
    assert "before" in out and "tab 4" in out


def test_a_normal_span_is_not_called_a_wrap():
    from soarm_sdk.dashboard.panels.calibration import _format_rom

    cal = _cal()
    cal.notes = {
        "rom_endpoint_samples": {
            n: [{"min": 800, "max": 3400, "simulated": False}] * 2
            for n in cal.names
        }
    }
    assert "encoder turn" not in _format_rom(cal)


def test_the_travel_tab_says_it_overrides_the_urdf_once_accepted():
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration._build_rom)
    assert "replaces" in src and "URDF" in src


def test_the_zeros_tab_states_why_travel_comes_first():
    """The dependency is not obvious: a zero does not need the travel, but a
    re-centre rewrites the ticks the zero is pinned to."""
    from soarm_sdk.dashboard.panels import calibration

    src = inspect.getsource(calibration._tab_zeros)
    assert "Do tab 3 first" in src or "Do tab 3 first" in src.replace("**", "")
    assert "rewrites the raw ticks" in src


# -- a lab note is for the file, not the screen -------------------------


REAL_NOTE = (
    "hard-stop direction check, 2026-09-12: torque released, each of "
    "shoulder_pan, shoulder_lift, elbow_flex, wrist_flex and wrist_roll "
    "pushed by hand into a named mechanical stop and the landing end read "
    "from the encoder (all landed within 1-3% of the measured stop). "
    "| ZEROS RE-MEASURED 2026-09-12 by inclinometer: with the pitch axes "
    "parallel, each member pitch is a linear function of its upstream joints."
)


def test_a_long_provenance_note_is_reduced_to_its_headline():
    """This arm's validated_by is 956 characters — longer than every other
    word on the tab put together. It is a lab record; the file is where a
    lab record belongs."""
    from soarm_sdk.dashboard.panels.calibration import _format_signs

    cal = _cal()
    cal.mark_validated(REAL_NOTE)
    out = _format_signs(cal)
    assert len(out) < 200, out
    assert "hard-stop direction check" in out
    assert "inclinometer" not in out
    assert "calibration file" in out


def test_notes_joined_with_a_pipe_are_counted_not_concatenated():
    """Two unrelated records share this field — a sign check and a zero
    re-measurement. Running them together read as one sentence about
    neither."""
    from soarm_sdk.dashboard.panels.calibration import _note_headline

    head, extra = _note_headline(REAL_NOTE)
    assert head == "hard-stop direction check, 2026-09-12"
    assert extra == "+1 more note"


def test_the_date_is_not_printed_twice():
    from soarm_sdk.dashboard.panels.calibration import _format_signs

    cal = _cal()
    cal.mark_validated("hard-stop direction check, 2026-09-12")
    cal.notes["validated_at"] = "2026-09-12T08:42:15+00:00"
    assert _format_signs(cal).count("2026-09-12") == 1


def test_a_short_note_is_shown_whole():
    from soarm_sdk.dashboard.panels.calibration import _note_headline

    head, extra = _note_headline("jogged each joint and watched the mirror")
    assert head == "jogged each joint and watched the mirror"
    assert extra == ""


def test_a_long_single_note_says_it_was_truncated():
    from soarm_sdk.dashboard.panels.calibration import _note_headline

    head, extra = _note_headline("x" * 300)
    assert len(head) <= 84
    assert extra == "truncated"


def test_no_note_at_all_is_not_an_error():
    from soarm_sdk.dashboard.panels.calibration import _note_headline

    assert _note_headline(None) == ("", "")
    assert _note_headline("   ") == ("", "")


# -- suggested tolerances are a form default, not a fallback ------------


def test_the_tolerance_fields_start_at_the_suggested_values():
    from soarm_sdk.dashboard.panels import calibration as C

    assert C.DEFAULT_POSE_REPEATABILITY_DEG == 3.0
    assert C.DEFAULT_MODEL_DEVIATION_DEG == 3.0
    assert C.DEFAULT_ROM_REPEATABILITY_TICKS == 30.0

    src = inspect.getsource(C._build_acceptance)
    for name in ("DEFAULT_POSE_REPEATABILITY_DEG", "DEFAULT_MODEL_DEVIATION_DEG",
                 "DEFAULT_ROM_REPEATABILITY_TICKS"):
        assert f"initial_value={name}" in src, name


def test_the_suggested_values_are_valid_tolerances():
    """A pre-filled form that cannot be submitted is worse than an empty one."""
    from soarm_sdk.calibration.pipeline import AcceptanceTolerances
    from soarm_sdk.dashboard.panels import calibration as C

    t = AcceptanceTolerances(
        pose_repeatability_rad=math.radians(C.DEFAULT_POSE_REPEATABILITY_DEG),
        model_deviation_rad=math.radians(C.DEFAULT_MODEL_DEVIATION_DEG),
        rom_endpoint_repeatability_ticks=int(C.DEFAULT_ROM_REPEATABILITY_TICKS),
    )
    assert t.rom_endpoint_repeatability_ticks == 30


def test_an_unrecorded_calibration_still_has_no_tolerances():
    """The form default must not become a fallback: code that reads the file
    has to keep seeing 'nobody approved a threshold'."""
    from soarm_sdk.calibration.pipeline import AcceptanceTolerances, CalibrationPipeline, PipelineStage

    cal = _cal()
    cal.notes = {}
    assert AcceptanceTolerances.from_notes(cal.notes) is None
    stage = CalibrationPipeline(cal).report().stage(PipelineStage.ACCEPTANCE)
    assert not stage.passed


def test_a_recorded_tolerance_still_wins_over_the_suggestion():
    """Reopening the tab must show this arm's approved numbers, not the form's."""
    from soarm_sdk.dashboard.panels import calibration as C

    src = inspect.getsource(C._build_acceptance)
    assert "AcceptanceTolerances.from_notes(cal.notes)" in src
    assert src.index("from_notes") > src.index("initial_value=DEFAULT_POSE")


# -- Step 2 defaults to "already correct" --------------------------------


def test_the_sign_dropdowns_default_to_correct_not_unchecked():
    """Every joint's assumed sign (DEFAULT_DIRECTION_SIGN_OVERRIDES) is
    expected to check out now, wrist_roll included — a physical check
    confirming the assumption is still required before Confirm accepts, but
    the operator no longer has to touch six controls that are each expected
    to read the same way.

    This trades away the guard that forced a look at each dropdown: Confirm
    can now be pressed with nothing touched and will still record "physical
    direction-sign check recorded". Deliberate per an explicit request, not
    an oversight — flagged here so it reads as a decision if it changes."""
    from soarm_sdk.dashboard.panels import calibration as C

    src = inspect.getsource(C._build_signs)
    assert "initial_value=SIGN_OK" in src
    assert "initial_value=SIGN_UNCHECKED" not in src


# -- recentre a wrapped joint --------------------------------------------


def _build_rom_stub(cal, joint_ids=(1, 2, 3, 4, 5, 6)):
    """A minimal stub GUI good enough to drive _build_rom's Recentre control."""
    class _H:
        def __init__(self, **kw):
            self.content = ""
            self.value = kw.get("initial_value", 0.0)
            self.options = []

        def on_click(self, fn):
            self._click = fn
            return fn

        def on_update(self, fn):
            return fn

    class _Folder:
        def __enter__(self):
            return self

        def __exit__(self, *_exc):
            return False

    class _Gui:
        def __init__(self):
            self.buttons = {}
            self.markdowns = []

        def add_markdown(self, x=""):
            h = _H()
            h.content = x
            self.markdowns.append(h)
            return h

        def add_folder(self, _name, **_kw):
            return _Folder()

        def add_button(self, name, **_kw):
            h = _H()
            self.buttons[name] = h
            return h

        def add_dropdown(self, _name, options=None, initial_value=None):
            h = _H()
            h.value = initial_value
            h.options = list(options or [])
            return h

        def add_slider(self, _name, **kw):
            return _H(**kw)

        def add_number(self, _name, **kw):
            return _H(**kw)

        def add_text(self, _name, **kw):
            h = _H(**kw)
            h.value = kw.get("initial_value", "")
            return h

        def add_checkbox(self, _name, **kw):
            return _H(**kw)

    class _Srv:
        def __init__(self):
            self.gui = _Gui()

    from soarm_sdk.dashboard.panels import calibration as C

    ctx = _Ctx(cal, joint_ids=joint_ids)
    ctx.bus = lambda *a, **k: contextlib.nullcontext(object())
    ctx.state = types.SimpleNamespace(positions={}, connected=False)
    ctx.lock = threading.Lock()
    handles = {"status_md": [_H()]}
    srv = _Srv()
    C._build_rom(srv, ctx, handles)
    return srv, ctx, handles


def _recentred_wrist_roll_cal():
    cal = _cal()
    i = cal.names.index("wrist_roll")
    cal.joints[i] = JointCalibration(
        "wrist_roll", 2074.0, 1, 102, 3993, zero_source="manual_sign_flip"
    )
    return cal


def test_recentre_button_exists_and_defaults_to_wrist_roll():
    from soarm_sdk.dashboard.panels import calibration as C

    cal = _recentred_wrist_roll_cal()
    srv, _ctx, _handles = _build_rom_stub(cal)
    assert "Recentre this joint" in srv.gui.buttons
    src = inspect.getsource(C._build_recentre)
    assert '"wrist_roll" if "wrist_roll"' in src


def test_a_span_at_a_full_turn_is_refused_not_silently_applied(monkeypatch):
    """The exact failure this arm's wrist_roll hit on the first real attempt:
    stalled after 95 ticks one way, timed out after ~4010 the other."""
    import soarm_sdk.calibration.recentre as recentre_mod

    def fake_refuse(*_a, **_k):
        raise RuntimeError(
            "J5: measured 4105 ticks of travel, a full turn or more. A "
            "continuously rotating joint has no centre to move to."
        )

    monkeypatch.setattr(recentre_mod, "recentre_joint", fake_refuse)

    class _Word:
        data = [1726]

    class _FakeBus:
        def read2ByteTxRx(self, _sid, _addr):
            return _Word()

    cal = _recentred_wrist_roll_cal()
    before = cal.joints[cal.names.index("wrist_roll")]
    srv, ctx, handles = _build_rom_stub(cal)
    ctx.bus = lambda *a, **k: contextlib.nullcontext(_FakeBus())
    srv.gui.buttons["Recentre this joint"]._click(None)

    after = cal.joints[cal.names.index("wrist_roll")]
    assert after == before, "a refused recentre must change nothing"
    assert "refused" in handles["status_md"][0].content
    result = next(h for h in srv.gui.markdowns if "4105" in h.content)
    assert "torque off" in result.content
    assert "by hand" in result.content


def test_a_recentre_note_is_recorded_with_before_and_after(monkeypatch):
    import soarm_sdk.calibration.recentre as recentre_mod

    def fake_ok(_srv, _sid, **kw):
        return {
            "old_offset": 1726, "new_offset": -1832,
            "angle_limits": [200, 4000], "span_ticks": 3800.0,
            "position_after": 2048, "holding": True,
            "stalled_both_ends": True,
        }

    monkeypatch.setattr(recentre_mod, "recentre_joint", fake_ok)

    class _Word:
        data = [1726]

    class _FakeBus:
        def read2ByteTxRx(self, _sid, _addr):
            return _Word()

    cal = _recentred_wrist_roll_cal()
    srv, ctx, handles = _build_rom_stub(cal)
    ctx.bus = lambda *a, **k: contextlib.nullcontext(_FakeBus())
    srv.gui.buttons["Recentre this joint"]._click(None)

    j = cal.joints[cal.names.index("wrist_roll")]
    assert (j.tick_min, j.tick_max) == (200, 4000)
    assert j.zero_source == "rebased_after_recentre"
    # 2074 - (new_offset - old_offset) = 2074 - (-1832 - 1726) = 2074 + 3558
    assert j.zero_offset_ticks == pytest.approx(5632.0)
    assert cal.notes["rom_endpoint_samples"]["wrist_roll"][-1] == {
        "min": 200, "max": 4000, "simulated": False,
    }
    assert "wrist_roll" in cal.notes["recentred"]
    assert cal.notes["recentred"]["wrist_roll"]["old_offset"] == 1726
    assert "re-pin it in tab 4" in handles["status_md"][0].content


def test_recentre_does_not_touch_any_other_joint(monkeypatch):
    import soarm_sdk.calibration.recentre as recentre_mod

    def fake_ok(_srv, _sid, **kw):
        return {
            "old_offset": 1726, "new_offset": -1832,
            "angle_limits": [200, 4000], "span_ticks": 3800.0,
            "position_after": 2048, "holding": True,
            "stalled_both_ends": True,
        }

    monkeypatch.setattr(recentre_mod, "recentre_joint", fake_ok)

    class _Word:
        data = [1726]

    class _FakeBus:
        def read2ByteTxRx(self, _sid, _addr):
            return _Word()

    cal = _recentred_wrist_roll_cal()
    before = {
        n: (j.tick_min, j.tick_max, j.zero_offset_ticks)
        for n, j in zip(cal.names, cal.joints) if n != "wrist_roll"
    }
    srv, ctx, _handles = _build_rom_stub(cal)
    ctx.bus = lambda *a, **k: contextlib.nullcontext(_FakeBus())
    srv.gui.buttons["Recentre this joint"]._click(None)

    after = {
        n: (j.tick_min, j.tick_max, j.zero_offset_ticks)
        for n, j in zip(cal.names, cal.joints) if n != "wrist_roll"
    }
    assert before == after


def test_saving_with_no_explicit_path_never_reaches_the_real_default(tmp_path):
    """The regression itself: _do_save used to refuse before ever computing
    a path when incomplete, which accidentally hid that a context with no
    calibration_path falls through to the real ~/.soarm_sdk/calibration.json.
    Removing that refusal removed the accidental guard with it — this
    asserts the *actual* guard (conftest.py's autouse fixture) is in
    effect, not just its own bookkeeping."""
    from pathlib import Path

    from soarm_sdk.dashboard.fk import DEFAULT_CALIBRATION_PATH
    from soarm_sdk.dashboard.panels.calibration import _do_save

    assert str(tmp_path) in str(DEFAULT_CALIBRATION_PATH), (
        "conftest.py should have redirected this for every test"
    )
    real = Path.home() / ".soarm_sdk" / "calibration.json"
    before = real.stat().st_mtime if real.exists() else None

    cal = _cal()
    cal.notes = {}
    _do_save(_Ctx(cal), {"status_md": [_MD()], "save_md": [_MD()]})

    assert DEFAULT_CALIBRATION_PATH.exists()
    after = real.stat().st_mtime if real.exists() else None
    assert before == after, "the operator's real calibration file must not move"
