"""Unit tests for the dashboard's tick -> URDF-frame mapping.

The 3-D view was rendering raw ticks against a nominal zero of 2048 with no
direction signs. On a real arm that is wrong by tens of degrees per joint and
inverts any joint whose servo turns opposite the URDF, which looks like the
model and the robot disagreeing when in fact only the view is uncalibrated.
"""

from __future__ import annotations

import pytest

pytest.importorskip("viser")

from soarm_sdk.calibration.frame import JointCalibration, RobotCalibration  # noqa: E402
from soarm_sdk.conversions import RADS_PER_TICK, ticks_to_radians  # noqa: E402
from soarm_sdk.dashboard.fk import load_calibration, update_fk  # noqa: E402


class _FakeHandle:
    def __init__(self):
        self.wxyz = None
        self.position = None


def _calibration():
    return RobotCalibration(
        joints=[
            JointCalibration(name="shoulder_pan", zero_offset_ticks=2244.0,
                             direction_sign=1, tick_min=866, tick_max=3647),
            JointCalibration(name="shoulder_lift", zero_offset_ticks=2384.0,
                             direction_sign=1, tick_min=983, tick_max=3432),
            JointCalibration(name="wrist_roll", zero_offset_ticks=2079.0,
                             direction_sign=-1, tick_min=102, tick_max=3993),
        ],
        arm_id="test_arm",
        validated=True,
    )


def _capture_cfg(monkeypatch):
    """Intercept the joint config update_fk hands to the FK routine."""
    seen = {}

    def _fake_link_transforms(urdf, cfg):
        seen.update(cfg)
        return {}

    monkeypatch.setattr("soarm_sdk.dashboard.fk.link_transforms", _fake_link_transforms)
    return seen


def test_without_calibration_ticks_map_through_the_nominal_zero(monkeypatch):
    seen = _capture_cfg(monkeypatch)

    update_fk(object(), {1: 2517}, {"link": _FakeHandle()},
              joint_ids=[1], joint_names=["shoulder_pan"])

    assert seen["shoulder_pan"] == pytest.approx(ticks_to_radians(2517))


def test_calibration_shifts_the_zero_to_the_measured_one(monkeypatch):
    seen = _capture_cfg(monkeypatch)

    update_fk(object(), {1: 2517}, {"link": _FakeHandle()},
              joint_ids=[1], joint_names=["shoulder_pan"],
              calibration=_calibration())

    # 2517 sits 273 ticks past the measured zero of 2244, not 469 past 2048
    assert seen["shoulder_pan"] == pytest.approx(273 * RADS_PER_TICK, rel=1e-6)


def test_the_uncalibrated_view_is_off_by_the_zero_offset(monkeypatch):
    """The size of the error the calibration removes, pinned as a number."""
    seen_raw = _capture_cfg(monkeypatch)
    update_fk(object(), {2: 2000}, {"l": _FakeHandle()},
              joint_ids=[2], joint_names=["shoulder_lift"])
    raw = seen_raw["shoulder_lift"]

    seen_cal = _capture_cfg(monkeypatch)
    update_fk(object(), {2: 2000}, {"l": _FakeHandle()},
              joint_ids=[2], joint_names=["shoulder_lift"],
              calibration=_calibration())
    cal = seen_cal["shoulder_lift"]

    # 2384 - 2048 = 336 ticks ~= 29.5 degrees of error in the old view
    assert (raw - cal) == pytest.approx(336 * RADS_PER_TICK, rel=1e-6)


def test_a_negative_direction_sign_flips_the_joint(monkeypatch):
    seen = _capture_cfg(monkeypatch)

    update_fk(object(), {5: 2579}, {"l": _FakeHandle()},
              joint_ids=[5], joint_names=["wrist_roll"],
              calibration=_calibration())

    # 500 ticks past the zero, inverted by direction_sign = -1
    assert seen["wrist_roll"] == pytest.approx(-500 * RADS_PER_TICK, rel=1e-6)


def test_joints_absent_from_the_calibration_fall_back_to_nominal(monkeypatch):
    seen = _capture_cfg(monkeypatch)

    update_fk(object(), {4: 2100}, {"l": _FakeHandle()},
              joint_ids=[4], joint_names=["wrist_flex"],
              calibration=_calibration())  # has no wrist_flex entry

    assert seen["wrist_flex"] == pytest.approx(ticks_to_radians(2100))


def test_missing_calibration_file_returns_none_rather_than_raising(tmp_path):
    assert load_calibration(tmp_path / "nope.json") is None


def test_unreadable_calibration_file_returns_none_rather_than_raising(tmp_path):
    bad = tmp_path / "calibration.json"
    bad.write_text("{not json")

    assert load_calibration(bad) is None


def test_a_saved_calibration_round_trips_through_the_loader(tmp_path):
    path = tmp_path / "calibration.json"
    _calibration().save(path)

    loaded = load_calibration(path)

    assert loaded is not None
    assert loaded.arm_id == "test_arm"
    assert loaded.direction_signs == [1, 1, -1]


def test_the_shipped_default_urdf_is_the_so101_revision():
    # SO100 and SO101 do not share a zero convention: SO100 puts shoulder_lift
    # at [0, 3.5] and elbow_flex at [-3.1416, 0], SO101 centres both. The saved
    # calibration is SO101-framed, so the view must load the matching model.
    from soarm_sdk.cli.dashboard import _DEFAULT_URDF

    assert _DEFAULT_URDF.parent.name == "SO101"
    assert "so101" in _DEFAULT_URDF.name


def test_ghost_meshes_are_translucent_hidden_and_two_sided():
    """add_mesh_trimesh takes no opacity, so the ghost uses add_mesh_simple.

    Two-sided because a translucent shell seen from inside shows its back
    faces; culling them leaves visible holes in the arm.
    """
    import inspect

    from soarm_sdk.dashboard import fk

    src = inspect.getsource(fk.load_ghost_meshes)
    assert "add_mesh_simple" in src
    assert "opacity=opacity" in src
    assert "visible=False" in src
    assert 'side="double"' in src
    assert 0.0 < fk.GHOST_OPACITY < 1.0


def test_the_ghost_shares_the_solid_mirrors_geometry():
    """Same silhouette, or overlaying one on the other proves nothing."""
    import inspect

    from soarm_sdk.dashboard import fk

    for fn in (fk.load_urdf_meshes, fk.load_ghost_meshes):
        assert "_link_geometries(urdf)" in inspect.getsource(fn)


def test_pose_meshes_needs_no_calibration_and_no_ticks():
    """A reference pose is already a configuration in URDF radians."""
    import inspect

    sig = inspect.signature(__import__(
        "soarm_sdk.dashboard.fk", fromlist=["pose_meshes"]
    ).pose_meshes)
    assert list(sig.parameters) == ["urdf", "cfg", "handles"]


def test_posing_without_a_urdf_is_a_no_op_not_a_crash():
    from soarm_sdk.dashboard.fk import pose_meshes

    pose_meshes(None, {"a": 0.0}, {})  # must not raise


def test_dashboard_app_has_a_ghost_fk_update_distinct_from_the_live_one():
    """A manifest player and the live-poll loop used to drive the same mesh
    through the same fk_update, so animating a preview fought the live
    arm's own position for the same pixels every tick. fk_update_ghost is
    the independent twin that removes the race instead of arbitrating it."""
    import inspect

    from soarm_sdk.dashboard.app import DashboardApp

    assert hasattr(DashboardApp, "fk_update_ghost")
    src = inspect.getsource(DashboardApp.fk_update_ghost)
    assert "self._ghost_handles" in src
    assert "self._mesh_handles" not in src


def test_ghost_fk_update_is_a_noop_without_a_urdf():
    from soarm_sdk.dashboard.app import DashboardApp

    app = DashboardApp(title="no urdf", port=8097)
    app.fk_update_ghost({1: 2048})  # must not raise
