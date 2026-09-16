"""Unit tests for soarm_sdk.monitoring.blueprint.

Structural only: rerun's own Grid/Tabs/TimeSeriesView/Blueprint objects
don't expose a stable public way to introspect their contents from Python
(they serialize to Rerun's wire format), so these tests check what our
code controls directly — argument construction, names, counts — not what
rerun does internally with them. Skipped entirely when rerun-sdk (the
``telemetry`` extra) isn't installed.
"""

from __future__ import annotations

import pytest

rrb = pytest.importorskip("rerun.blueprint")

from soarm_sdk.monitoring.blueprint import (  # noqa: E402
    DEFAULT_FIELDS,
    build_by_channel_blueprint,
    build_by_servo_blueprint,
    build_monitor_blueprint,
)

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def test_default_fields_excludes_goal_and_tracking_error():
    field_names = [f for f, _ in DEFAULT_FIELDS]
    assert "goal_position_rad" not in field_names
    assert "tracking_error_rad" not in field_names
    assert "position_rad" in field_names


def test_by_servo_blueprint_builds_without_error():
    grid = build_by_servo_blueprint(JOINT_NAMES)
    assert isinstance(grid, rrb.Grid)


def test_by_servo_blueprint_respects_default_field(monkeypatch):
    seen_contents = []
    orig_init = rrb.TimeSeriesView.__init__

    def _spy(self, *, origin, contents, name=None, **kwargs):
        seen_contents.append(contents)
        return orig_init(self, origin=origin, contents=contents, name=name, **kwargs)

    monkeypatch.setattr(rrb.TimeSeriesView, "__init__", _spy)
    build_by_servo_blueprint(JOINT_NAMES, default_field="current_mA")
    assert len(seen_contents) == len(JOINT_NAMES)
    assert all(c == "$origin/current_mA" for c in seen_contents)


def test_by_channel_blueprint_builds_without_error():
    tabs = build_by_channel_blueprint(JOINT_NAMES)
    assert isinstance(tabs, rrb.Tabs)


def test_by_channel_blueprint_one_panel_per_field(monkeypatch):
    seen_origins = []
    orig_init = rrb.TimeSeriesView.__init__

    def _spy(self, *, origin, contents, name=None, **kwargs):
        seen_origins.append(origin)
        return orig_init(self, origin=origin, contents=contents, name=name, **kwargs)

    monkeypatch.setattr(rrb.TimeSeriesView, "__init__", _spy)
    build_by_channel_blueprint(JOINT_NAMES)
    # One TimeSeriesView per (joint, field) pair.
    assert len(seen_origins) == len(JOINT_NAMES) * len(DEFAULT_FIELDS)


def test_monitor_blueprint_builds_without_error():
    blueprint = build_monitor_blueprint(JOINT_NAMES)
    assert isinstance(blueprint, rrb.Blueprint)


def test_monitor_blueprint_works_with_a_single_joint():
    # A caller monitoring one joint (e.g. --joint-ids 6) shouldn't crash
    # either preset.
    blueprint = build_monitor_blueprint(["gripper"])
    assert isinstance(blueprint, rrb.Blueprint)


def test_custom_prefix_is_used_in_entity_paths(monkeypatch):
    seen_origins = []
    orig_init = rrb.TimeSeriesView.__init__

    def _spy(self, *, origin, contents, name=None, **kwargs):
        seen_origins.append(origin)
        return orig_init(self, origin=origin, contents=contents, name=name, **kwargs)

    monkeypatch.setattr(rrb.TimeSeriesView, "__init__", _spy)
    build_by_servo_blueprint(["gripper"], prefix="custom_prefix")
    assert seen_origins == ["custom_prefix/gripper"]
