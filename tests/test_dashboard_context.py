"""Unit tests for soarm_sdk.dashboard.context.DashboardContext.

Skipped entirely if viser isn't installed — the whole soarm_sdk.dashboard
subpackage requires the `viser` extra, since importing the package's
__init__.py pulls in app.py, which requires viser.
"""

from __future__ import annotations

import pytest

viser = pytest.importorskip("viser")

from soarm_sdk.dashboard.context import DashboardContext, JointState  # noqa: E402


class _FakeGuiHandle:
    def __init__(self, value):
        self.value = value
        self.content = None


def _make_context(joint_ids=None) -> DashboardContext:
    return DashboardContext(
        device_h=_FakeGuiHandle("/dev/nonexistent"),
        baud_h=_FakeGuiHandle(1_000_000),
        interval_h=_FakeGuiHandle(200),
        conn_status_md=_FakeGuiHandle(None),
        joint_ids=joint_ids if joint_ids is not None else [1, 2, 3, 4, 5, 6],
    )


def test_joint_state_defaults():
    state = JointState()
    assert state.positions == {}
    assert state.connected is False
    assert state.poll_count == 0


def test_parse_ids_range():
    assert DashboardContext.parse_ids("1-6") == [1, 2, 3, 4, 5, 6]


def test_parse_ids_comma_and_space_separated():
    assert DashboardContext.parse_ids("1, 3, 5") == [1, 3, 5]
    assert DashboardContext.parse_ids("1 3 5") == [1, 3, 5]


def test_parse_ids_mixed_range_and_singles():
    assert DashboardContext.parse_ids("1-3,6") == [1, 2, 3, 6]


def test_parse_ids_dedupes_and_sorts():
    assert DashboardContext.parse_ids("5,1,3,1") == [1, 3, 5]


def test_context_not_polling_initially():
    ctx = _make_context()
    assert ctx.polling is False


def test_stop_polling_without_start_is_safe():
    ctx = _make_context()
    ctx.stop_polling()  # must not raise even though nothing was started
    assert ctx.state.connected is False
    assert ctx.conn_status_md.content == "*Disconnected.*"
