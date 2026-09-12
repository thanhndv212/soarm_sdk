"""Tests for torque control on ServoHardwareInterface.

Torque writes share the RS-485 bus with the background state reader, so they
are queued and drained on that thread rather than sent from the caller's.
These exercise the queue/drain contract against a fake servo double — the
real bus is out of scope here, as elsewhere in this suite.
"""

from __future__ import annotations

import numpy as np
import pytest

from soarm_sdk.robot.hardware import ServoHardwareInterface


class _FakeSyncWrite:
    def __init__(self):
        self.cleared = 0
        self.sent = 0

    def clearParam(self):  # noqa: N802 — mirrors the protocol class
        self.cleared += 1

    def txPacket(self):  # noqa: N802
        self.sent += 1


class _FakeServo:
    def __init__(self):
        self.byte_writes: list[tuple[int, int, int]] = []
        self.goals: list[tuple[int, int]] = []
        self.groupSyncWrite = _FakeSyncWrite()  # noqa: N815

    def write1ByteTxRx(self, sid, addr, value):  # noqa: N802
        self.byte_writes.append((sid, addr, value))
        return 0, 0

    def SyncWritePosEx(self, sid, ticks, speed, acc):  # noqa: N802
        self.goals.append((sid, ticks))


def _iface(**kw):
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1, 2], **kw)
    hw._srv = _FakeServo()
    return hw


def _torque_values(srv):
    from soarm_sdk.protocol.registers import STS_TORQUE_ENABLE

    return [v for _, addr, v in srv.byte_writes if addr == STS_TORQUE_ENABLE]


def test_disable_queues_rather_than_writing_from_the_caller_thread():
    """The bus thread owns the port; a caller-thread write would interleave
    with its sync-read packets."""
    hw = _iface()
    hw._pending_torque = False  # what set_torque() queues

    assert _torque_values(hw._srv) == []
    hw._apply_pending_torque()
    assert _torque_values(hw._srv) == [0, 0]


def test_drain_reports_completion_so_a_blocking_caller_returns():
    hw = _iface()
    hw._pending_torque = False
    hw._apply_pending_torque()
    assert hw._pending_torque is None
    assert hw.torque_enabled is False


def test_reenabling_parks_the_goal_at_the_measured_pose():
    """Otherwise the servo drives to whatever it was chasing when torque was
    cut — a lurch, once the arm has been moved by hand in between."""
    hw = _iface()
    hw._torque_enabled = False
    hw._cached_positions_ticks = [1500, 2600]

    hw._pending_torque = True
    hw._apply_pending_torque()

    assert hw._srv.goals == [(1, 1500), (2, 2600)]
    assert hw._srv.groupSyncWrite.sent == 1
    assert _torque_values(hw._srv) == [1, 1]


def test_disabling_does_not_write_goals():
    hw = _iface()
    hw._pending_torque = False
    hw._apply_pending_torque()
    assert hw._srv.goals == []


def test_a_queued_move_does_not_survive_going_limp():
    hw = _iface()
    hw.set_robot_joint_positions(np.array([0.1, 0.1]))
    assert hw._pending_command is not None

    with pytest.raises(RuntimeError):
        hw.set_torque(False, timeout_s=0.05)  # nothing is draining it

    assert hw._pending_command is None


def test_a_limp_arm_is_never_driven_by_a_stale_command():
    hw = _iface()
    hw._torque_enabled = False
    hw.set_robot_joint_positions(np.array([0.1, 0.1]))

    hw._write_if_pending()

    assert hw._srv.goals == []


def test_set_torque_raises_when_nothing_drains_the_request():
    """Blocking is the point: a caller about to say 'safe to move by hand'
    must not be told that on the strength of a queued request."""
    hw = _iface()
    with pytest.raises(RuntimeError, match="not applied within"):
        hw.set_torque(False, timeout_s=0.05)


def test_set_torque_returns_once_the_bus_thread_has_applied_it():
    hw = _iface()
    import threading

    threading.Timer(0.02, hw._apply_pending_torque).start()
    hw.set_torque(False, timeout_s=2.0)  # must not raise
    assert hw.torque_enabled is False


def test_torque_starts_on_by_default_and_can_start_off():
    assert _iface().torque_enabled is True
    assert _iface(torque_on_start=False).torque_enabled is False
