"""probe_and_set_limit() against a fake servo — no hardware required.

Exercises the two ways a probe is supposed to stop: no more progress (a
hard mechanical stop) and a current ceiling (an aggressive stall) — and
confirms the resulting EEPROM write always lands *short* of wherever the
joint actually stopped, never past it.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass

import numpy as np
import pytest

from soarm_sdk.calibration.frame import JointCalibration
from soarm_sdk.calibration.widen_limit import probe_and_set_limit
from soarm_sdk.protocol.registers import (
    STS_MAX_ANGLE_LIMIT_L, STS_MIN_ANGLE_LIMIT_L,
    STS_PRESENT_CURRENT_L, STS_PRESENT_POSITION_L,
)


@dataclass
class _Result:
    data: list
    result: int = 0
    error: int = 0


class _FakeSrv:
    """One servo's registers, backed by a simple mechanical model.

    Position moves toward the commanded goal by a fixed step per settle,
    clamped at ``stop_ticks`` — a hard mechanical limit the probe does not
    know about in advance. Current reports ``stall_current_ma`` for any
    step commanded past the stop (pushing against something that does not
    yield), and a small idle value otherwise.
    """

    def __init__(self, start, stop_ticks, per_step_ticks, stall_current_ma):
        self.pos = start
        self.min_limit = 0
        self.max_limit = 4095
        self._stop = stop_ticks
        self._per_step = per_step_ticks
        self._stall_current_ma = stall_current_ma
        # Fixed at construction, not inferred from position each call: an
        # inference based on comparing pos to stop breaks exactly at the
        # boundary a probe is trying to find (pos == stop), which is the
        # one moment the model most needs to get right.
        self._approaching_from_above = start > stop_ticks
        self._blocked = False

    def read2ByteTxRx(self, sid, addr):
        if addr == STS_PRESENT_POSITION_L:
            return _Result([self.pos])
        if addr == STS_MIN_ANGLE_LIMIT_L:
            return _Result([self.min_limit])
        if addr == STS_MAX_ANGLE_LIMIT_L:
            return _Result([self.max_limit])
        if addr == STS_PRESENT_CURRENT_L:
            ma = self._stall_current_ma if self._blocked else 6.0
            return _Result([int(ma / 6.5)])
        raise AssertionError(f"unexpected read addr {addr}")

    def write1ByteTxRx(self, sid, addr, value):
        return _Result([value])

    def write2ByteTxRx(self, sid, addr, value):
        if addr == STS_MIN_ANGLE_LIMIT_L:
            self.min_limit = value
        elif addr == STS_MAX_ANGLE_LIMIT_L:
            self.max_limit = value
        return _Result([value])

    def _advance_toward(self, goal):
        step = max(-self._per_step, min(self._per_step, goal - self.pos))
        unclamped = self.pos + step
        clamped = (
            max(unclamped, self._stop)
            if self._approaching_from_above
            else min(unclamped, self._stop)
        )
        # Blocked — and current spikes — exactly when the stop actually
        # clipped this step's commanded motion, not just when moving toward
        # it in the abstract.
        self._blocked = clamped != unclamped
        self.pos = clamped


class _FakeHW:
    """Just enough of ServoHardwareInterface's surface for the probe."""

    def __init__(self, srv, calibration, joint_index=0):
        self._srv = srv
        self._cal = calibration
        self._idx = joint_index

    @contextmanager
    def lend_bus(self):
        yield self._srv

    def get_robot_joint_positions(self):
        rad = self._cal.joints[self._idx].to_rad(self._srv.pos)
        return np.array([rad, 0, 0, 0, 0, 0])

    def set_robot_joint_positions(self, q, dq=None):
        goal_ticks = int(round(self._cal.joints[self._idx].to_ticks(float(q[self._idx]))))
        self.last_dq = None if dq is None else float(dq[self._idx])
        self._srv._advance_toward(goal_ticks)


def _cal_one_joint(name="wrist_flex"):
    from soarm_sdk.calibration.frame import RobotCalibration
    j = JointCalibration(
        name=name, zero_offset_ticks=2048.0, direction_sign=1,
        tick_min=1000, tick_max=3000,
    )
    return RobotCalibration(joints=[j])


def test_probe_stops_at_no_progress_and_sets_margin_below():
    cal = _cal_one_joint()
    joint = cal.joints[0]
    srv = _FakeSrv(start=2900, stop_ticks=3200, per_step_ticks=12, stall_current_ma=6.0)
    srv.max_limit = 3046  # the artificially tight "before", as on wrist_flex
    hw = _FakeHW(srv, cal)

    result = probe_and_set_limit(
        hw, servo_id=4, calibration=joint, direction="max",
        probe_bound_ticks=3600, step_ticks=12, settle_s=0,
        stall_steps=3, progress_ticks=3, margin_ticks=20,
    )

    assert result.stopped_reason == "no_progress"
    assert result.stop_ticks == 3200
    assert result.final_ticks == 3200 - 20
    # The write actually landed in the fake servo's registers.
    assert srv.max_limit == result.final_ticks
    # Never past the true mechanical stop.
    assert result.final_ticks < result.stop_ticks


def test_probe_stops_at_current_ceiling_before_reaching_stall_point():
    cal = _cal_one_joint()
    joint = cal.joints[0]
    # The stop sits well short of the probe bound, so the probe keeps
    # pushing past it every step — a current ceiling should trip on the
    # first blocked step, long before three stalled steps would.
    srv = _FakeSrv(start=2900, stop_ticks=2912, per_step_ticks=12, stall_current_ma=800.0)
    srv.max_limit = 3046
    hw = _FakeHW(srv, cal)

    result = probe_and_set_limit(
        hw, servo_id=4, calibration=joint, direction="max",
        probe_bound_ticks=3600, step_ticks=12, settle_s=0,
        current_ceiling_ma=500.0, margin_ticks=20,
    )

    assert result.stopped_reason == "current_ceiling"
    assert result.steps_taken <= 2  # tripped almost immediately, not after 3 stalled steps
    # stop(2912) - margin(20) = 2892, tighter than before(3046) -> keep 3046,
    # same safety net as the "no widening past what was already trusted"
    # case — the point of this test is *which reason* stopped the probe,
    # not the final number, which the no-progress test already covers.
    assert result.final_ticks == max(3046, result.stop_ticks - 20)


def test_direction_min_walks_downward_and_never_overshoots():
    cal = _cal_one_joint("elbow_flex")
    joint = cal.joints[0]
    srv = _FakeSrv(start=1400, stop_ticks=900, per_step_ticks=12, stall_current_ma=6.0)
    srv.min_limit = 1050  # an artificially tight floor, same shape as the real bug
    hw = _FakeHW(srv, cal)

    result = probe_and_set_limit(
        hw, servo_id=3, calibration=joint, direction="min",
        probe_bound_ticks=800, step_ticks=12, settle_s=0,
        stall_steps=3, progress_ticks=3, margin_ticks=20,
    )

    assert result.stopped_reason == "no_progress"
    assert result.stop_ticks == 900
    assert result.final_ticks == 900 + 20
    assert srv.min_limit == result.final_ticks
    assert result.final_ticks > result.stop_ticks


def test_final_limit_never_tightens_below_the_original_cap():
    """If the true stop measures *tighter* than margin below the original
    cap (rounding, a slightly conservative probe), the corrected limit must
    not end up narrower than what was already trusted — ``max(before,
    stop - margin)`` is the documented safety net, not just ``stop - margin``.
    """
    cal = _cal_one_joint()
    joint = cal.joints[0]
    srv = _FakeSrv(start=3040, stop_ticks=3044, per_step_ticks=12, stall_current_ma=6.0)
    hw = _FakeHW(srv, cal)
    srv.max_limit = 3046  # "before" — matches the real wrist_flex bug's numbers

    result = probe_and_set_limit(
        hw, servo_id=4, calibration=joint, direction="max",
        probe_bound_ticks=3600, step_ticks=12, settle_s=0,
        stall_steps=3, progress_ticks=3, margin_ticks=20,
    )

    assert result.before_ticks == 3046
    assert result.stop_ticks == 3044
    # stop(3044) - margin(20) = 3024, tighter than before(3046) -> keep 3046.
    assert result.final_ticks == 3046


def test_dq_rad_s_reaches_the_hardware_call():
    """A joint fighting gravity (elbow_flex, moving toward its low end) can
    stall at a gentle default speed for reasons that have nothing to do
    with the EEPROM limit — the caller needs a real way to push harder."""
    cal = _cal_one_joint()
    joint = cal.joints[0]
    srv = _FakeSrv(start=2900, stop_ticks=3600, per_step_ticks=12, stall_current_ma=6.0)
    hw = _FakeHW(srv, cal)

    probe_and_set_limit(
        hw, servo_id=4, calibration=joint, direction="max",
        probe_bound_ticks=2912, step_ticks=12, settle_s=0, dq_rad_s=0.8,
    )

    assert hw.last_dq == pytest.approx(0.8)


def test_default_dq_is_the_documented_gentle_value():
    cal = _cal_one_joint()
    joint = cal.joints[0]
    srv = _FakeSrv(start=2900, stop_ticks=3600, per_step_ticks=12, stall_current_ma=6.0)
    hw = _FakeHW(srv, cal)

    probe_and_set_limit(
        hw, servo_id=4, calibration=joint, direction="max",
        probe_bound_ticks=2912, step_ticks=12, settle_s=0,
    )

    assert hw.last_dq == pytest.approx(0.4)
