"""Unit tests for the characterisation rigs.

Driven against a fake arm with a *known* backlash and a known load-dependent
droop, so each rig has to recover the number that was planted. The fake
publishes telemetry continuously from a background thread, the way real
hardware does, since the rigs settle by averaging the tail of the stream.
"""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from soarm_sdk.conversions import RADS_PER_TICK
from soarm_sdk.diagnostics import (
    BacklashResult,
    DroopResult,
    measure_backlash,
    measure_droop,
)
from soarm_sdk.robot.telemetry import ServoSample, TelemetryStream


class _FakeArm:
    """A 6-joint arm with configurable lost motion and load-dependent sag."""

    def __init__(self, *, backlash_rad=0.0, droop_per_load=0.0, n=6):
        self.n = n
        self.q = np.zeros(n)
        self.backlash = backlash_rad
        self.droop_per_load = droop_per_load
        self.load = np.zeros(n)
        self.commands = []
        self._streams = []
        self._lock = threading.Lock()
        self._seq = 0
        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    # -- the interface the rigs use ------------------------------------
    def get_robot_joint_positions(self):
        with self._lock:
            return self.q.copy()

    def set_robot_joint_positions(self, q, speed=None):
        target = np.array(q, dtype=float)
        with self._lock:
            delta = target - self.q
            settled = target.copy()
            for i in range(self.n):
                # Lost motion: the joint stops short of its target, on the
                # side it approached from.
                if delta[i] > 1e-9:
                    settled[i] -= self.backlash / 2.0
                elif delta[i] < -1e-9:
                    settled[i] += self.backlash / 2.0
            # Gravity-ish load, proportional to displacement from zero, and a
            # sag proportional to that load.
            self.load = np.abs(settled) * 20.0
            settled = settled - np.sign(settled) * self.load * self.droop_per_load
            self.q = settled
            self.commands.append(target.copy())

    def subscribe(self, *, maxlen=1000):
        stream = TelemetryStream(maxlen=maxlen)
        with self._lock:
            self._streams.append(stream)
        return stream

    def unsubscribe(self, stream):
        with self._lock:
            if stream in self._streams:
                self._streams.remove(stream)
        stream.close()

    def stop(self):
        self._running = False

    # -- continuous telemetry, like the real bus thread ----------------
    def _publish_loop(self):
        while self._running:
            with self._lock:
                self._seq += 1
                sample = ServoSample(
                    t_mono=time.monotonic(),
                    seq=self._seq,
                    ids=tuple(range(1, self.n + 1)),
                    position_ticks=tuple(int(v / RADS_PER_TICK) for v in self.q),
                    position_rad=tuple(float(v) for v in self.q),
                    velocity_ticks=(0,) * self.n,
                    velocity_rad_s=(0.0,) * self.n,
                    load_percent=tuple(float(v) for v in self.load),
                    current_mA=tuple(float(v) * 6.5 for v in self.load),
                    voltage_V=(12.0,) * self.n,
                    temperature_C=(40,) * self.n,
                    status_flags=(0,) * self.n,
                    moving=(False,) * self.n,
                )
                streams = list(self._streams)
            for s in streams:
                s._publish(sample)
            time.sleep(0.002)


@pytest.fixture
def arm():
    a = _FakeArm()
    yield a
    a.stop()


# ---------------------------------------------------------------------------
# measure_backlash
# ---------------------------------------------------------------------------


def test_backlash_recovers_a_planted_hysteresis_gap():
    a = _FakeArm(backlash_rad=0.02)
    try:
        result = measure_backlash(a, 2, excursion_rad=0.2, cycles=2, settle_s=0.05)
    finally:
        a.stop()

    assert result.backlash_rad == pytest.approx(0.02, abs=1e-6)
    assert result.backlash_ticks == pytest.approx(0.02 / RADS_PER_TICK, rel=1e-6)
    assert result.significant is True


def test_backlash_of_a_perfect_joint_is_zero_and_not_significant():
    a = _FakeArm(backlash_rad=0.0)
    try:
        result = measure_backlash(a, 1, excursion_rad=0.2, cycles=2, settle_s=0.05)
    finally:
        a.stop()

    assert result.backlash_rad == pytest.approx(0.0, abs=1e-9)
    assert result.significant is False


def test_backlash_commands_the_same_target_from_both_directions(arm):
    measure_backlash(arm, 0, excursion_rad=0.1, cycles=1, settle_s=0.05)

    j0 = [float(c[0]) for c in arm.commands]
    # away below, back to target, away above, back to target
    assert j0[0] == pytest.approx(-0.1)
    assert j0[1] == pytest.approx(0.0)
    assert j0[2] == pytest.approx(+0.1)
    assert j0[3] == pytest.approx(0.0)


def test_backlash_leaves_other_joints_alone(arm):
    measure_backlash(arm, 3, excursion_rad=0.1, cycles=1, settle_s=0.05)

    for cmd in arm.commands:
        assert list(np.delete(cmd, 3)) == pytest.approx([0.0] * 5)


def test_backlash_result_with_no_data_reports_nan_not_a_crash():
    empty = BacklashResult(joint_index=0, target_rad=0.0)
    assert empty.backlash_rad != empty.backlash_rad  # NaN
    assert empty.significant is False
    assert "no data" in empty.summary()


# ---------------------------------------------------------------------------
# measure_droop
# ---------------------------------------------------------------------------


def test_droop_records_one_point_per_offset_and_returns_to_centre(arm):
    result = measure_droop(arm, 1, offsets_rad=(-0.2, 0.0, 0.2), settle_s=0.05)

    assert len(result.commanded_rad) == 3
    assert len(result.settled_rad) == 3
    assert len(result.load_percent) == 3
    assert float(arm.commands[-1][1]) == pytest.approx(0.0)  # back to centre


def test_droop_correlates_steady_state_error_with_load():
    a = _FakeArm(droop_per_load=0.001)
    try:
        result = measure_droop(a, 1, offsets_rad=(0.0, 0.1, 0.2, 0.3), settle_s=0.05)
    finally:
        a.stop()

    assert result.worst_error_rad > 0.0
    # more load -> more error, which is the whole point of the measurement
    assert result.load_error_correlation() > 0.9


def test_droop_correlation_is_nan_when_load_never_varies(arm):
    result = measure_droop(arm, 1, offsets_rad=(0.0, 0.0), settle_s=0.05)

    r = result.load_error_correlation()
    assert r != r  # NaN: nothing to correlate


def test_droop_error_is_commanded_minus_settled():
    result = DroopResult(
        joint_index=0,
        commanded_rad=[1.0, 2.0],
        settled_rad=[0.9, 1.7],
    )
    assert result.error_rad == pytest.approx([0.1, 0.3])
    assert result.worst_error_rad == pytest.approx(0.3)
