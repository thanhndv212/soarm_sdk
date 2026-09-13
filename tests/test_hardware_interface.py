"""Unit tests for soarm_sdk.robot.hardware.ServoHardwareInterface.

Covers pure logic reachable without an open serial port: state caching,
tick<->radian conversion round trips, and command-buffer constructon.
start()/stop() and the background read/write loop need real hardware and
are out of scope for unit tests.
"""

from __future__ import annotations

import json
import threading

import numpy as np
import pytest

from soarm_sdk.protocol.group_sync_read import GroupSyncRead
from soarm_sdk.protocol.packet_handler import PacketResult
from soarm_sdk.protocol.registers import (
    COMM_SUCCESS,
    STS_TELEMETRY_LENGTH,
    STS_TELEMETRY_START,
    STS_TORQUE_ENABLE,
)
from soarm_sdk.protocol.sts import sts as _Sts
from soarm_sdk.robot.hardware import ServoHardwareInterface


def test_state_age_is_infinite_before_first_read():
    hw = ServoHardwareInterface(port="/dev/nonexistent")
    assert hw.state_age() == float("inf")


def test_get_robot_joint_positions_reads_from_seeded_cache():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1, 2])
    hw._cached_positions_ticks = [2048, 2048]  # TICK_ZERO for both joints

    positions = hw.get_robot_joint_positions()

    assert positions == pytest.approx([0.0, 0.0], abs=1e-6)


def test_get_robot_joint_state_converts_ticks_to_physical_units():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    hw._cached_positions_ticks = [2048]
    hw._cached_speeds_ticks = [100]
    hw._cached_currents_mA = [250.0]
    hw._last_read_time = 42.0

    state = hw.get_robot_joint_state()

    assert state.positions == pytest.approx([0.0], abs=1e-6)
    assert state.velocities[0] != 0.0  # 100 ticks/s converts to a nonzero rad/s
    assert state.efforts == pytest.approx([250.0])
    assert state.timestamp == pytest.approx(42.0)


def test_set_robot_joint_positions_queues_a_command():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1, 2])

    hw.set_robot_joint_positions(np.array([0.0, 0.0]))

    assert hw._pending_command is not None
    assert hw._pending_command.ticks_list == [2048, 2048]
    assert hw._pending_command.speed_ticks_list == [hw._default_speed] * 2


def test_set_robot_joint_positions_speed_override_takes_priority_over_dq():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])

    hw.set_robot_joint_positions(np.array([0.0]), dq=np.array([5.0]), speed=42)

    assert hw._pending_command.speed_ticks_list == [42]


def test_get_body_pose_without_fk_fn_raises():
    hw = ServoHardwareInterface(port="/dev/nonexistent")
    with pytest.raises(NotImplementedError):
        hw.get_body_pose("ee")


def test_get_body_pose_delegates_to_fk_fn():
    expected = (np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    hw = ServoHardwareInterface(port="/dev/nonexistent", fk_fn=lambda q: expected)

    pos, quat = hw.get_body_pose("ee")

    assert np.array_equal(pos, expected[0])
    assert np.array_equal(quat, expected[1])


def test_write_if_pending_is_a_noop_with_no_pending_command():
    hw = ServoHardwareInterface(port="/dev/nonexistent")
    hw._write_if_pending()  # must not raise even though _srv is None


def test_read_once_is_a_noop_before_start():
    hw = ServoHardwareInterface(port="/dev/nonexistent")
    hw._read_once()  # _gsr/_srv are None pre-start(); must not raise
    assert hw.read_errors == 0


# ===========================================================================
# Safety layer: joint limits and step clamping
#
# These exercise _apply_safety directly rather than set_robot_joint_positions,
# because the latter queues a command for a bus thread that is not running
# without hardware. The clamping is what needs testing; the queueing is not.
# ===========================================================================


def _iface(**kw) -> ServoHardwareInterface:
    """Interface over a nonexistent port with a seeded position cache.

    2048 ticks is 0 rad under the default zero offsets, so "current
    position" is the origin unless a test says otherwise.
    """
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1, 2], **kw)
    hw._cached_positions_ticks = [2048, 2048]
    return hw


def test_no_limits_configured_lets_anything_through():
    """Defaults must stay off, or adding safety would break every caller."""
    hw = _iface()
    out = hw._apply_safety(np.array([5.0, -5.0]))
    assert out == pytest.approx([5.0, -5.0])
    assert hw.limit_clamps == 0 and hw.step_clamps == 0


def test_joint_limits_are_enforced_on_write():
    hw = _iface(joint_limits=([-1.0, -1.0], [1.0, 1.0]))
    out = hw._apply_safety(np.array([2.5, -0.5]))
    assert out == pytest.approx([1.0, -0.5])
    assert hw.limit_clamps == 1


def test_limit_clamps_accumulate_across_calls():
    hw = _iface(joint_limits=([-1.0, -1.0], [1.0, 1.0]))
    hw._apply_safety(np.array([2.0, 2.0]))
    hw._apply_safety(np.array([2.0, 0.0]))
    assert hw.limit_clamps == 3


def test_in_range_target_is_not_counted_as_a_clamp():
    hw = _iface(joint_limits=([-1.0, -1.0], [1.0, 1.0]))
    hw._apply_safety(np.array([1.0, -1.0]))  # exactly on the limits
    assert hw.limit_clamps == 0


def test_step_clamp_limits_distance_from_the_measured_position():
    hw = _iface(max_step_rad=0.05)
    out = hw._apply_safety(np.array([1.0, -1.0]))
    assert out == pytest.approx([0.05, -0.05])
    assert hw.step_clamps == 2


def test_step_clamp_measures_against_actual_not_previous_command():
    """A lagging joint must not accumulate an ever-larger commanded jump.

    Commanding repeatedly while the arm does not move should keep producing
    the same bounded target, not walk away from the measured position.
    """
    hw = _iface(max_step_rad=0.05)
    first = hw._apply_safety(np.array([1.0, 1.0]))
    second = hw._apply_safety(np.array([1.0, 1.0]))
    assert first == pytest.approx(second)


def test_small_step_passes_untouched():
    hw = _iface(max_step_rad=0.05)
    out = hw._apply_safety(np.array([0.01, -0.02]))
    assert out == pytest.approx([0.01, -0.02])
    assert hw.step_clamps == 0


def test_limits_apply_before_the_step_clamp():
    """Order matters: clamping to a reachable pose first, then bounding the
    step toward it, is what makes an out-of-range target approach the limit
    rather than stall at the current position."""
    hw = _iface(joint_limits=([-0.2, -0.2], [0.2, 0.2]), max_step_rad=0.05)
    out = hw._apply_safety(np.array([10.0, 10.0]))
    assert out == pytest.approx([0.05, 0.05])
    assert hw.limit_clamps == 2 and hw.step_clamps == 2


def test_bad_safety_configuration_is_rejected_at_construction():
    with pytest.raises(ValueError, match="max_step_rad"):
        _iface(max_step_rad=0.0)
    with pytest.raises(ValueError, match="lower"):
        _iface(joint_limits=([1.0, 1.0], [-1.0, -1.0]))
    with pytest.raises(ValueError, match="entries per side"):
        _iface(joint_limits=([-1.0], [1.0]))


def test_wrong_length_target_is_rejected():
    hw = _iface()
    with pytest.raises(ValueError, match="joint values"):
        hw._apply_safety(np.array([0.0, 0.0, 0.0]))


def test_calibration_supplies_offsets_and_signs_together():
    """Passing a calibration must override both, so the two cannot be set
    from different sources and disagree."""
    from soarm_sdk.calibration.frame import seed_from_travel

    cal = seed_from_travel(
        names=["a", "b"],
        urdf_limits=[(-1.0, 1.0), (-1.0, 1.0)],
        tick_ranges=[(1000, 3000), (500, 2500)],
    )
    hw = ServoHardwareInterface(
        port="/dev/nonexistent", joint_ids=[1, 2], calibration=cal
    )
    assert hw._zero_offsets == pytest.approx([2000.0, 1500.0])
    assert hw._direction_signs == [1, 1]


def test_calibration_joint_count_must_match():
    from soarm_sdk.calibration.frame import seed_from_travel

    cal = seed_from_travel(["a"], [(-1.0, 1.0)], [(1000, 3000)])
    with pytest.raises(ValueError, match="calibration has 1 joints"):
        ServoHardwareInterface(
            port="/dev/nonexistent", joint_ids=[1, 2], calibration=cal
        )


# ---------------------------------------------------------------------------
# Telemetry block decode (_read_once)
#
# The sync read covers the whole read-only SRAM block, addresses 56-70, in one
# transaction. These tests drive the real GroupSyncRead.getData offsets and the
# real sts sign decode against a canned block, with txRxPacket stubbed so no
# serial port is touched.
# ---------------------------------------------------------------------------


def _telemetry_block(
    *, pos=2048, speed=0, load=0, voltage=120, temperature=40, status=0,
    moving=0, current=0,
):
    """Bytes as a servo returns them for addresses 56..70 inclusive."""

    def lo_hi(value):
        return [value & 0xFF, (value >> 8) & 0xFF]

    block = [0] * STS_TELEMETRY_LENGTH
    block[0:2] = lo_hi(pos)  # 56-57 position
    block[2:4] = lo_hi(speed)  # 58-59 speed
    block[4:6] = lo_hi(load)  # 60-61 load
    block[6] = voltage  # 62
    block[7] = temperature  # 63
    # 64 is a gap
    block[9] = status  # 65
    block[10] = moving  # 66
    # 67, 68 are gaps
    block[13:15] = lo_hi(current)  # 69-70 current
    return block


def _wire_up(hw, blocks):
    """Attach a real GroupSyncRead pre-loaded with *blocks* (sid -> block)."""
    srv = _Sts(None)
    gsr = GroupSyncRead(srv, STS_TELEMETRY_START, STS_TELEMETRY_LENGTH)
    for sid, block in blocks.items():
        gsr.addParam(sid)
        # _readRx yields [error_byte, *data] on success.
        gsr.data_dict[sid] = [0] + list(block)
    gsr.txRxPacket = lambda: COMM_SUCCESS
    hw._srv = srv
    hw._gsr = gsr
    return hw


def test_read_once_decodes_every_field_of_the_block():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1, 2])
    _wire_up(
        hw,
        {
            1: _telemetry_block(
                pos=1024, speed=50, load=250, voltage=121,
                temperature=38, status=0, moving=1, current=100,
            ),
            2: _telemetry_block(
                pos=3072, speed=0, load=0, voltage=120,
                temperature=41, status=0x20, moving=0, current=0,
            ),
        },
    )

    hw._read_once()

    assert hw.read_errors == 0
    assert hw._cached_positions_ticks == [1024, 3072]
    assert hw._cached_speeds_ticks == [50, 0]
    assert hw._cached_loads_pct == pytest.approx([25.0, 0.0])
    assert hw._cached_currents_mA == pytest.approx([650.0, 0.0])
    assert hw._cached_voltages_V == pytest.approx([12.1, 12.0])
    assert hw._cached_temperatures_C == [38, 41]
    assert hw._cached_status_flags == [0, 0x20]
    assert hw._cached_moving == [True, False]
    assert hw._last_read_time > 0.0


def test_read_once_decodes_a_negative_load():
    # Load signs at bit 10, not bit 15 — a whole-word sign decode would read
    # this as +1524 rather than -50.0%.
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block(load=(1 << 10) | 500)})

    hw._read_once()

    assert hw._cached_loads_pct == pytest.approx([-50.0])


def test_read_once_decodes_a_negative_speed():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block(speed=(1 << 15) | 200)})

    hw._read_once()

    assert hw._cached_speeds_ticks == [-200]


def test_read_once_rejects_a_truncated_block_without_touching_the_cache():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block(pos=1234)})
    # A short response: enough bytes for position, not for current at 69-70.
    hw._gsr.data_dict[1] = [0] + _telemetry_block(pos=1234)[:6]

    hw._read_once()

    assert hw.read_errors == 1
    assert hw._cached_positions_ticks == [2048]  # untouched seed value
    assert hw._last_read_time == 0.0


def test_efforts_are_populated_from_a_real_read():
    # Regression: _cached_currents_mA was allocated but never written, so
    # JointState.efforts was silently always zeros.
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block(current=200)})

    hw._read_once()
    state = hw.get_robot_joint_state()

    assert state.efforts == pytest.approx([1300.0])


def test_get_servo_health_snapshots_the_block_with_one_timestamp():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1, 2])
    _wire_up(
        hw,
        {
            1: _telemetry_block(load=100, current=50, voltage=118, temperature=39),
            2: _telemetry_block(load=200, current=60, voltage=119, temperature=40),
        },
    )

    hw._read_once()
    health = hw.get_servo_health()

    assert health.loads_percent == pytest.approx([10.0, 20.0])
    assert health.currents_mA == pytest.approx([325.0, 390.0])
    assert health.voltages_V == pytest.approx([11.8, 11.9])
    assert health.temperatures_C == pytest.approx([39.0, 40.0])
    assert health.moving.tolist() == [False, False]
    assert health.timestamp == hw.get_robot_joint_state().timestamp


# ---------------------------------------------------------------------------
# Telemetry tap wired into the bus thread
# ---------------------------------------------------------------------------


def _stub_writes(hw):
    """Neutralise the sync-write packet so only the recording path runs."""

    class _FakeSyncWrite:
        def __init__(self):
            self.sent = []

        def clearParam(self):  # noqa: N802
            self.sent.clear()

        def addParam(self, servo_id, payload):  # noqa: N802
            self.sent.append((servo_id, list(payload)))
            return True

        def txPacket(self):  # noqa: N802
            return COMM_SUCCESS

    hw._srv.groupSyncWrite = _FakeSyncWrite()
    hw.register_writes = []
    hw._srv.write1ByteTxRx = lambda sid, addr, val: (
        hw.register_writes.append((sid, addr, val)),
        PacketResult([0], COMM_SUCCESS),
    )[1]
    return hw


def test_no_subscribers_means_nothing_is_built():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block(pos=1500)})

    hw._read_once()

    assert hw.subscriber_count == 0
    assert hw._cached_positions_ticks == [1500]  # the read still happened


def test_subscribe_receives_one_sample_per_successful_read():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block(pos=1500, load=100, current=40)})
    stream = hw.subscribe()

    hw._read_once()
    hw._read_once()

    samples = stream.drain()
    assert len(samples) == 2
    assert samples[0].position_ticks == (1500,)
    assert samples[0].load_percent == pytest.approx((10.0,))
    assert samples[0].current_mA == pytest.approx((260.0,))
    assert samples[0].t_mono > 0.0


def test_seq_gaps_mark_a_failed_read():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block()})
    stream = hw.subscribe()

    hw._read_once()  # seq 1, published
    hw._gsr.txRxPacket = lambda: -6  # COMM_RX_TIMEOUT: seq 2, dropped
    hw._read_once()
    hw._gsr.txRxPacket = lambda: COMM_SUCCESS
    hw._read_once()  # seq 3, published

    seqs = [s.seq for s in stream.drain()]
    assert seqs == [1, 3]
    assert hw.read_errors == 1


def test_sample_carries_the_command_that_was_in_force():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block(pos=2048)})
    stream = hw.subscribe()

    hw._read_once()
    assert stream.drain()[0].goal_position_ticks is None  # nothing sent yet

    _stub_writes(hw)
    hw._torque_enabled = True
    hw.set_robot_joint_positions(np.array([0.5]))
    hw._write_if_pending()
    hw._read_once()

    sample = stream.drain()[0]
    assert sample.goal_position_ticks is not None
    assert sample.goal_speed_ticks == (hw._default_speed,)
    assert sample.tracking_error_rad() == pytest.approx((0.5,), abs=1e-3)


def test_unsubscribe_stops_delivery_and_closes_the_stream():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block()})
    stream = hw.subscribe()

    hw.unsubscribe(stream)
    hw._read_once()

    assert hw.subscriber_count == 0
    assert stream.closed is True
    assert stream.drain() == []


def test_two_subscribers_each_get_every_sample():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block()})
    a, b = hw.subscribe(), hw.subscribe()

    hw._read_once()

    assert len(a.drain()) == 1
    assert len(b.drain()) == 1


def test_a_raising_consumer_cannot_break_the_producer():
    # The bus thread also writes to the servos, so a broken subscriber must
    # not stop the loop or starve the other subscribers.
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block()})
    bad = hw.subscribe()
    good = hw.subscribe()

    def _explode(_sample):
        raise RuntimeError("consumer is broken")

    bad._publish = _explode  # type: ignore[method-assign]

    hw._read_once()  # must not raise
    hw._read_once()

    assert len(good.drain()) == 2


def test_end_to_end_bus_tick_to_jsonl_file(tmp_path):
    # The whole chain with only the wire faked: real interface, real sample
    # construction, real stream, real recorder thread, real file sink.
    from soarm_sdk.robot.telemetry_sinks import JsonlSink, TelemetryRecorder

    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1, 2])
    _wire_up(
        hw,
        {
            1: _telemetry_block(pos=2048, load=150, current=80, temperature=39),
            2: _telemetry_block(pos=1024, load=(1 << 10) | 300, current=40),
        },
    )
    path = tmp_path / "telemetry.jsonl"

    with TelemetryRecorder(hw, [JsonlSink(path, flush_every=1)], poll_timeout=0.05):
        for _ in range(3):
            hw._read_once()
        deadline = threading.Event()
        for _ in range(200):
            if path.exists() and len(path.read_text().splitlines()) >= 3:
                break
            deadline.wait(0.01)

    rows = [json.loads(line) for line in path.read_text().strip().splitlines()]
    assert len(rows) == 3
    assert rows[0]["ids"] == [1, 2]
    assert rows[0]["position_ticks"] == [2048, 1024]
    assert rows[0]["load_percent"] == pytest.approx([15.0, -30.0])
    assert rows[0]["current_mA"] == pytest.approx([520.0, 260.0])
    assert [r["seq"] for r in rows] == [1, 2, 3]
    assert hw.subscriber_count == 0  # recorder unsubscribed on exit


# ---------------------------------------------------------------------------
# Parking goals before torque
# ---------------------------------------------------------------------------


def test_park_goals_writes_the_measured_pose_not_the_zero_pose():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1, 2])
    _wire_up(hw, {1: _telemetry_block(pos=1500), 2: _telemetry_block(pos=3000)})
    _stub_writes(hw)
    hw._read_once()

    hw._park_goals_at_measured()

    # SyncWritePosEx payload is [acc, pos_lo, pos_hi, 0, 0, spd_lo, spd_hi]
    sent = {sid: payload for sid, payload in hw._srv.groupSyncWrite.sent}
    assert sent[1][1] | (sent[1][2] << 8) == 1500
    assert sent[2][1] | (sent[2][2] << 8) == 3000


def test_park_goals_is_a_noop_before_start():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    hw._park_goals_at_measured()  # _srv is None; must not raise


def test_re_enabling_torque_parks_first():
    hw = ServoHardwareInterface(port="/dev/nonexistent", joint_ids=[1])
    _wire_up(hw, {1: _telemetry_block(pos=1234)})
    _stub_writes(hw)
    hw._read_once()
    hw._pending_torque = True

    hw._apply_pending_torque()

    sent = dict(hw._srv.groupSyncWrite.sent)
    assert sent[1][1] | (sent[1][2] << 8) == 1234
    # ... and the park precedes the torque-enable write
    assert hw.register_writes == [(1, STS_TORQUE_ENABLE, 1)]
    assert hw._torque_enabled is True
