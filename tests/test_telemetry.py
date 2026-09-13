"""Unit tests for the telemetry tap.

Two properties matter more than the data plumbing and are tested first: the
bus thread must never be harmed by a consumer, and a consumer that falls
behind must lose the oldest samples rather than block the producer. The bus
thread also issues servo writes, so a producer stalled by a consumer would
delay commands to the arm.
"""

from __future__ import annotations

import threading

import pytest

from soarm_sdk.robot.telemetry import ServoSample, TelemetryStream


def _sample(seq: int = 0, **kw) -> ServoSample:
    base = dict(
        t_mono=float(seq),
        seq=seq,
        ids=(1,),
        position_ticks=(2048,),
        position_rad=(0.0,),
        velocity_ticks=(0,),
        velocity_rad_s=(0.0,),
        load_percent=(0.0,),
        current_mA=(0.0,),
        voltage_V=(12.0,),
        temperature_C=(40,),
        status_flags=(0,),
        moving=(False,),
    )
    base.update(kw)
    return ServoSample(**base)


# ---------------------------------------------------------------------------
# TelemetryStream
# ---------------------------------------------------------------------------


def test_drain_returns_samples_oldest_first_and_empties_the_buffer():
    stream = TelemetryStream(maxlen=8)
    for i in range(3):
        stream._publish(_sample(i))

    drained = stream.drain()

    assert [s.seq for s in drained] == [0, 1, 2]
    assert stream.drain() == []


def test_overflow_drops_the_oldest_and_counts_it():
    stream = TelemetryStream(maxlen=3)
    for i in range(5):
        stream._publish(_sample(i))

    assert [s.seq for s in stream.drain()] == [2, 3, 4]
    assert stream.dropped == 2
    assert stream.published == 5


def test_publishing_never_blocks_a_full_stream():
    # A consumer that never drains must not stall the producer. If _publish
    # ever grew a blocking wait, this would hang rather than fail.
    stream = TelemetryStream(maxlen=2)
    done = threading.Event()

    def produce():
        for i in range(1000):
            stream._publish(_sample(i))
        done.set()

    threading.Thread(target=produce, daemon=True).start()

    assert done.wait(timeout=5.0), "producer blocked on a full stream"
    assert stream.dropped == 998


def test_wait_returns_immediately_once_a_sample_is_buffered():
    stream = TelemetryStream()
    assert stream.wait(timeout=0.01) is False

    stream._publish(_sample(1))

    assert stream.wait(timeout=0.01) is True


def test_close_discards_buffered_samples_and_rejects_new_ones():
    stream = TelemetryStream()
    stream._publish(_sample(1))

    stream.close()
    stream._publish(_sample(2))

    assert stream.closed is True
    assert stream.drain() == []


def test_maxlen_must_be_positive():
    with pytest.raises(ValueError):
        TelemetryStream(maxlen=0)


# ---------------------------------------------------------------------------
# ServoSample
# ---------------------------------------------------------------------------


def test_tracking_error_is_none_before_any_command():
    assert _sample().tracking_error_rad() is None


def test_tracking_error_is_commanded_minus_measured():
    sample = _sample(
        position_rad=(0.10, -0.20),
        goal_position_rad=(0.15, -0.18),
        goal_position_ticks=(2100, 1900),
    )

    assert sample.tracking_error_rad() == pytest.approx((0.05, 0.02))


def test_as_dict_omits_the_goal_fields_when_nothing_was_commanded():
    d = _sample().as_dict()

    assert d["seq"] == 0
    assert "goal_position_ticks" not in d


def test_as_dict_is_json_serialisable_with_a_command():
    import json

    sample = _sample(
        goal_position_ticks=(2100,),
        goal_position_rad=(0.05,),
        goal_speed_ticks=(300,),
        goal_t_mono=1.5,
    )

    round_tripped = json.loads(json.dumps(sample.as_dict()))

    assert round_tripped["goal_position_ticks"] == [2100]
    assert round_tripped["goal_t_mono"] == 1.5


def test_samples_are_immutable_once_published():
    sample = _sample()
    with pytest.raises(Exception):
        sample.seq = 99  # type: ignore[misc]
