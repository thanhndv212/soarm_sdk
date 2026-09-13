"""Unit tests for telemetry sinks and the recorder thread.

No serial port and no rerun install: the recorder is driven against a fake
interface, and RerunSink against an injected fake module, so what is tested is
this package's own logic — entity paths, API probing, buffering, failure
isolation — rather than a third party's.
"""

from __future__ import annotations

import json
import threading

import pytest

from soarm_sdk.robot.telemetry import ServoSample, TelemetryStream
from soarm_sdk.robot.telemetry_sinks import (
    JsonlSink,
    RerunSink,
    TelemetryRecorder,
    TelemetrySink,
)


def _sample(seq: int = 0, **kw) -> ServoSample:
    base = dict(
        t_mono=float(seq),
        seq=seq,
        ids=(1, 2),
        position_ticks=(2048, 1024),
        position_rad=(0.0, -1.0),
        velocity_ticks=(0, 5),
        velocity_rad_s=(0.0, 0.05),
        load_percent=(0.0, 12.5),
        current_mA=(0.0, 65.0),
        voltage_V=(12.0, 12.0),
        temperature_C=(40, 41),
        status_flags=(0, 0),
        moving=(False, True),
    )
    base.update(kw)
    return ServoSample(**base)


class _FakeInterface:
    """Just the two methods TelemetryRecorder uses."""

    def __init__(self):
        self.stream = None
        self.unsubscribed = False

    def subscribe(self, *, maxlen):
        self.stream = TelemetryStream(maxlen=maxlen)
        return self.stream

    def unsubscribe(self, stream):
        self.unsubscribed = True
        stream.close()


class _CollectingSink(TelemetrySink):
    def __init__(self):
        self.samples = []
        self.closed = False

    def write(self, sample):
        self.samples.append(sample)

    def close(self):
        self.closed = True


# ---------------------------------------------------------------------------
# JsonlSink
# ---------------------------------------------------------------------------


def test_jsonl_sink_writes_one_json_object_per_line(tmp_path):
    path = tmp_path / "telemetry.jsonl"
    sink = JsonlSink(path)

    sink.write(_sample(0))
    sink.write(_sample(1))
    sink.close()

    lines = path.read_text().strip().splitlines()
    assert len(lines) == 2
    assert [json.loads(line)["seq"] for line in lines] == [0, 1]
    assert json.loads(lines[0])["position_rad"] == [0.0, -1.0]


def test_jsonl_sink_creates_missing_parent_directories(tmp_path):
    path = tmp_path / "runs" / "deep" / "telemetry.jsonl"

    JsonlSink(path).close()

    assert path.exists()


def test_jsonl_sink_buffers_between_flushes(tmp_path):
    path = tmp_path / "t.jsonl"
    sink = JsonlSink(path, flush_every=3)

    for i in range(2):
        sink.write(_sample(i))
    assert path.read_text() == ""  # still buffered

    sink.write(_sample(2))
    assert len(path.read_text().strip().splitlines()) == 3

    sink.close()


def test_jsonl_sink_flushes_whatever_is_pending_on_close(tmp_path):
    path = tmp_path / "t.jsonl"
    sink = JsonlSink(path, flush_every=1000)
    sink.write(_sample(0))

    sink.close()

    assert len(path.read_text().strip().splitlines()) == 1


def test_jsonl_sink_appends_rather_than_truncating(tmp_path):
    path = tmp_path / "t.jsonl"
    first = JsonlSink(path)
    first.write(_sample(0))
    first.close()

    second = JsonlSink(path)
    second.write(_sample(1))
    second.close()

    assert len(path.read_text().strip().splitlines()) == 2


def test_jsonl_sink_rejects_a_nonsense_flush_interval(tmp_path):
    with pytest.raises(ValueError):
        JsonlSink(tmp_path / "t.jsonl", flush_every=0)


# ---------------------------------------------------------------------------
# RerunSink — against an injected fake, since rerun is not a dependency
# ---------------------------------------------------------------------------


class _FakeRerun:
    def __init__(self, *, scalar_name="Scalars", time_api="set_time_seconds"):
        self.logged = []
        self.times = []
        setattr(self, scalar_name, lambda v: ("scalar", v))
        if time_api == "set_time_seconds":
            self.set_time_seconds = lambda tl, t: self.times.append((tl, t))
        elif time_api == "set_time":
            self.set_time = lambda tl, duration: self.times.append((tl, duration))

    def log(self, entity, value):
        self.logged.append((entity, value))


def test_rerun_sink_logs_one_series_per_joint_and_field():
    fake = _FakeRerun()
    sink = RerunSink(joint_names=["shoulder_pan", "shoulder_lift"], rr_module=fake)

    sink.write(_sample(0))

    entities = [e for e, _ in fake.logged]
    assert "soarm/shoulder_pan/position_rad" in entities
    assert "soarm/shoulder_lift/current_mA" in entities
    # no command was issued, so no goal or error series
    assert not any("goal_position_rad" in e for e in entities)
    assert not any("tracking_error_rad" in e for e in entities)


def test_rerun_sink_falls_back_to_servo_ids_without_joint_names():
    fake = _FakeRerun()
    RerunSink(rr_module=fake).write(_sample(0))

    assert "soarm/id_1/position_rad" in [e for e, _ in fake.logged]


def test_rerun_sink_logs_goal_and_tracking_error_once_commanded():
    fake = _FakeRerun()
    sink = RerunSink(rr_module=fake)

    sink.write(
        _sample(
            0,
            goal_position_rad=(0.1, -0.9),
            goal_position_ticks=(2100, 1100),
            goal_speed_ticks=(300, 300),
        )
    )

    logged = dict(fake.logged)
    assert logged["soarm/id_1/goal_position_rad"] == ("scalar", 0.1)
    assert logged["soarm/id_1/tracking_error_rad"] == ("scalar", pytest.approx(0.1))


def test_rerun_sink_sets_the_timeline_from_the_sample_timestamp():
    fake = _FakeRerun()
    RerunSink(rr_module=fake, timeline="bus").write(_sample(7))

    assert fake.times == [("bus", 7.0)]


def test_rerun_sink_accepts_the_modern_set_time_spelling():
    fake = _FakeRerun(time_api="set_time")
    RerunSink(rr_module=fake).write(_sample(3))

    assert fake.times == [("bus", 3.0)]


def test_rerun_sink_accepts_the_singular_scalar_spelling():
    fake = _FakeRerun(scalar_name="Scalar")
    sink = RerunSink(rr_module=fake)

    sink.write(_sample(0))

    assert fake.logged  # resolved Scalar rather than raising


def test_rerun_sink_logs_without_a_timeline_when_none_is_available():
    fake = _FakeRerun(time_api=None)
    RerunSink(rr_module=fake).write(_sample(0))

    assert fake.logged
    assert fake.times == []


def test_rerun_sink_rejects_a_module_with_no_scalar_type():
    class _Bare:
        def log(self, *a):
            pass

    with pytest.raises(AttributeError):
        RerunSink(rr_module=_Bare())


# ---------------------------------------------------------------------------
# TelemetryRecorder
# ---------------------------------------------------------------------------


def _wait_for(predicate, timeout=2.0):
    deadline = threading.Event()
    for _ in range(int(timeout / 0.01)):
        if predicate():
            return True
        deadline.wait(0.01)
    return predicate()


def test_recorder_delivers_samples_to_every_sink():
    iface = _FakeInterface()
    a, b = _CollectingSink(), _CollectingSink()

    with TelemetryRecorder(iface, [a, b], poll_timeout=0.05):
        iface.stream._publish(_sample(1))
        assert _wait_for(lambda: a.samples and b.samples)

    assert [s.seq for s in a.samples] == [1]
    assert [s.seq for s in b.samples] == [1]


def test_recorder_closes_sinks_and_unsubscribes_on_exit():
    iface = _FakeInterface()
    sink = _CollectingSink()

    with TelemetryRecorder(iface, [sink], poll_timeout=0.05):
        pass

    assert sink.closed is True
    assert iface.unsubscribed is True


def test_recorder_drains_what_arrived_during_shutdown():
    iface = _FakeInterface()
    sink = _CollectingSink()
    rec = TelemetryRecorder(iface, [sink], poll_timeout=0.05)
    rec.start()
    iface.stream._publish(_sample(9))

    rec.stop()

    assert [s.seq for s in sink.samples] == [9]


def test_a_raising_sink_is_dropped_and_the_others_keep_receiving():
    class _Exploding(TelemetrySink):
        def write(self, sample):
            raise RuntimeError("sink is broken")

    iface = _FakeInterface()
    good = _CollectingSink()
    rec = TelemetryRecorder(iface, [_Exploding(), good], poll_timeout=0.05)
    rec.start()
    iface.stream._publish(_sample(1))
    assert _wait_for(lambda: len(good.samples) == 1)
    iface.stream._publish(_sample(2))
    assert _wait_for(lambda: len(good.samples) == 2)
    rec.stop()

    assert [s.seq for s in good.samples] == [1, 2]
    assert len(rec.failed_sinks) == 1


def test_starting_twice_is_a_noop():
    iface = _FakeInterface()
    rec = TelemetryRecorder(iface, [_CollectingSink()], poll_timeout=0.05)
    rec.start()
    first = iface.stream

    rec.start()

    assert iface.stream is first
    rec.stop()
