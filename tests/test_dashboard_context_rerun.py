"""Unit tests for DashboardContext's optional Rerun feed (rerun=True).

Separate from test_dashboard_context.py because this needs the `telemetry`
extra (rerun-sdk) on top of viser — skipped entirely if either is missing.
soarm_sdk.monitoring.blueprint imports rerun.blueprint at module load time,
so even patching it requires rerun to be importable first.
"""

from __future__ import annotations

from contextlib import contextmanager

import pytest

viser = pytest.importorskip("viser")
rerun = pytest.importorskip("rerun")

from soarm_sdk.dashboard.context import DashboardContext  # noqa: E402
from soarm_sdk.robot.telemetry import ServoSample, TelemetryStream  # noqa: E402


class _FakeGuiHandle:
    def __init__(self, value):
        self.value = value
        self.content = None


class _FakeInterface:
    _state_freq = 100.0

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.started = False
        self.stopped = False
        self.srv = object()
        self.stream = TelemetryStream(maxlen=100)
        self.subscribe_calls = 0

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def subscribe(self, *, maxlen):
        self.subscribe_calls += 1
        # A distinct stream per call, matching the real interface: each
        # subscriber gets its own queue, not a shared one.
        return TelemetryStream(maxlen=maxlen)

    @contextmanager
    def lend_bus(self, **_):
        yield self.srv


class _FakeRecorder:
    instances = []

    def __init__(self, interface, sinks):
        self.interface = interface
        self.sinks = sinks
        self.started = False
        self.stopped = False
        _FakeRecorder.instances.append(self)

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True


class _FakeSink:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


def _sample(seq=1, ids=(1, 2)):
    n = len(ids)
    return ServoSample(
        t_mono=float(seq), seq=seq, ids=tuple(ids),
        position_ticks=tuple(1000 + i for i in range(n)),
        position_rad=(0.0,) * n,
        velocity_ticks=tuple(10 + i for i in range(n)),
        velocity_rad_s=(0.0,) * n,
        load_percent=(0.0,) * n,
        current_mA=tuple(50.0 + i for i in range(n)),
        voltage_V=(12.0,) * n,
        temperature_C=tuple(40 + i for i in range(n)),
        status_flags=(0,) * n,
        moving=(False,) * n,
    )


@pytest.fixture(autouse=True)
def _reset_fake_recorder_instances():
    _FakeRecorder.instances.clear()
    yield
    _FakeRecorder.instances.clear()


def _rerun_context(monkeypatch, *, rerun_enabled=True, joint_ids=None):
    ctx = DashboardContext(
        device_h=_FakeGuiHandle("/dev/nonexistent"),
        baud_h=_FakeGuiHandle(1_000_000),
        interval_h=_FakeGuiHandle(200),
        conn_status_md=_FakeGuiHandle(None),
        joint_ids=joint_ids if joint_ids is not None else [1, 2],
        use_stream=True,
        rerun=rerun_enabled,
    )
    made = {}

    def _iface_factory(**kwargs):
        made["iface"] = _FakeInterface(**kwargs)
        return made["iface"]

    monkeypatch.setattr("soarm_sdk.robot.hardware.ServoHardwareInterface", _iface_factory)
    monkeypatch.setattr("soarm_sdk.robot.telemetry_sinks.TelemetryRecorder", _FakeRecorder)
    monkeypatch.setattr("soarm_sdk.robot.telemetry_sinks.RerunSink", _FakeSink)
    monkeypatch.setattr("rerun.init", lambda *a, **k: None)
    return ctx, made


def test_rerun_disabled_by_default():
    ctx = DashboardContext(
        device_h=_FakeGuiHandle("/dev/nonexistent"),
        baud_h=_FakeGuiHandle(1_000_000),
        interval_h=_FakeGuiHandle(200),
        conn_status_md=_FakeGuiHandle(None),
        joint_ids=[1, 2],
    )
    assert ctx.rerun_enabled is False


def test_rerun_enabled_starts_a_recorder_on_the_same_interface(monkeypatch):
    ctx, made = _rerun_context(monkeypatch)
    ctx.start_polling()
    try:
        assert len(_FakeRecorder.instances) == 1
        recorder = _FakeRecorder.instances[0]
        assert recorder.interface is made["iface"]
        assert recorder.started is True
        # Two independent subscriptions: the dashboard's own poll loop and
        # this Rerun recorder — never one shared queue.
        assert made["iface"].subscribe_calls == 1  # dashboard's own _stream
    finally:
        ctx.stop_polling()


def test_rerun_disabled_never_creates_a_recorder(monkeypatch):
    ctx, made = _rerun_context(monkeypatch, rerun_enabled=False)
    ctx.start_polling()
    try:
        assert _FakeRecorder.instances == []
    finally:
        ctx.stop_polling()


def test_rerun_failure_does_not_break_streaming(monkeypatch):
    ctx, made = _rerun_context(monkeypatch)
    monkeypatch.setattr(
        "rerun.init",
        lambda *a, **k: (_ for _ in ()).throw(RuntimeError("no viewer binary")),
    )

    ctx.start_polling()
    try:
        assert made["iface"].started is True
        assert "Rerun failed to start" in ctx.conn_status_md.content
        assert "Streaming" in ctx.conn_status_md.content
    finally:
        ctx.stop_polling()


def test_stop_polling_stops_the_rerun_recorder(monkeypatch):
    ctx, made = _rerun_context(monkeypatch)
    ctx.start_polling()
    recorder = _FakeRecorder.instances[0]

    ctx.stop_polling()

    assert recorder.stopped is True
    assert made["iface"].stopped is True
    assert ctx._rerun_recorder is None


def test_rerun_uses_joint_names_matching_joint_ids(monkeypatch):
    from soarm_sdk.dashboard.fk import SOARM100_JOINT_NAMES

    ctx, made = _rerun_context(monkeypatch, joint_ids=[3, 4])
    ctx.start_polling()
    try:
        sink = _FakeRecorder.instances[0].sinks[0]
        assert sink.kwargs["joint_names"] == [
            SOARM100_JOINT_NAMES[2],
            SOARM100_JOINT_NAMES[3],
        ]
    finally:
        ctx.stop_polling()
