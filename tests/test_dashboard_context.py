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


# ---------------------------------------------------------------------------
# Streaming mode
#
# The dashboard consumes ServoHardwareInterface telemetry instead of reopening
# the port every poll. Driven here against a fake interface: what matters is
# that state is fed from samples, that bus() borrows the held port rather than
# opening a second one, and that connecting never energises the arm.
# ---------------------------------------------------------------------------

import threading  # noqa: E402
from contextlib import contextmanager  # noqa: E402

from soarm_sdk.robot.telemetry import ServoSample, TelemetryStream  # noqa: E402


class _FakeInterface:
    _state_freq = 100.0

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.started = False
        self.stopped = False
        self.lent = 0
        self.srv = object()
        self.stream = TelemetryStream(maxlen=100)

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def subscribe(self, *, maxlen):
        return self.stream

    @contextmanager
    def lend_bus(self, **_):
        self.lent += 1
        yield self.srv


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


def _streaming_context(monkeypatch):
    ctx = DashboardContext(
        device_h=_FakeGuiHandle("/dev/nonexistent"),
        baud_h=_FakeGuiHandle(1_000_000),
        interval_h=_FakeGuiHandle(200),
        conn_status_md=_FakeGuiHandle(None),
        joint_ids=[1, 2],
        use_stream=True,
    )
    made = {}

    def _factory(**kwargs):
        made["iface"] = _FakeInterface(**kwargs)
        return made["iface"]

    monkeypatch.setattr(
        "soarm_sdk.robot.hardware.ServoHardwareInterface", _factory
    )
    return ctx, made


def test_streaming_is_off_by_default():
    assert _make_context().use_stream is False


def test_connecting_in_stream_mode_never_enables_torque(monkeypatch):
    ctx, made = _streaming_context(monkeypatch)

    ctx.start_polling()
    try:
        assert made["iface"].kwargs["torque_on_start"] is False
        assert made["iface"].started is True
    finally:
        ctx.stop_polling()


def test_stream_samples_populate_state_including_health(monkeypatch):
    ctx, made = _streaming_context(monkeypatch)
    ctx.start_polling()
    try:
        made["iface"].stream._publish(_sample(seq=7))
        deadline = threading.Event()
        for _ in range(200):
            with ctx.lock:
                if ctx.state.positions:
                    break
            deadline.wait(0.01)

        with ctx.lock:
            assert ctx.state.positions == {1: 1000, 2: 1001}
            assert ctx.state.speeds == {1: 10, 2: 11}
            # health arrives with every sample, not every fifth poll
            assert ctx.state.temps == {1: 40, 2: 41}
            assert ctx.state.currents == {1: 50.0, 2: 51.0}
            assert ctx.state.connected is True
            assert ctx.state.poll_count == 7
    finally:
        ctx.stop_polling()


def test_bus_borrows_the_held_port_instead_of_opening_another(monkeypatch):
    ctx, made = _streaming_context(monkeypatch)
    ctx.start_polling()
    try:
        with ctx.bus() as srv:
            assert srv is made["iface"].srv
        assert made["iface"].lent == 1
    finally:
        ctx.stop_polling()


def test_stop_polling_stops_the_interface(monkeypatch):
    ctx, made = _streaming_context(monkeypatch)
    ctx.start_polling()

    ctx.stop_polling()

    assert made["iface"].stopped is True
    assert ctx.state.connected is False


def test_a_raising_hook_does_not_report_the_bus_as_disconnected(monkeypatch):
    # In the legacy loop a throwing hook landed in the shared try and set
    # connected=False — the UI claimed a bus failure that never happened.
    ctx, made = _streaming_context(monkeypatch)
    ctx.add_sample_hook(lambda *_: (_ for _ in ()).throw(RuntimeError("boom")))
    ctx.start_polling()
    try:
        made["iface"].stream._publish(_sample(seq=3))
        deadline = threading.Event()
        for _ in range(200):
            with ctx.lock:
                if ctx.state.positions:
                    break
            deadline.wait(0.01)

        with ctx.lock:
            assert ctx.state.connected is True
            assert ctx.state.poll_error is None
    finally:
        ctx.stop_polling()


def test_connect_failure_is_reported_not_raised(monkeypatch):
    ctx = DashboardContext(
        device_h=_FakeGuiHandle("/dev/nonexistent"),
        baud_h=_FakeGuiHandle(1_000_000),
        interval_h=_FakeGuiHandle(200),
        conn_status_md=_FakeGuiHandle(None),
        joint_ids=[1],
        use_stream=True,
    )

    def _boom(**_):
        raise RuntimeError("cannot open port")

    monkeypatch.setattr(
        "soarm_sdk.robot.hardware.ServoHardwareInterface", _boom
    )

    ctx.start_polling()  # must not raise

    with ctx.lock:
        assert ctx.state.connected is False
        assert "cannot open port" in ctx.state.poll_error
