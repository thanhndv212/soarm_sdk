"""Unit tests for soarm_sdk.cli.dashboard's public, reusable CLI layer.

build_parser/resolve_device/resolve_use_stream are pure functions over
argparse.Namespace / plain values — no viser, no hardware. launch() does
construct a DashboardApp, which binds a real port; that call is faked
here (matching this repo's convention of never spinning up a real viser
server or opening a real port in a test) so what's actually asserted is
that launch() resolves arguments correctly and wires them through, not
that Viser itself works.

This module is also soarm_tamp's dashboard CLI layer (see
soarm_tamp/dashboard/__main__.py), so default_use_stream is exercised at
both its True and False settings throughout, not just the value this
package's own two dashboards happen to use.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from soarm_sdk.cli.dashboard import (
    build_parser,
    launch,
    resolve_device,
    resolve_use_stream,
)
from soarm_sdk.dashboard.app import DashboardProfile

# ---------------------------------------------------------------------------
# build_parser
# ---------------------------------------------------------------------------


def test_default_use_stream_false_adds_opt_in_stream_flag():
    args = build_parser("d", default_use_stream=False).parse_args([])
    assert args.stream is False
    assert not hasattr(args, "no_stream")


def test_default_use_stream_true_adds_opt_out_no_stream_flag():
    args = build_parser("d", default_use_stream=True).parse_args([])
    assert args.no_stream is False
    assert not hasattr(args, "stream")


def test_stream_flag_parses_when_present():
    args = build_parser("d", default_use_stream=False).parse_args(["--stream"])
    assert args.stream is True


def test_no_stream_flag_parses_when_present():
    args = build_parser("d", default_use_stream=True).parse_args(["--no-stream"])
    assert args.no_stream is True


def test_urdf_default_comes_from_caller_not_this_packages_own_path():
    custom = Path("/some/other/robot.urdf")
    args = build_parser("d", default_urdf=custom).parse_args([])
    assert args.urdf == custom


def test_rerun_flag_present_regardless_of_stream_polarity():
    for default_use_stream in (True, False):
        args = build_parser("d", default_use_stream=default_use_stream).parse_args(["--rerun"])
        assert args.rerun is True


@pytest.mark.parametrize(
    "argv,attr,expected",
    [
        (["--device", "/dev/ttyX"], "device", "/dev/ttyX"),
        (["--baud", "57600"], "baud", 57600),
        (["--port", "9999"], "port", 9999),
        (["--interval-ms", "50"], "interval_ms", 50),
        (["--calibration", "/tmp/cal.json"], "calibration", Path("/tmp/cal.json")),
    ],
)
def test_shared_flags_parse(argv, attr, expected):
    args = build_parser("d").parse_args(argv)
    assert getattr(args, attr) == expected


# ---------------------------------------------------------------------------
# resolve_device
# ---------------------------------------------------------------------------


class _Args:
    def __init__(self, device=""):
        self.device = device


def test_resolve_device_keeps_explicit_device_and_never_scans(monkeypatch):
    def _boom():
        raise AssertionError("must not scan when a device was given explicitly")

    monkeypatch.setattr("soarm_sdk.cli.dashboard.get_available_ports", _boom)
    assert resolve_device(_Args(device="/dev/ttyX"), "label") == "/dev/ttyX"


def test_resolve_device_auto_selects_first_available_port(monkeypatch, capsys):
    monkeypatch.setattr(
        "soarm_sdk.cli.dashboard.get_available_ports",
        lambda: [("/dev/ttyAUTO", ""), ("/dev/ttyOTHER", "")],
    )
    device = resolve_device(_Args(device=""), "mylabel")
    assert device == "/dev/ttyAUTO"
    assert "mylabel" in capsys.readouterr().out


def test_resolve_device_stays_empty_when_nothing_found(monkeypatch):
    monkeypatch.setattr("soarm_sdk.cli.dashboard.get_available_ports", lambda: [])
    assert resolve_device(_Args(device=""), "label") == ""


# ---------------------------------------------------------------------------
# resolve_use_stream
# ---------------------------------------------------------------------------


class _StreamArgs:
    def __init__(self, *, stream=False, no_stream=False, rerun=False):
        self.stream = stream
        self.no_stream = no_stream
        self.rerun = rerun


def test_opt_in_stream_off_by_default():
    args = _StreamArgs(stream=False)
    assert resolve_use_stream(args, default_use_stream=False, log_label="x") is False


def test_opt_in_stream_on_when_flag_given():
    args = _StreamArgs(stream=True)
    assert resolve_use_stream(args, default_use_stream=False, log_label="x") is True


def test_opt_out_stream_on_by_default():
    args = _StreamArgs(no_stream=False)
    assert resolve_use_stream(args, default_use_stream=True, log_label="x") is True


def test_opt_out_stream_off_when_no_stream_given():
    args = _StreamArgs(no_stream=True)
    assert resolve_use_stream(args, default_use_stream=True, log_label="x") is False


def test_rerun_forces_stream_on_under_opt_in_default(capsys):
    args = _StreamArgs(stream=False, rerun=True)
    assert resolve_use_stream(args, default_use_stream=False, log_label="lbl") is True
    assert "lbl" in capsys.readouterr().out


def test_rerun_forces_stream_on_under_opt_out_default_even_with_no_stream(capsys):
    args = _StreamArgs(no_stream=True, rerun=True)
    assert resolve_use_stream(args, default_use_stream=True, log_label="lbl") is True
    assert "lbl" in capsys.readouterr().out


def test_rerun_with_stream_already_on_prints_nothing_extra(capsys):
    args = _StreamArgs(stream=True, rerun=True)
    assert resolve_use_stream(args, default_use_stream=False, log_label="lbl") is True
    assert capsys.readouterr().out == ""


# ---------------------------------------------------------------------------
# launch — DashboardApp itself is faked; only the wiring is under test
# ---------------------------------------------------------------------------


class _FakeApp:
    instances = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.registered = None
        self.ran = False
        _FakeApp.instances.append(self)

    def run(self):
        self.ran = True


@pytest.fixture(autouse=True)
def _reset_fake_app():
    _FakeApp.instances.clear()
    yield
    _FakeApp.instances.clear()


def _profile(**overrides):
    defaults = dict(
        name="p",
        title="Test Profile",
        register=lambda app: setattr(app, "registered", True),
        description="a test profile",
    )
    defaults.update(overrides)
    return DashboardProfile(**defaults)


def test_launch_constructs_app_with_resolved_device_and_stream(monkeypatch):
    monkeypatch.setattr("soarm_sdk.cli.dashboard.DashboardApp", _FakeApp)
    monkeypatch.setattr(
        "soarm_sdk.cli.dashboard.get_available_ports", lambda: [("/dev/ttyAUTO", "")]
    )

    launch(_profile(), [], "lbl", default_urdf=Path("/urdf"), default_use_stream=False)

    app = _FakeApp.instances[0]
    assert app.kwargs["device"] == "/dev/ttyAUTO"
    assert app.kwargs["use_stream"] is False
    assert app.kwargs["rerun"] is False
    assert app.kwargs["title"] == "Test Profile"
    assert app.registered is True
    assert app.ran is True


def test_launch_forwards_explicit_device_and_rerun(monkeypatch):
    monkeypatch.setattr("soarm_sdk.cli.dashboard.DashboardApp", _FakeApp)

    launch(
        _profile(),
        ["--device", "/dev/ttyManual", "--rerun"],
        "lbl",
        default_urdf=Path("/urdf"),
        default_use_stream=False,
    )

    app = _FakeApp.instances[0]
    assert app.kwargs["device"] == "/dev/ttyManual"
    assert app.kwargs["use_stream"] is True  # --rerun forced it on
    assert app.kwargs["rerun"] is True


def test_launch_respects_opt_out_stream_default(monkeypatch):
    monkeypatch.setattr("soarm_sdk.cli.dashboard.DashboardApp", _FakeApp)

    launch(
        _profile(),
        ["--no-stream"],
        "lbl",
        default_urdf=Path("/urdf"),
        default_use_stream=True,
    )

    assert _FakeApp.instances[0].kwargs["use_stream"] is False


def test_launch_uses_default_urdf_when_not_overridden(monkeypatch):
    monkeypatch.setattr("soarm_sdk.cli.dashboard.DashboardApp", _FakeApp)
    custom_urdf = Path("/some/other/robot.urdf")

    launch(_profile(), [], "lbl", default_urdf=custom_urdf, default_use_stream=False)

    assert _FakeApp.instances[0].kwargs["urdf_path"] == custom_urdf
