"""CLI-level behaviour of ``soarm-reconfigure`` that the library tests miss."""

from __future__ import annotations

import pytest

from soarm_sdk.cli import reconfigure


def test_list_ports_is_a_query_and_does_not_reconfigure(monkeypatch, capsys):
    """--list-ports prints the ports and stops.

    It used to fall through into run_calibration(), which opens --device
    (default /dev/ttyUSB0) and raises SerialException on a machine that
    only wanted the port list -- after printing that list twice, because
    run_calibration prints it again for Namespace callers.
    """
    printed = []
    monkeypatch.setattr(reconfigure, "print_ports", lambda: printed.append("ports"))

    def _must_not_run(_args):  # pragma: no cover - the failure is the assert
        raise AssertionError("--list-ports must not start a reconfiguration run")

    monkeypatch.setattr(reconfigure, "run_calibration", _must_not_run)

    assert reconfigure.main(["--list-ports"]) == 0
    assert printed == ["ports"], "the port list should be printed exactly once"


def test_without_list_ports_the_run_still_happens(monkeypatch):
    """The early return is scoped to the flag, not to every invocation."""
    monkeypatch.setattr(reconfigure, "print_ports", lambda: None)
    monkeypatch.setattr(reconfigure, "run_calibration", lambda _args: 0)
    assert reconfigure.main(["--device", "/dev/null"]) == 0


def test_run_calibration_errors_become_parser_errors(monkeypatch):
    """A bad configuration exits 2 with a message, not a traceback."""
    monkeypatch.setattr(reconfigure, "print_ports", lambda: None)

    def _boom(_args):
        raise ValueError("nope")

    monkeypatch.setattr(reconfigure, "run_calibration", _boom)
    with pytest.raises(SystemExit) as exc:
        reconfigure.main(["--device", "/dev/null"])
    assert exc.value.code == 2
