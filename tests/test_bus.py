"""Unit tests for soarm_sdk.bus.

Covers pure control-flow logic (scan_servos, write1/write2, list_ports)
against fake packet-handler/port doubles. discover_servos/read_diagnostics
are thin open/close wrappers around scan_servos and are not re-tested here
since they require an actual open serial port.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from soarm_sdk import bus
from soarm_sdk.protocol_packet_handler import PacketResult
from soarm_sdk.stservo_def import COMM_RX_FAIL, COMM_SUCCESS


# ---------------------------------------------------------------------------
# list_ports
# ---------------------------------------------------------------------------


class _FakeComPort:
    def __init__(self, device: str, description: str = ""):
        self.device = device
        self.description = description


def test_list_ports_filters_to_usb_devices(monkeypatch):
    fake_ports = [
        _FakeComPort("/dev/tty.usbserial-A", "USB Serial A"),
        _FakeComPort("/dev/tty.Bluetooth-Incoming-Port"),
        _FakeComPort("/dev/tty.usbmodem123", "USB Modem"),
    ]
    monkeypatch.setattr(bus._list_ports_mod, "comports", lambda: fake_ports)

    result = bus.list_ports()

    assert result == [
        ("/dev/tty.usbserial-A", "USB Serial A"),
        ("/dev/tty.usbmodem123", "USB Modem"),
    ]


def test_list_ports_falls_back_to_glob_when_comports_empty(monkeypatch):
    monkeypatch.setattr(bus._list_ports_mod, "comports", lambda: [])

    class _FakeDevPath:
        def __str__(self) -> str:
            return "/dev/tty.usbserial-FAKE"

        def is_char_device(self) -> bool:
            return True

    monkeypatch.setattr(Path, "glob", lambda self, pattern: [_FakeDevPath()])

    assert bus.list_ports() == [("/dev/tty.usbserial-FAKE", "")]


def test_list_ports_returns_empty_when_nothing_found(monkeypatch):
    monkeypatch.setattr(bus._list_ports_mod, "comports", lambda: [])
    monkeypatch.setattr(Path, "glob", lambda self, pattern: [])

    assert bus.list_ports() == []


# ---------------------------------------------------------------------------
# scan_servos
# ---------------------------------------------------------------------------


class _FakePacketHandler:
    """Stand-in for an `sts` instance bound to an already-open port."""

    def __init__(self, responses: dict[int, object]):
        self._responses = responses

    def ping(self, servo_id: int):
        return self._responses.get(servo_id, PacketResult([], COMM_RX_FAIL))


def test_scan_servos_collects_only_successful_responses():
    handler = _FakePacketHandler(
        {
            1: PacketResult([777], COMM_SUCCESS, 0),
            2: PacketResult([], COMM_RX_FAIL, 0),
        }
    )

    found = bus.scan_servos(handler, [1, 2, 3])

    assert found == {1: 777}


def test_scan_servos_skips_unrecognized_response_shape():
    class _WeirdHandler:
        def ping(self, servo_id: int):
            return None  # neither a tuple nor a PacketResult-like object

    # Must not raise — unrecognized shapes are silently skipped.
    assert bus.scan_servos(_WeirdHandler(), [1]) == {}


def test_scan_servos_accepts_plain_tuple_responses():
    class _TupleHandler:
        def ping(self, servo_id: int):
            return (COMM_SUCCESS, 0, [42])

    assert bus.scan_servos(_TupleHandler(), [5]) == {5: 42}


# ---------------------------------------------------------------------------
# write1 / write2
# ---------------------------------------------------------------------------


class _FakeWritePacketHandler:
    def __init__(self, result: int, error: int = 0):
        self._result = result
        self._error = error
        self.calls: list[tuple[int, int, int]] = []

    def write1ByteTxRx(self, servo_id, address, value):
        self.calls.append((servo_id, address, value))
        return self._result, self._error

    def write2ByteTxRx(self, servo_id, address, value):
        self.calls.append((servo_id, address, value))
        return self._result, self._error


def test_write1_succeeds_silently_on_comm_success():
    handler = _FakeWritePacketHandler(COMM_SUCCESS)
    bus.write1(handler, servo_id=1, address=40, value=1, label="torque")
    assert handler.calls == [(1, 40, 1)]


def test_write1_raises_on_failure():
    handler = _FakeWritePacketHandler(COMM_RX_FAIL, error=5)
    with pytest.raises(RuntimeError, match="torque"):
        bus.write1(handler, servo_id=1, address=40, value=1, label="torque")


def test_write2_succeeds_silently_on_comm_success():
    handler = _FakeWritePacketHandler(COMM_SUCCESS)
    bus.write2(handler, servo_id=1, address=9, value=100, label="min angle")
    assert handler.calls == [(1, 9, 100)]


def test_write2_raises_on_nonzero_error_even_if_comm_succeeds():
    handler = _FakeWritePacketHandler(COMM_SUCCESS, error=1)
    with pytest.raises(RuntimeError, match="min angle"):
        bus.write2(handler, servo_id=1, address=9, value=100, label="min angle")
