"""Serial port helper for communicating with STServo devices."""

from __future__ import annotations

import sys
import time
from typing import Iterable

import serial

DEFAULT_BAUDRATE = 1_000_000
LATENCY_TIMER = 50


class PortHandler:
    """Wrap ``pyserial`` to match the legacy SDK interface."""

    def __init__(self, port_name: str) -> None:
        self.is_open = False
        self.baudrate = DEFAULT_BAUDRATE
        self.packet_start_time = 0.0
        self.packet_timeout = 0.0
        self.tx_time_per_byte = 0.0

        self.is_using = False
        self.port_name = port_name
        self.ser: serial.Serial | None = None

    # --- lifecycle -----------------------------------------------------
    def openPort(self) -> bool:  # noqa: N802 (legacy API)
        return self.setBaudRate(self.baudrate)

    def closePort(self) -> None:  # noqa: N802
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.is_open = False

    def clearPort(self) -> None:  # noqa: N802
        if self.ser is not None:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

    # --- configuration -------------------------------------------------
    def setPortName(self, port_name: str) -> None:  # noqa: N802
        self.port_name = port_name

    def getPortName(self) -> str:  # noqa: N802
        return self.port_name

    def setBaudRate(self, baudrate: int) -> bool:  # noqa: N802
        baud = self._validate_baudrate(baudrate)
        if baud <= 0:
            return False
        self.baudrate = baudrate
        return self._setup_port()

    def getBaudRate(self) -> int:  # noqa: N802
        return self.baudrate

    # --- I/O helpers ---------------------------------------------------
    def getBytesAvailable(self) -> int:  # noqa: N802
        return 0 if self.ser is None else self.ser.in_waiting

    def readPort(self, length: int) -> bytes:  # noqa: N802
        if self.ser is None:
            return b""
        data = self.ser.read(length)
        if sys.version_info <= (3, 0):  # pragma: no cover - legacy guard
            return bytes([ord(ch) for ch in data])
        return data

    def writePort(self, packet: Iterable[int]) -> int:  # noqa: N802
        if self.ser is None:
            raise RuntimeError("Serial port is not open")
        return self.ser.write(bytes(packet))

    # --- timeout helpers -----------------------------------------------
    def setPacketTimeout(self, packet_length: int) -> None:  # noqa: N802
        self.packet_start_time = self._current_time()
        self.packet_timeout = (
            self.tx_time_per_byte * packet_length
            + self.tx_time_per_byte * 3.0
            + LATENCY_TIMER
        )

    def setPacketTimeoutMillis(self, msec: float) -> None:  # noqa: N802
        self.packet_start_time = self._current_time()
        self.packet_timeout = msec

    def isPacketTimeout(self) -> bool:  # noqa: N802
        if self.getTimeSinceStart() > self.packet_timeout:
            self.packet_timeout = 0
            return True
        return False

    def getTimeSinceStart(self) -> float:  # noqa: N802
        elapsed = self._current_time() - self.packet_start_time
        if elapsed < 0.0:
            self.packet_start_time = self._current_time()
            return 0.0
        return elapsed

    # --- internal helpers ----------------------------------------------
    def _setup_port(self) -> bool:
        if self.is_open:
            self.closePort()

        self.ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            timeout=0,
        )
        self.is_open = True
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0
        return True

    @staticmethod
    def _validate_baudrate(baudrate: int) -> int:
        if baudrate in {
            4800,
            9600,
            14400,
            19200,
            38400,
            57600,
            115200,
            128000,
            250000,
            500000,
            1000000,
        }:
            return baudrate
        return -1

    @staticmethod
    def _current_time() -> float:
        return round(time.time() * 1_000_000_000) / 1_000_000.0
