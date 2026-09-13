"""Unit tests for the sts telemetry accessors.

`ReadLoad` and `ReadCurrent` address 2-byte sign-magnitude registers
(PRESENT_LOAD 60-61, PRESENT_CURRENT 69-70). They previously issued 1-byte
reads, truncating to the low 8 bits and dropping the sign entirely. These
tests pin the width and the sign decode against canned register bytes; no
serial port is involved.
"""

from __future__ import annotations

import pytest

from soarm_sdk.protocol.packet_handler import PacketResult
from soarm_sdk.protocol.registers import (
    COMM_RX_TIMEOUT,
    COMM_SUCCESS,
    STS_PRESENT_CURRENT_L,
    STS_PRESENT_LOAD_L,
    STS_PRESENT_TEMPERATURE,
    STS_PRESENT_VOLTAGE,
)
from soarm_sdk.protocol.sts import sts


class _FakeReadSts(sts):
    """An ``sts`` whose register reads come from a canned address -> bytes map.

    Overrides ``readTxRx``, the single funnel every ``readNByteTxRx`` helper
    goes through, so the width each accessor asks for is observable.
    """

    def __init__(self, registers: dict, result: int = COMM_SUCCESS) -> None:
        super().__init__(None)
        self._registers = registers
        self._result = result
        self.requests: list = []  # (address, length) per call

    def readTxRx(self, sts_id, address, length):  # noqa: N802
        self.requests.append((address, length))
        if self._result != COMM_SUCCESS:
            return PacketResult([], self._result)
        return PacketResult(list(self._registers[address][:length]), COMM_SUCCESS)


def _word(value: int) -> list:
    """Little-endian byte pair, matching ``sts_makeword``'s low/high order."""
    return [value & 0xFF, (value >> 8) & 0xFF]


# ---------------------------------------------------------------------------
# PRESENT_LOAD — 2 bytes, magnitude in bits 0-9, direction in bit 10
# ---------------------------------------------------------------------------


def test_read_load_uses_a_two_byte_read():
    srv = _FakeReadSts({STS_PRESENT_LOAD_L: _word(500)})

    srv.ReadLoad(1)

    assert srv.requests == [(STS_PRESENT_LOAD_L, 2)]


def test_read_load_decodes_positive_magnitude_as_percent():
    srv = _FakeReadSts({STS_PRESENT_LOAD_L: _word(500)})

    value, result, _ = srv.ReadLoad(1)

    assert result == COMM_SUCCESS
    assert value == pytest.approx(50.0)


def test_read_load_decodes_the_direction_bit_as_a_negative_load():
    # bit 10 set == the other direction; magnitude is still 500.
    srv = _FakeReadSts({STS_PRESENT_LOAD_L: _word((1 << 10) | 500)})

    value, _, _ = srv.ReadLoad(1)

    assert value == pytest.approx(-50.0)


def test_read_load_full_scale_survives_the_old_one_byte_truncation():
    # 1000 == 100.0%. Truncated to one byte this read 1000 & 0xFF == 232,
    # i.e. 23.2% — the bug this test exists to prevent regressing.
    srv = _FakeReadSts({STS_PRESENT_LOAD_L: _word(1000)})

    value, _, _ = srv.ReadLoad(1)

    assert value == pytest.approx(100.0)


def test_read_load_returns_zero_on_a_failed_read():
    srv = _FakeReadSts({}, result=COMM_RX_TIMEOUT)

    value, result, _ = srv.ReadLoad(1)

    assert value == 0.0
    assert result == COMM_RX_TIMEOUT


# ---------------------------------------------------------------------------
# PRESENT_CURRENT — 2 bytes, sign-magnitude in bit 15, 6.5 mA per LSB
# ---------------------------------------------------------------------------


def test_read_current_uses_a_two_byte_read():
    srv = _FakeReadSts({STS_PRESENT_CURRENT_L: _word(100)})

    srv.ReadCurrent(1)

    assert srv.requests == [(STS_PRESENT_CURRENT_L, 2)]


def test_read_current_scales_by_6_5_ma_per_lsb():
    srv = _FakeReadSts({STS_PRESENT_CURRENT_L: _word(100)})

    value, _, _ = srv.ReadCurrent(1)

    assert value == pytest.approx(650.0)


def test_read_current_decodes_the_sign_bit():
    srv = _FakeReadSts({STS_PRESENT_CURRENT_L: _word((1 << 15) | 100)})

    value, _, _ = srv.ReadCurrent(1)

    assert value == pytest.approx(-650.0)


def test_read_current_above_one_byte_is_not_truncated():
    # 300 LSB == 1950 mA. One byte would have read 300 & 0xFF == 44 -> 286 mA.
    srv = _FakeReadSts({STS_PRESENT_CURRENT_L: _word(300)})

    value, _, _ = srv.ReadCurrent(1)

    assert value == pytest.approx(1950.0)


# ---------------------------------------------------------------------------
# The genuinely 1-byte registers must stay 1-byte
# ---------------------------------------------------------------------------


def test_voltage_and_temperature_remain_single_byte_reads():
    srv = _FakeReadSts(
        {STS_PRESENT_VOLTAGE: [120], STS_PRESENT_TEMPERATURE: [41]}
    )

    voltage, _, _ = srv.ReadVoltage(1)
    temperature, _, _ = srv.ReadTemperature(1)

    assert voltage == pytest.approx(12.0)
    assert temperature == 41
    assert srv.requests == [(STS_PRESENT_VOLTAGE, 1), (STS_PRESENT_TEMPERATURE, 1)]
