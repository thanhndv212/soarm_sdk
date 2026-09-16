"""Unit tests for soarm_sdk.tuning.gains_io, against a fake bus double."""

from __future__ import annotations

import pytest

from soarm_sdk.protocol.packet_handler import PacketResult
from soarm_sdk.protocol.registers import (
    COMM_RX_FAIL,
    COMM_SUCCESS,
    STS_D_COEF,
    STS_I_COEF,
    STS_LOCK,
    STS_P_COEF,
)
from soarm_sdk.tuning.gains_io import Gains, read_gains, write_gains


class _FakeBus:
    """Stand-in for an open `sts` instance's register read/write calls.

    ``read1ByteTxRx`` returns a :class:`PacketResult` (``.data``/``.result``/
    ``.error``) on the real hardware handler, not a plain tuple — this fake
    must match that shape, or a bug in ``gains_io`` that only shows up
    against the real return type would pass here and only surface on real
    hardware (``read_gains`` originally unpacked it as a 3-tuple and crashed
    the first time it ran against an actual servo).
    """

    def __init__(self, registers=None, fail_addr=None):
        self._registers = dict(registers or {})
        self._fail_addr = fail_addr
        self.written = []  # [(servo_id, addr, value)]

    def read1ByteTxRx(self, servo_id, addr):
        if addr == self._fail_addr:
            return PacketResult([], COMM_RX_FAIL, 0)
        return PacketResult([self._registers.get((servo_id, addr), 0)], COMM_SUCCESS, 0)

    def write1ByteTxRx(self, servo_id, addr, value):
        if addr == self._fail_addr:
            return COMM_RX_FAIL, 0
        self.written.append((servo_id, addr, value))
        self._registers[(servo_id, addr)] = value
        return COMM_SUCCESS, 0


@pytest.mark.parametrize("p,d,i", [(0, 0, 0), (32, 32, 0), (254, 254, 254)])
def test_gains_accepts_valid_range(p, d, i):
    Gains(p=p, d=d, i=i)  # must not raise


@pytest.mark.parametrize("field,value", [("p", -1), ("d", 255), ("i", 300)])
def test_gains_rejects_out_of_range(field, value):
    defaults = dict(p=32, d=32, i=0)
    defaults[field] = value
    with pytest.raises(ValueError):
        Gains(**defaults)


def test_read_gains_reads_all_three_registers():
    bus = _FakeBus({(1, STS_P_COEF): 40, (1, STS_D_COEF): 20, (1, STS_I_COEF): 5})
    gains = read_gains(bus, 1)
    assert gains == Gains(p=40, d=20, i=5)


def test_read_gains_raises_on_comm_failure():
    bus = _FakeBus(fail_addr=STS_D_COEF)
    with pytest.raises(IOError):
        read_gains(bus, 1)


def test_write_gains_unlocks_writes_then_locks_in_order():
    bus = _FakeBus()
    write_gains(bus, 1, Gains(p=40, d=20, i=5))
    addrs_in_order = [addr for (_sid, addr, _val) in bus.written]
    assert addrs_in_order == [STS_LOCK, STS_P_COEF, STS_D_COEF, STS_I_COEF, STS_LOCK]
    assert bus.written[0][2] == 0  # unlock
    assert bus.written[-1][2] == 1  # lock
    assert bus._registers[(1, STS_P_COEF)] == 40
    assert bus._registers[(1, STS_D_COEF)] == 20
    assert bus._registers[(1, STS_I_COEF)] == 5


def test_write_gains_relocks_eeprom_even_if_a_gain_write_fails():
    bus = _FakeBus(fail_addr=STS_D_COEF)
    with pytest.raises(RuntimeError):
        write_gains(bus, 1, Gains(p=40, d=20, i=5))
    addrs_written = [addr for (_sid, addr, _val) in bus.written]
    assert addrs_written[0] == STS_LOCK  # unlock happened
    assert addrs_written[-1] == STS_LOCK  # relock happened despite the failure
    assert addrs_written[-1:] == [STS_LOCK]
