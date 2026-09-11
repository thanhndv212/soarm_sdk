"""Unit tests for soarm_sdk.calibration.rom_sweep.

``simulate_rom_sweep`` needs no hardware and is tested directly.
``run_rom_sweep`` is exercised against a fake bus/servo double so the
stall-detection control flow is covered without a real serial port.
"""

from __future__ import annotations

from contextlib import contextmanager

from soarm_sdk.calibration import rom_sweep as rom_sweep_mod
from soarm_sdk.calibration.rom_sweep import run_rom_sweep, simulate_rom_sweep
from soarm_sdk.protocol.registers import COMM_SUCCESS


def test_simulate_rom_sweep_returns_result_per_joint():
    res = simulate_rom_sweep(
        joint_ids=[1, 2],
        sweep_speed=100,
        timeout_s=1.0,
    )
    assert set(res.keys()) == {1, 2}
    for sid, d in res.items():
        assert d["pos_min"] < d["pos_max"]
        assert d["zero"] == (d["pos_min"] + d["pos_max"]) // 2
        assert d["stalled_fwd"] is True
        assert d["stalled_rev"] is True


def test_simulate_rom_sweep_respects_max_range_ticks():
    res = simulate_rom_sweep(
        joint_ids=[1],
        sweep_speed=100,
        timeout_s=1.0,
        max_range_ticks=100,
    )
    assert res[1]["range_ticks"] <= 200  # +/-100 ticks from centre


def test_simulate_rom_sweep_calls_fk_update_fn():
    seen = []
    simulate_rom_sweep(
        joint_ids=[1],
        sweep_speed=200,
        timeout_s=0.2,
        fk_update_fn=lambda positions: seen.append(dict(positions)),
    )
    assert len(seen) > 0
    assert all(1 in snapshot for snapshot in seen)


def test_simulate_rom_sweep_starts_from_current_positions():
    log = []
    simulate_rom_sweep(
        joint_ids=[1],
        sweep_speed=500,
        timeout_s=0.05,
        current_positions={1: 3000},
        log_fn=log.append,
    )
    # Just confirm it runs to completion without needing a live position;
    # the starting tick only affects the first (fast) leg of the sweep.
    assert any("done" in line for line in log)


class _FakeServo:
    """Minimal double for the ``sts`` instance the sweep drives."""

    def __init__(self, pos_min: int, pos_max: int):
        self._pos_min = pos_min
        self._pos_max = pos_max
        self._pos = (pos_min + pos_max) // 2
        self._direction = 0
        self.modes: list[int] = []

    def write1ByteTxRx(self, sid, addr, value):
        return COMM_SUCCESS, 0

    def WheelMode(self, sid):
        pass

    def WriteSpec(self, sid, speed, acc):
        self._direction = 1 if speed > 0 else (-1 if speed < 0 else 0)

    def ReadPos(self, sid):
        # Move toward whichever limit the current sweep direction points at,
        # then hold there — simulating a servo stalled against a hard stop.
        if self._direction > 0:
            self._pos = min(self._pos + 50, self._pos_max)
        elif self._direction < 0:
            self._pos = max(self._pos - 50, self._pos_min)
        return self._pos, COMM_SUCCESS, 0

    def WritePosEx(self, sid, pos, speed, acc):
        self._pos = pos


def test_run_rom_sweep_finds_limits_against_a_fake_bus(monkeypatch):
    # The sweep sleeps 0.08s between polls for real hardware; skip that
    # here so the fake servo's fixed 50-ticks/poll motion can reach both
    # limits inside a short, deterministic timeout.
    monkeypatch.setattr(rom_sweep_mod.time, "sleep", lambda s: None)

    fake = _FakeServo(pos_min=1000, pos_max=3000)

    @contextmanager
    def bus():
        yield fake

    res = run_rom_sweep(
        bus,
        joint_ids=[5],
        sweep_speed=50,
        stall_thr=5,
        stall_win=3,
        timeout_s=2.0,
    )
    assert res[5]["stalled_fwd"] is True
    assert res[5]["stalled_rev"] is True
    assert res[5]["pos_max"] == 3000
    assert res[5]["pos_min"] == 1000
