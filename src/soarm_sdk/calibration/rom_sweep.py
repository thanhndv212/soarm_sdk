"""Range-of-motion sweep: drive each joint to its mechanical limits.

This used to live entirely inside the Homing Wizard viser panel
(``dashboard/panels/setup.py``), which meant the actual measurement
logic — stall detection, timeout handling, the simulated fallback — was
untestable without a browser and unusable by anything that isn't that
panel. It is the same measurement :func:`~soarm_sdk.calibration.frame.
seed_from_travel` needs as input, so it belongs next to it.

The dashboard panel now calls into :func:`run_rom_sweep` /
:func:`simulate_rom_sweep` and only owns the GUI wiring (buttons, sliders,
markdown status) around them.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Any, Callable, ContextManager, Dict, List, Optional

from ..bus.discovery import write1
from ..protocol.registers import STS_LOCK, STS_MODE, STS_TORQUE_ENABLE, COMM_SUCCESS

__all__ = ["run_rom_sweep", "simulate_rom_sweep"]

_STALL_POLL_S = 0.08

# Realistic per-joint ROM half-ranges (ticks from centre = 2048), used only
# by simulate_rom_sweep's dry-run fallback.
_SIM_ROM_HALF: Dict[int, int] = {
    1: 1380,  # shoulder_pan  ~ 121 deg
    2: 1150,  # shoulder_lift ~ 101 deg
    3: 1300,  # elbow_flex    ~ 114 deg
    4: 1000,  # wrist_flex    ~  88 deg
    5: 2000,  # wrist_roll    ~ 175 deg (continuous-ish)
    6: 512,  # gripper       ~  45 deg
}


def run_rom_sweep(
    bus: Callable[[], ContextManager[Any]],
    joint_ids: List[int],
    sweep_speed: int,
    stall_thr: int,
    stall_win: int,
    timeout_s: float,
    max_range_ticks: int = 0,
    log_fn: Callable[[str], None] = print,
) -> Dict[int, dict]:
    """Drive each joint in wheel mode to discover its mechanical limits.

    Parameters
    ----------
    bus
        A zero-argument callable returning a context manager that yields
        an ``sts`` instance on the open bus — e.g. ``DashboardContext.bus``
        or ``functools.partial(sts_bus, device, baud)``. Kept generic so
        this has no dependency on the dashboard.
    """
    results: Dict[int, dict] = {}
    n = len(joint_ids)

    with bus() as srv:
        for idx, sid in enumerate(joint_ids):
            log_fn(
                f"**J{sid}** ({idx + 1}/{n}) — enabling torque, entering wheel mode…"
            )
            write1(srv, sid, STS_TORQUE_ENABLE, 1, "torque on")
            write1(srv, sid, STS_LOCK, 0, "unlock EEPROM")
            srv.WheelMode(sid)

            log_fn(f"**J{sid}** sweeping → positive limit (speed +{sweep_speed})…")
            srv.WriteSpec(sid, sweep_speed, 5)
            recent: deque = deque(maxlen=stall_win)
            pos_max = 0
            stalled_fwd = False
            _start_fwd, _r0, _ = srv.ReadPos(sid)
            if _r0 != COMM_SUCCESS:
                _start_fwd = 0
            _min_travel = max(stall_thr * 4, 30)
            t0 = time.time()
            while time.time() - t0 < timeout_s:
                p, r, _ = srv.ReadPos(sid)
                if r == COMM_SUCCESS:
                    if p > pos_max:
                        pos_max = p
                    if abs(p - _start_fwd) >= _min_travel:
                        recent.append(p)
                    if max_range_ticks > 0 and abs(p - _start_fwd) >= max_range_ticks:
                        break
                if len(recent) >= stall_win and (max(recent) - min(recent)) <= stall_thr:
                    stalled_fwd = True
                    break
                time.sleep(_STALL_POLL_S)
            srv.WriteSpec(sid, 0, 5)
            time.sleep(0.4)
            log_fn(
                f"**J{sid}** (+) {'stalled' if stalled_fwd else 'timed-out'}"
                f" at **{pos_max}** ticks"
            )

            log_fn(f"**J{sid}** sweeping ← negative limit (speed −{sweep_speed})…")
            recent.clear()
            srv.WriteSpec(sid, -sweep_speed, 5)
            pos_min = 4095
            stalled_rev = False
            _start_rev, _r1, _ = srv.ReadPos(sid)
            if _r1 != COMM_SUCCESS:
                _start_rev = pos_max
            t0 = time.time()
            while time.time() - t0 < timeout_s:
                p, r, _ = srv.ReadPos(sid)
                if r == COMM_SUCCESS:
                    if p < pos_min:
                        pos_min = p
                    if abs(p - _start_rev) >= _min_travel:
                        recent.append(p)
                    if max_range_ticks > 0 and abs(p - _start_rev) >= max_range_ticks:
                        break
                if len(recent) >= stall_win and (max(recent) - min(recent)) <= stall_thr:
                    stalled_rev = True
                    break
                time.sleep(_STALL_POLL_S)
            srv.WriteSpec(sid, 0, 5)
            time.sleep(0.4)
            log_fn(
                f"**J{sid}** (−) {'stalled' if stalled_rev else 'timed-out'}"
                f" at **{pos_min}** ticks"
            )

            write1(srv, sid, STS_MODE, 0, "restore servo mode")
            zero = (pos_min + pos_max) // 2
            srv.WritePosEx(sid, zero, sweep_speed, 20)
            time.sleep(0.3)

            results[sid] = {
                "pos_min": pos_min,
                "pos_max": pos_max,
                "zero": zero,
                "range_ticks": pos_max - pos_min,
                "stalled_fwd": stalled_fwd,
                "stalled_rev": stalled_rev,
            }
            log_fn(
                f"**J{sid}** done — min={pos_min}, max={pos_max},"
                f" zero={zero}, range={pos_max - pos_min} ticks"
            )

        for sid in joint_ids:
            write1(srv, sid, STS_LOCK, 1, "lock")

    return results


def simulate_rom_sweep(
    joint_ids: List[int],
    sweep_speed: int,
    timeout_s: float,
    max_range_ticks: int = 0,
    log_fn: Callable[[str], None] = print,
    fk_update_fn: Optional[Callable[[Dict[int, int]], None]] = None,
    current_positions: Optional[Dict[int, int]] = None,
) -> Dict[int, dict]:
    """Simulate a ROM sweep without any hardware.

    Each joint is assigned a plausible min/max derived from
    ``_SIM_ROM_HALF`` centred on 2048. When *fk_update_fn* is given, the
    simulated position is streamed to it at ~20 Hz so a 3-D scene can
    animate during the sweep.
    """
    _FK_STEP_S = 0.05  # 20 Hz FK update rate

    current: Dict[int, int] = dict(current_positions or {})
    for sid in joint_ids:
        current.setdefault(sid, 2048)

    def _animate(sid: int, start: int, end: int, duration: float) -> None:
        steps = max(1, int(duration / _FK_STEP_S))
        for i in range(steps + 1):
            frac = i / steps
            current[sid] = int(start + frac * (end - start))
            if fk_update_fn is not None:
                fk_update_fn(dict(current))
            if i < steps:
                time.sleep(_FK_STEP_S)

    results: Dict[int, dict] = {}
    n = len(joint_ids)
    sim_delay = min(0.6, timeout_s / 4.0)

    for idx, sid in enumerate(joint_ids):
        half = _SIM_ROM_HALF.get(sid, 1024)
        if max_range_ticks > 0:
            half = min(half, max_range_ticks)
        pos_min = max(0, 2048 - half)
        pos_max = min(4095, 2048 + half)
        start_tick = current.get(sid, 2048)
        zero = (pos_min + pos_max) // 2

        log_fn(f"**J{sid}** ({idx + 1}/{n}) [SIM] enabling torque, entering wheel mode…")
        time.sleep(0.05)

        log_fn(f"**J{sid}** [SIM] sweeping → positive limit (speed +{sweep_speed})…")
        _animate(sid, start_tick, pos_max, sim_delay)
        log_fn(f"**J{sid}** (+) [SIM] stalled at **{pos_max}** ticks")

        log_fn(f"**J{sid}** [SIM] sweeping ← negative limit (speed −{sweep_speed})…")
        _animate(sid, pos_max, pos_min, sim_delay)
        log_fn(f"**J{sid}** (−) [SIM] stalled at **{pos_min}** ticks")

        _animate(sid, pos_min, zero, sim_delay * 0.5)
        current[sid] = zero

        results[sid] = {
            "pos_min": pos_min,
            "pos_max": pos_max,
            "zero": zero,
            "range_ticks": pos_max - pos_min,
            "stalled_fwd": True,
            "stalled_rev": True,
        }
        log_fn(
            f"**J{sid}** [SIM] done — min={pos_min}, max={pos_max},"
            f" zero={zero}, range={pos_max - pos_min} ticks"
        )

    return results
