#!/usr/bin/env python
"""Viser dashboard for soarm_sdk: robot control + real-time 3-D visualisation.

Tabs
----
1. Start Up       -- connect/disconnect, quick torque, scan servos
2. Homing Wizard  -- automatic ROM sweep or manual limit recording
3. PID Tuning     -- read/write P/D/I gains + live step-response chart
4. Command Panel  -- per-joint position / speed / acc commands + sync packet
5. Recorder       -- record & replay demonstration trajectories
6. Monitor        -- live uPlot charts, joint table, health, servo inspector
7. Reconfigure    -- calibration (IDs / limits / mode / baud) + config export/import

Launch
------
    python examples/viser_dashboard.py \\
        [--device /dev/ttyXXX] [--baud 1000000] \\
        [--port 8080] [--urdf PATH] [--interval-ms 200]
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import threading
import time
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Generator, List, Optional

import numpy as np

try:
    import viser
except ImportError as exc:
    raise RuntimeError(
        "viser is required. Install with: pip install soarm-sdk[viser]"
    ) from exc

try:
    import trimesh  # noqa: F401 – imported for yourdfpy side-effects
    import yourdfpy

    _URDF_AVAILABLE = True
except ImportError:
    _URDF_AVAILABLE = False

# ---------------------------------------------------------------------------
# Path setup — allows running without installing the package.
# ---------------------------------------------------------------------------
_src_root = Path(__file__).resolve().parents[1] / "src"
if _src_root.is_dir() and str(_src_root) not in sys.path:
    sys.path.insert(0, str(_src_root))

import importlib as _il  # noqa: E402

from soarm_sdk import (  # noqa: E402
    COMM_SUCCESS,
    STS_ACC,
    STS_BAUD_RATE,
    STS_GOAL_SPEED_L,
    STS_ID,
    STS_LOCK,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_MODE,
    STS_TORQUE_ENABLE,
    PortHandler,
    discover_servos,
    get_available_ports,
    parse_range,
    read_servo_diagnostics,
    run_calibration,
    sts as _Sts,
    write1,
    write2,
)
from soarm_sdk.conversions import ticks_to_radians  # noqa: E402

_def = _il.import_module("soarm_sdk.stservo_def")
STS_OFS_L: int = _def.STS_OFS_L
STS_GOAL_POSITION_L: int = _def.STS_GOAL_POSITION_L
STS_PRESENT_TEMPERATURE: int = _def.STS_PRESENT_TEMPERATURE
STS_STATUS: int = _def.STS_STATUS
STS_PRESENT_CURRENT_L: int = _def.STS_PRESENT_CURRENT_L
STS_PRESENT_POSITION_L: int = _def.STS_PRESENT_POSITION_L
STS_PRESENT_SPEED_L: int = _def.STS_PRESENT_SPEED_L
STS_P_COEF: int = _def.STS_P_COEF
STS_D_COEF: int = _def.STS_D_COEF
STS_I_COEF: int = _def.STS_I_COEF

SOARM100_IDS: List[int] = [1, 2, 3, 4, 5, 6]
SOARM100_JOINT_NAMES: List[str] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
_DEFAULT_URDF = (
    Path(__file__).resolve().parents[2]
    / "SO-ARM100"
    / "Simulation"
    / "SO100"
    / "so100.urdf"
)
_STALL_POLL_S = 0.08
_HEALTH_EVERY = 5  # read temp/current every Nth poll iteration
_CHART_WINDOW = 200  # rolling history length (200 × 100 ms = 20 s)
_CHART_COLORS = [
    "#e74c3c",
    "#3498db",
    "#2ecc71",
    "#f39c12",
    "#9b59b6",
    "#1abc9c",
]

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------


@dataclass
class JointState:
    positions: Dict[int, int] = field(default_factory=dict)
    speeds: Dict[int, int] = field(default_factory=dict)
    temps: Dict[int, Optional[float]] = field(default_factory=dict)
    currents: Dict[int, Optional[float]] = field(default_factory=dict)
    connected: bool = False
    poll_error: Optional[str] = None
    poll_count: int = 0


_lock: threading.Lock = threading.Lock()
_state: JointState = JointState()

# Recording buffer – written by poll loop, read/cleared by recorder tab
_recording_state: Dict[str, Any] = {
    "active": False,
    "data": [],
    "ids": set(SOARM100_IDS),
}

# ---------------------------------------------------------------------------
# Bus helpers
# ---------------------------------------------------------------------------

_bus_lock: threading.Lock = threading.Lock()


@contextmanager
def _bus(device: str, baud: int) -> Generator[_Sts, None, None]:
    """Open the servo bus exclusively, yield an sts instance, close on exit."""
    with _bus_lock:
        ph = PortHandler(device)
        try:
            if not ph.openPort():
                raise RuntimeError(f"Cannot open port {device!r}")
            if not ph.setBaudRate(baud):
                raise RuntimeError(f"Cannot set baud rate {baud}")
            yield _Sts(ph)
        finally:
            ph.closePort()


def _parse_ids(raw: str) -> List[int]:
    ids: set = set()
    for token in [
        s.strip() for s in raw.replace(" ", ",").split(",") if s.strip()
    ]:
        if "-" in token:
            for v in parse_range(token):
                ids.add(int(v))
        else:
            ids.add(int(token, 0))
    return sorted(ids)


# ---------------------------------------------------------------------------
# Polling thread
# ---------------------------------------------------------------------------


def _poll_loop(
    device: str,
    baud: int,
    ids: List[int],
    interval_s: float,
    stop_event: threading.Event,
) -> None:
    """Continuously read joint pos/speed (and periodically temp/current).

    Updates the module-level ``_state`` under ``_lock``.
    Also appends to ``_recording_state["data"]`` when recording is active.
    """
    poll_count = 0
    while not stop_event.is_set():
        t_start = time.monotonic()
        try:
            with _bus(device, baud) as srv:
                gsr = srv.groupSyncRead
                gsr.clearParam()
                for sid in ids:
                    gsr.addParam(sid)
                sync_result = gsr.txRxPacket()

                new_pos: Dict[int, int] = {}
                new_spd: Dict[int, int] = {}
                new_temp: Dict[int, Optional[float]] = {}
                new_curr: Dict[int, Optional[float]] = {}
                now = time.time()

                for sid in ids:
                    if sync_result == COMM_SUCCESS:
                        avail, _ = gsr.isAvailable(
                            sid, STS_PRESENT_POSITION_L, 2
                        )
                        if avail:
                            raw_pos = gsr.getData(
                                sid, STS_PRESENT_POSITION_L, 2
                            )
                            raw_spd = gsr.getData(sid, STS_PRESENT_SPEED_L, 2)
                            new_pos[sid] = srv.sts_tohost(raw_pos, 15)
                            new_spd[sid] = srv.sts_tohost(raw_spd, 15)
                    else:
                        pos, speed, r, _ = srv.ReadPosSpeed(sid)
                        if r == COMM_SUCCESS:
                            new_pos[sid] = pos
                            new_spd[sid] = speed

                    if poll_count % _HEALTH_EVERY == 0:
                        temp, r2, _ = srv.ReadTemperature(sid)
                        curr, r3, _ = srv.ReadCurrent(sid)
                        new_temp[sid] = temp if r2 == COMM_SUCCESS else None
                        new_curr[sid] = curr if r3 == COMM_SUCCESS else None

            with _lock:
                _state.positions.update(new_pos)
                _state.speeds.update(new_spd)
                if new_temp:
                    _state.temps.update(new_temp)
                if new_curr:
                    _state.currents.update(new_curr)
                _state.connected = True
                _state.poll_error = None
                _state.poll_count = poll_count + 1

                if _recording_state["active"]:
                    rec_ids: set = _recording_state["ids"]
                    for sid in new_pos:
                        if sid in rec_ids:
                            _recording_state["data"].append(
                                {
                                    "timestamp": round(now, 4),
                                    "id": sid,
                                    "position": new_pos[sid],
                                    "speed": new_spd.get(sid, 0),
                                }
                            )

            poll_count += 1

        except Exception as exc:
            with _lock:
                _state.connected = False
                _state.poll_error = str(exc)

        elapsed = time.monotonic() - t_start
        remaining = interval_s - elapsed
        if remaining > 0:
            stop_event.wait(timeout=remaining)


# ---------------------------------------------------------------------------
# FK / visualisation helpers
# ---------------------------------------------------------------------------


def _mat3_to_wxyz(R: np.ndarray) -> np.ndarray:
    """Convert a 3×3 rotation matrix to a (w, x, y, z) unit quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])


def _load_urdf_meshes(
    server: viser.ViserServer,
    urdf_path: Path,
) -> tuple:
    """Load the URDF and register all link meshes in the Viser scene.

    Returns ``(urdf_object, mesh_handles)`` where ``mesh_handles`` maps
    scene-graph node name → ``GlbHandle``.  Both are ``None`` / ``{}`` on
    failure so callers can skip FK updates gracefully.
    """
    if not _URDF_AVAILABLE:
        print(
            "[viser_dashboard] yourdfpy/trimesh not installed – no 3-D view."
        )
        return None, {}

    if not urdf_path.exists():
        print(f"[viser_dashboard] URDF not found: {urdf_path}")
        return None, {}

    try:
        urdf = yourdfpy.URDF.load(str(urdf_path), load_meshes=True)
    except Exception as exc:
        print(f"[viser_dashboard] URDF load failed: {exc}")
        return None, {}

    scene = urdf.scene
    mesh_handles: Dict[str, Any] = {}

    for node_name in scene.graph.nodes_geometry:
        T_world, geom_name = scene.graph[node_name]
        geom = scene.geometry.get(geom_name)
        if geom is None:
            continue
        if isinstance(geom, trimesh.Scene):
            geom = geom.dump(concatenate=True)
        if not isinstance(geom, trimesh.Trimesh):
            continue

        wxyz = _mat3_to_wxyz(T_world[:3, :3])
        handle = server.scene.add_mesh_trimesh(
            name=f"/robot/{node_name}",
            mesh=geom,
            wxyz=tuple(wxyz),
            position=tuple(T_world[:3, 3]),
        )
        mesh_handles[node_name] = handle

    return urdf, mesh_handles


def _update_fk(
    urdf: "yourdfpy.URDF",
    positions: Dict[int, int],
    mesh_handles: Dict[str, Any],
) -> None:
    """Recompute FK from joint positions (ticks) and push transforms to Viser."""
    cfg: Dict[str, float] = {}
    for sid, jname in zip(SOARM100_IDS, SOARM100_JOINT_NAMES):
        ticks = positions.get(sid)
        if ticks is not None:
            cfg[jname] = ticks_to_radians(ticks)

    if not cfg:
        return

    urdf.update_cfg(cfg)
    scene = urdf.scene
    for node_name, handle in mesh_handles.items():
        T_world, _ = scene.graph[node_name]
        handle.wxyz = tuple(_mat3_to_wxyz(T_world[:3, :3]))
        handle.position = tuple(T_world[:3, 3])


# ---------------------------------------------------------------------------
# Telemetry / Health formatting helpers
# ---------------------------------------------------------------------------


def _format_telem_md(state: JointState) -> str:
    if not state.connected:
        err = state.poll_error or "no hardware"
        return f"**Status**: Disconnected — *{err}*"

    lines = [
        f"**Poll #{state.poll_count}**\n",
        "| Joint | Name | Pos (ticks) | Pos (rad) | Speed (t/s) |",
        "|-------|------|:-----------:|:---------:|:-----------:|",
    ]
    for sid, jname in zip(SOARM100_IDS, SOARM100_JOINT_NAMES):
        pos = state.positions.get(sid)
        spd = state.speeds.get(sid)
        rad = f"{ticks_to_radians(pos):.3f}" if pos is not None else "—"
        lines.append(
            f"| J{sid} | {jname} | {pos if pos is not None else '—'}"
            f" | {rad} | {spd if spd is not None else '—'} |"
        )
    return "\n".join(lines)


def _format_health_md(state: JointState) -> str:
    if not state.temps and not state.currents:
        return "*Health data: updates every 5 polls…*"

    lines = [
        "| Joint | Temp (°C) | Current (mA) |",
        "|-------|:---------:|:------------:|",
    ]
    for sid in SOARM100_IDS:
        temp = state.temps.get(sid)
        curr = state.currents.get(sid)
        t_str = f"{temp:.1f}" if temp is not None else "—"
        c_str = f"{curr:.0f}" if curr is not None else "—"
        lines.append(f"| J{sid} | {t_str} | {c_str} |")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# ROM sweep helper (no Streamlit dependencies)
# ---------------------------------------------------------------------------


def _run_rom_sweep_auto(
    device: str,
    baud: int,
    joint_ids: List[int],
    sweep_speed: int,
    stall_thr: int,
    stall_win: int,
    timeout_s: float,
    max_range_ticks: int = 0,
    log_fn: Callable[[str], None] = print,
) -> Dict[int, dict]:
    """Drive each joint in wheel mode to discover its mechanical limits.

    Identical logic to the Streamlit version but uses ``log_fn`` for output
    instead of ``st.write()``.
    """
    results: Dict[int, dict] = {}
    n = len(joint_ids)

    with _bus(device, baud) as srv:
        for idx, sid in enumerate(joint_ids):
            log_fn(
                f"**J{sid}** ({idx + 1}/{n}) — enabling torque, entering wheel mode…"
            )
            write1(srv, sid, STS_TORQUE_ENABLE, 1, "torque on")
            write1(srv, sid, STS_LOCK, 0, "unlock EEPROM")
            srv.WheelMode(sid)

            # ── Forward sweep ────────────────────────────────────────────────
            log_fn(
                f"**J{sid}** sweeping → positive limit (speed +{sweep_speed})…"
            )
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
                    if (
                        max_range_ticks > 0
                        and abs(p - _start_fwd) >= max_range_ticks
                    ):
                        break
                if (
                    len(recent) >= stall_win
                    and (max(recent) - min(recent)) <= stall_thr
                ):
                    stalled_fwd = True
                    break
                time.sleep(_STALL_POLL_S)
            srv.WriteSpec(sid, 0, 5)
            time.sleep(0.4)
            log_fn(
                f"**J{sid}** (+) {'stalled' if stalled_fwd else 'timed-out'}"
                f" at **{pos_max}** ticks"
            )

            # ── Reverse sweep ────────────────────────────────────────────────
            log_fn(
                f"**J{sid}** sweeping ← negative limit (speed −{sweep_speed})…"
            )
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
                    if (
                        max_range_ticks > 0
                        and abs(p - _start_rev) >= max_range_ticks
                    ):
                        break
                if (
                    len(recent) >= stall_win
                    and (max(recent) - min(recent)) <= stall_thr
                ):
                    stalled_rev = True
                    break
                time.sleep(_STALL_POLL_S)
            srv.WriteSpec(sid, 0, 5)
            time.sleep(0.4)
            log_fn(
                f"**J{sid}** (−) {'stalled' if stalled_rev else 'timed-out'}"
                f" at **{pos_min}** ticks"
            )

            # ── Restore servo mode and move to midpoint ───────────────────────
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


# Realistic per-joint ROM half-ranges (ticks from centre = 2048).
# Values approximate the physical SO-ARM100 limits; used only in sim mode.
_SIM_ROM_HALF: Dict[int, int] = {
    1: 1380,  # shoulder_pan  ≈ 121°
    2: 1150,  # shoulder_lift ≈ 101°
    3: 1300,  # elbow_flex    ≈ 114°
    4: 1000,  # wrist_flex    ≈  88°
    5: 2000,  # wrist_roll    ≈ 175° (continuous-ish)
    6:  512,  # gripper       ≈  45°
}


def _run_rom_sweep_sim(
    joint_ids: List[int],
    sweep_speed: int,
    timeout_s: float,
    max_range_ticks: int = 0,
    log_fn: Callable[[str], None] = print,
    fk_update_fn: Optional[Callable[[Dict[int, int]], None]] = None,
) -> Dict[int, dict]:
    """Simulate a ROM sweep without any hardware.

    Each joint is assigned a plausible min/max derived from
    ``_SIM_ROM_HALF`` centred on 2048.  When *fk_update_fn* is provided
    the simulated position is streamed to it at ~20 Hz so the 3-D scene
    animates during the sweep.  The sweep is capped to 0.6 s per
    direction regardless of *timeout_s*.
    """
    _FK_STEP_S = 0.05  # 20 Hz FK update rate

    # Seed current positions from shared state so non-swept joints hold still.
    with _lock:
        current: Dict[int, int] = dict(_state.positions)
    for sid in SOARM100_IDS:
        current.setdefault(sid, 2048)

    def _animate(sid: int, start: int, end: int, duration: float) -> None:
        """Interpolate joint *sid* from *start* to *end* over *duration* seconds."""
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

        log_fn(
            f"**J{sid}** ({idx + 1}/{n}) [SIM] enabling torque, entering wheel mode…"
        )
        time.sleep(0.05)

        log_fn(
            f"**J{sid}** [SIM] sweeping → positive limit (speed +{sweep_speed})…"
        )
        _animate(sid, start_tick, pos_max, sim_delay)
        log_fn(f"**J{sid}** (+) [SIM] stalled at **{pos_max}** ticks")

        log_fn(
            f"**J{sid}** [SIM] sweeping ← negative limit (speed −{sweep_speed})…"
        )
        _animate(sid, pos_max, pos_min, sim_delay)
        log_fn(f"**J{sid}** (−) [SIM] stalled at **{pos_min}** ticks")

        # Return to midpoint
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


# ---------------------------------------------------------------------------
# Tab builders
# ---------------------------------------------------------------------------


def _build_tab_startup(
    server: viser.ViserServer,
    device_h: "viser.GuiTextHandle",
    baud_h: "viser.GuiNumberHandle",
    interval_h: "viser.GuiNumberHandle",
    stop_event: threading.Event,
    poll_ref: Dict[str, Any],
    conn_status_md: "viser.GuiMarkdownHandle",
) -> None:
    """Connection, quick torque, and scan-servos in a single Start Up tab."""
    server.gui.add_markdown("## Start Up")

    # ── Connection ────────────────────────────────────────────────────────
    with server.gui.add_folder("Connection"):
        ports = get_available_ports()
        port_info = (
            "\n".join(f"- `{d}` — {desc}" for d, desc in ports)
            if ports
            else "*No USB serial ports detected.*"
        )
        server.gui.add_markdown(port_info)
        server.gui.add_markdown(
            "*Device / baud / interval are shared across all tabs.*"
        )
        connect_btn = server.gui.add_button(
            "Connect & Start Polling", color="green"
        )
        disconnect_btn = server.gui.add_button("Disconnect", color="red")

    # ── Quick Torque ──────────────────────────────────────────────────────
    with server.gui.add_folder("Quick Torque"):
        torque_ids_h = server.gui.add_text("Joint IDs", initial_value="1-6")
        qt_on = server.gui.add_button("Torque ON", color="blue")
        qt_off = server.gui.add_button("Torque OFF", color="red")
        torque_status_md = server.gui.add_markdown("")

    # ── Scan Servos ───────────────────────────────────────────────────────
    with server.gui.add_folder("Scan Servos"):
        server.gui.add_markdown(
            "Ping every ID in the given range and list responding servos."
        )
        scan_range_h = server.gui.add_text(
            "Scan range", initial_value="1-10"
        )
        scan_btn = server.gui.add_button("Scan", color="blue")
        scan_md = server.gui.add_markdown("*Press Scan to discover servos.*")

    # ── Callbacks ─────────────────────────────────────────────────────────
    @connect_btn.on_click
    def _do_connect(_: viser.GuiEvent) -> None:
        dev = device_h.value
        bd = int(baud_h.value)
        ivl = float(interval_h.value) / 1000.0
        poll_ref["interval_s"] = ivl
        stop_event.set()
        old = poll_ref.get("thread")
        if old and old.is_alive():
            old.join(timeout=2.0)
        stop_event.clear()
        t = threading.Thread(
            target=_poll_loop,
            args=(dev, bd, SOARM100_IDS, ivl, stop_event),
            daemon=True,
        )
        t.start()
        poll_ref["thread"] = t
        conn_status_md.content = f"**Polling** `{dev}` @ {bd} baud"

    @disconnect_btn.on_click
    def _do_disconnect(_: viser.GuiEvent) -> None:
        stop_event.set()
        t = poll_ref.get("thread")
        if t:
            t.join(timeout=2.0)
        poll_ref["thread"] = None
        with _lock:
            _state.connected = False
        conn_status_md.content = "*Disconnected.*"

    @qt_on.on_click
    def _do_qt_on(_: viser.GuiEvent) -> None:
        try:
            ids = _parse_ids(torque_ids_h.value)
            with _bus(device_h.value, int(baud_h.value)) as srv:
                for sid in ids:
                    write1(srv, sid, STS_TORQUE_ENABLE, 1, "torque on")
            torque_status_md.content = f"Torque ON — J{ids}"
        except Exception as exc:
            torque_status_md.content = f"**Error**: {exc}"

    @qt_off.on_click
    def _do_qt_off(_: viser.GuiEvent) -> None:
        try:
            ids = _parse_ids(torque_ids_h.value)
            with _bus(device_h.value, int(baud_h.value)) as srv:
                for sid in ids:
                    write1(srv, sid, STS_TORQUE_ENABLE, 0, "torque off")
            torque_status_md.content = f"Torque OFF — J{ids}"
        except Exception as exc:
            torque_status_md.content = f"**Error**: {exc}"

    @scan_btn.on_click
    def _do_scan(_: viser.GuiEvent) -> None:
        device = device_h.value
        baud = int(baud_h.value)
        scan_md.content = "*Scanning…*"
        try:
            ids = _parse_ids(scan_range_h.value)
            found = discover_servos(device, baud, ids)
            if found:
                lines = ["| ID | Status |", "|----|--------|"]
                for sid in sorted(found):
                    lines.append(f"| {sid} | ✓ responding |")
                scan_md.content = "\n".join(lines)
            else:
                scan_md.content = "**No servos found** in the scanned range."
        except Exception as exc:
            scan_md.content = f"**Scan error**: {exc}"


# ---------------------------------------------------------------------------


def _build_tab_homing(
    server: viser.ViserServer,
    device_h: "viser.GuiTextHandle",
    baud_h: "viser.GuiNumberHandle",
    stop_event: threading.Event,
    poll_ref: Dict[str, Any],
    fk_update_fn: Optional[Callable[[Dict[int, int]], None]] = None,
) -> None:
    server.gui.add_markdown("## Homing Wizard")
    server.gui.add_markdown(
        "Calibrate servo midpoints via automatic motor sweep or manual hand movement."
    )

    _OPT_AUTO = "Automatic — motor sweep"
    _OPT_MAN = "Manual — move by hand"

    mode_h = server.gui.add_dropdown(
        "Mode",
        options=[_OPT_AUTO, _OPT_MAN],
        initial_value=_OPT_AUTO,
    )
    joints_h = server.gui.add_text("Joint IDs", initial_value="1-6")
    target_ref_h = server.gui.add_number(
        "Zero reference (ticks)", initial_value=2048, min=0, max=4095, step=1
    )

    # ── Automatic-mode controls ───────────────────────────────────────────
    sim_mode_h = server.gui.add_checkbox(
        "Dry-run (simulate sweep, no hardware required)",
        initial_value=False,
    )
    sweep_params_folder = server.gui.add_folder("Sweep Parameters")
    with sweep_params_folder:
        speed_h = server.gui.add_slider(
            "Sweep speed (ticks/s)", min=30, max=500, step=10, initial_value=150,
        )
        stall_thr_h = server.gui.add_number(
            "Stall threshold (ticks)", initial_value=5, min=1, max=50, step=1
        )
        stall_win_h = server.gui.add_number(
            "Stall window (samples)", initial_value=8, min=3, max=20, step=1
        )
        timeout_h = server.gui.add_number(
            "Timeout/direction (s)", initial_value=30.0, min=5.0, max=120.0, step=1.0,
        )
        max_range_h = server.gui.add_number(
            "Max travel/direction (ticks, 0 = unlimited)",
            initial_value=0, min=0, max=4096, step=50,
        )
    sweep_btn = server.gui.add_button("Sweep All Joints", color="green")

    # ── Manual-mode controls ──────────────────────────────────────────────
    man_header_md = server.gui.add_markdown(
        "Disable torque then move each joint by hand to its limits."
    )
    torque_off_btn = server.gui.add_button("Disable Torque on Selected Joints")
    man_status_md = server.gui.add_markdown("*No manual positions recorded yet.*")
    man_compute_btn = server.gui.add_button(
        "Compute from Recorded Limits", color="blue"
    )
    man_positions: Dict[int, Dict[str, Optional[int]]] = {
        sid: {"min": None, "max": None} for sid in SOARM100_IDS
    }
    man_min_btns: Dict[int, Any] = {}
    man_max_btns: Dict[int, Any] = {}
    for sid in SOARM100_IDS:
        man_min_btns[sid] = server.gui.add_button(f"Record Min J{sid}")
        man_max_btns[sid] = server.gui.add_button(f"Record Max J{sid}")

    # ── Shared apply/save controls ─────────────────────────────────────────
    apply_btn = server.gui.add_button(
        "Apply: Write Offsets + Limits to EEPROM", color="blue"
    )
    save_btn = server.gui.add_button("Save Config (soarm100_rom.json)")
    log_md = server.gui.add_markdown("*Select a mode and proceed.*")

    # ── Mode visibility helper ─────────────────────────────────────────────
    _auto_controls = [
        sim_mode_h, sweep_params_folder, sweep_btn,
    ]
    _man_controls = [
        man_header_md, torque_off_btn, man_status_md, man_compute_btn,
        *man_min_btns.values(), *man_max_btns.values(),
    ]

    def _apply_visibility(selected: str) -> None:
        is_auto = selected == _OPT_AUTO
        for h in _auto_controls:
            h.visible = is_auto
        for h in _man_controls:
            h.visible = not is_auto

    # Set initial state
    _apply_visibility(mode_h.value)

    @mode_h.on_update
    def _on_mode_change(ev: viser.GuiEvent) -> None:
        _apply_visibility(mode_h.value)

    # Shared results store
    rom_results: Dict[int, dict] = {}

    def _stop_poll() -> None:
        stop_event.set()
        t = poll_ref.get("thread")
        if t is not None and t.is_alive():
            t.join(timeout=3.0)

    def _restart_poll() -> None:
        stop_event.clear()
        dev = device_h.value
        bd = int(baud_h.value)
        ivl = poll_ref.get("interval_s", 0.2)
        t = threading.Thread(
            target=_poll_loop,
            args=(dev, bd, SOARM100_IDS, ivl, stop_event),
            daemon=True,
        )
        t.start()
        poll_ref["thread"] = t

    def _update_man_status() -> None:
        lines = ["| Joint | Min | Max |", "|-------|-----|-----|"]
        for sid in SOARM100_IDS:
            mn = man_positions[sid]["min"]
            mx = man_positions[sid]["max"]
            lines.append(
                f"| J{sid} | {mn if mn is not None else '—'}"
                f" | {mx if mx is not None else '—'} |"
            )
        man_status_md.content = "\n".join(lines)

    @sweep_btn.on_click
    def _do_sweep(_: viser.GuiEvent) -> None:
        simulate = sim_mode_h.value
        device = device_h.value
        baud = int(baud_h.value)

        if not simulate and not device:
            log_md.content = (
                "**Error**: No serial device configured. "
                "Enable *Dry-run* to test without hardware."
            )
            return

        try:
            ids = _parse_ids(joints_h.value)
        except Exception:
            ids = list(SOARM100_IDS)

        if not simulate:
            _stop_poll()

        label = "[SIM] Simulating" if simulate else "Sweeping"
        log_md.content = f"{label} joints {ids}…"
        try:
            if simulate:
                res = _run_rom_sweep_sim(
                    joint_ids=ids,
                    sweep_speed=int(speed_h.value),
                    timeout_s=float(timeout_h.value),
                    max_range_ticks=int(max_range_h.value),
                    log_fn=lambda msg: setattr(log_md, "content", msg),
                    fk_update_fn=fk_update_fn,
                )
                method = "auto-sweep (simulated)"
            else:
                res = _run_rom_sweep_auto(
                    device=device,
                    baud=baud,
                    joint_ids=ids,
                    sweep_speed=int(speed_h.value),
                    stall_thr=int(stall_thr_h.value),
                    stall_win=int(stall_win_h.value),
                    timeout_s=float(timeout_h.value),
                    max_range_ticks=int(max_range_h.value),
                    log_fn=lambda msg: setattr(log_md, "content", msg),
                )
                method = "auto-sweep"
            for sid_r, d in res.items():
                d["method"] = method
            rom_results.update(res)
            suffix = " (simulated — Apply will still write to real EEPROM if connected)" if simulate else ""
            log_md.content = f"**Sweep complete{suffix}.** Check results and press Apply."
        except Exception as exc:
            log_md.content = f"**Sweep error**: {exc}"
        finally:
            if not simulate:
                _restart_poll()

    @apply_btn.on_click
    def _do_apply(_: viser.GuiEvent) -> None:
        if not rom_results:
            log_md.content = "**Error**: No results yet — run a sweep or manual recording first."
            return
        device = device_h.value
        baud = int(baud_h.value)
        target_ref = int(target_ref_h.value)
        try:
            with _bus(device, baud) as srv:
                for sid_a, d in rom_results.items():
                    off = target_ref - d["zero"]
                    reg = (abs(off) | 0x0800) if off < 0 else abs(off)
                    lmin = max(0, min(4095, d["pos_min"] + off))
                    lmax = max(0, min(4095, d["pos_max"] + off))
                    write1(srv, sid_a, STS_LOCK, 0, "unlock")
                    write2(srv, sid_a, STS_OFS_L, reg, f"offset J{sid_a}")
                    write2(
                        srv,
                        sid_a,
                        STS_MIN_ANGLE_LIMIT_L,
                        lmin,
                        f"min J{sid_a}",
                    )
                    write2(
                        srv,
                        sid_a,
                        STS_MAX_ANGLE_LIMIT_L,
                        lmax,
                        f"max J{sid_a}",
                    )
                    write1(srv, sid_a, STS_LOCK, 1, "lock")
            log_md.content = (
                "**EEPROM updated.** Offsets and limits written to all joints."
            )
        except Exception as exc:
            log_md.content = f"**Apply error**: {exc}"

    @save_btn.on_click
    def _do_save(_: viser.GuiEvent) -> None:
        if not rom_results:
            log_md.content = "**Error**: No results to save."
            return
        target_ref = int(target_ref_h.value)
        device = device_h.value
        cfg_data: Dict[str, Any] = {
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "device": device,
            "target_ref": target_ref,
            "joints": {},
        }
        for sid_s, d in sorted(rom_results.items()):
            off = target_ref - d["zero"]
            cfg_data["joints"][str(sid_s)] = {
                "pos_min": d["pos_min"],
                "pos_max": d["pos_max"],
                "zero_midpoint": d["zero"],
                "range_ticks": d["range_ticks"],
                "method": d.get("method", "—"),
                "offset_signed": off,
                "offset_register": (
                    (abs(off) | 0x0800) if off < 0 else abs(off)
                ),
                "limit_min": max(0, min(4095, d["pos_min"] + off)),
                "limit_max": max(0, min(4095, d["pos_max"] + off)),
            }
        save_path = Path("soarm100_rom.json")
        save_path.write_text(json.dumps(cfg_data, indent=2))
        log_md.content = f"**Saved**: `{save_path.resolve()}`"

    @torque_off_btn.on_click
    def _do_torque_off(_: viser.GuiEvent) -> None:
        device = device_h.value
        baud = int(baud_h.value)
        try:
            ids = _parse_ids(joints_h.value)
        except Exception:
            ids = list(SOARM100_IDS)
        try:
            with _bus(device, baud) as srv:
                for sid_t in ids:
                    write1(srv, sid_t, STS_TORQUE_ENABLE, 0, "torque off")
            man_status_md.content = (
                f"Torque OFF on J{ids}. Move joints by hand."
            )
        except Exception as exc:
            man_status_md.content = f"**Error**: {exc}"

    def _make_man_handlers(sid_cap: int) -> None:
        @man_min_btns[sid_cap].on_click
        def _(_: viser.GuiEvent) -> None:
            device = device_h.value
            baud = int(baud_h.value)
            try:
                with _bus(device, baud) as srv:
                    p, r, _ = srv.ReadPos(sid_cap)
                if r == COMM_SUCCESS:
                    man_positions[sid_cap]["min"] = p
                    _update_man_status()
            except Exception as exc:
                man_status_md.content = f"**Error**: {exc}"

        @man_max_btns[sid_cap].on_click
        def _(_: viser.GuiEvent) -> None:
            device = device_h.value
            baud = int(baud_h.value)
            try:
                with _bus(device, baud) as srv:
                    p, r, _ = srv.ReadPos(sid_cap)
                if r == COMM_SUCCESS:
                    man_positions[sid_cap]["max"] = p
                    _update_man_status()
            except Exception as exc:
                man_status_md.content = f"**Error**: {exc}"

    for sid in SOARM100_IDS:
        _make_man_handlers(sid)

    @man_compute_btn.on_click
    def _do_man_compute(_: viser.GuiEvent) -> None:
        computed = []
        for sid_c in SOARM100_IDS:
            mn = man_positions[sid_c]["min"]
            mx = man_positions[sid_c]["max"]
            if mn is None or mx is None:
                continue
            if mn > mx:
                mn, mx = mx, mn
            zero = (mn + mx) // 2
            rom_results[sid_c] = {
                "pos_min": mn,
                "pos_max": mx,
                "zero": zero,
                "range_ticks": mx - mn,
                "method": "manual",
            }
            computed.append(sid_c)
        if computed:
            log_md.content = f"Computed from manual limits for J{computed}. Press Apply to write."
        else:
            log_md.content = (
                "**Error**: Record both min and max for at least one joint."
            )


# ---------------------------------------------------------------------------


def _build_tab_command(
    server: viser.ViserServer,
    device_h: "viser.GuiTextHandle",
    baud_h: "viser.GuiNumberHandle",
) -> None:
    server.gui.add_markdown("## Command Panel")
    server.gui.add_markdown(
        "Send position or velocity commands to individual joints or all joints "
        "simultaneously via a single sync packet."
    )

    mode_h = server.gui.add_dropdown(
        "Control mode",
        options=["Servo (position)", "Wheel (speed)"],
        initial_value="Servo (position)",
    )
    refresh_btn = server.gui.add_button("Refresh Current Positions")
    status_md = server.gui.add_markdown(
        "*Press Refresh to load positions from hardware.*"
    )

    pos_handles: Dict[int, Any] = {}
    spd_handles: Dict[int, Any] = {}
    acc_handles: Dict[int, Any] = {}
    send_btns: Dict[int, Any] = {}

    for sid in SOARM100_IDS:
        with server.gui.add_folder(
            f"J{sid} — {SOARM100_JOINT_NAMES[sid - 1]}"
        ):
            pos_handles[sid] = server.gui.add_slider(
                "Position (ticks)", min=0, max=4095, step=1, initial_value=2048
            )
            spd_handles[sid] = server.gui.add_number(
                "Speed (ticks/s)", initial_value=300, min=0, max=3000, step=10
            )
            acc_handles[sid] = server.gui.add_number(
                "Acc", initial_value=50, min=0, max=254, step=1
            )
            send_btns[sid] = server.gui.add_button(
                f"Send J{sid}", color="blue"
            )

    send_all_btn = server.gui.add_button(
        "Send All (Sync Packet)", color="green"
    )
    torque_on_btn = server.gui.add_button("Torque ON all")
    torque_off_btn = server.gui.add_button("Torque OFF all")

    @refresh_btn.on_click
    def _do_refresh(_: viser.GuiEvent) -> None:
        device = device_h.value
        baud = int(baud_h.value)
        try:
            with _bus(device, baud) as srv:
                lines = []
                for sid in SOARM100_IDS:
                    p, r, _ = srv.ReadPos(sid)
                    if r == COMM_SUCCESS:
                        pos_handles[sid].value = int(p)
                        lines.append(f"J{sid}={p}")
            status_md.content = "Loaded: " + ", ".join(lines)
        except Exception as exc:
            status_md.content = f"**Error**: {exc}"

    def _make_send_handler(sid_cap: int) -> None:
        @send_btns[sid_cap].on_click
        def _(_: viser.GuiEvent) -> None:
            device = device_h.value
            baud = int(baud_h.value)
            is_servo = mode_h.value.startswith("Servo")
            try:
                with _bus(device, baud) as srv:
                    if is_servo:
                        pos = int(pos_handles[sid_cap].value)
                        spd = int(spd_handles[sid_cap].value)
                        acc = int(acc_handles[sid_cap].value)
                        srv.WritePosEx(sid_cap, pos, spd, acc)
                        status_md.content = (
                            f"J{sid_cap} → pos={pos}, spd={spd}, acc={acc}"
                        )
                    else:
                        spd = int(spd_handles[sid_cap].value)
                        acc = int(acc_handles[sid_cap].value)
                        srv.WheelMode(sid_cap)
                        srv.WriteSpec(sid_cap, spd, acc)
                        status_md.content = (
                            f"J{sid_cap} wheel spd={spd}, acc={acc}"
                        )
            except Exception as exc:
                status_md.content = f"**Error**: {exc}"

    for sid in SOARM100_IDS:
        _make_send_handler(sid)

    @send_all_btn.on_click
    def _do_send_all(_: viser.GuiEvent) -> None:
        device = device_h.value
        baud = int(baud_h.value)
        is_servo = mode_h.value.startswith("Servo")
        try:
            with _bus(device, baud) as srv:
                srv.groupSyncWrite.clearParam()
                for sid in SOARM100_IDS:
                    pos = int(pos_handles[sid].value)
                    spd = int(spd_handles[sid].value)
                    acc = int(acc_handles[sid].value)
                    if is_servo:
                        srv.SyncWritePosEx(sid, pos, spd, acc)
                    else:
                        srv.WheelMode(sid)
                        srv.WriteSpec(sid, spd, acc)
                if is_servo:
                    srv.groupSyncWrite.txPacket()
                    srv.groupSyncWrite.clearParam()
            status_md.content = "Sync packet sent to all joints."
        except Exception as exc:
            status_md.content = f"**Error**: {exc}"

    @torque_on_btn.on_click
    def _do_ton(_: viser.GuiEvent) -> None:
        try:
            with _bus(device_h.value, int(baud_h.value)) as srv:
                for sid in SOARM100_IDS:
                    write1(srv, sid, STS_TORQUE_ENABLE, 1, "torque on")
            status_md.content = "Torque ON — all joints."
        except Exception as exc:
            status_md.content = f"**Error**: {exc}"

    @torque_off_btn.on_click
    def _do_toff(_: viser.GuiEvent) -> None:
        try:
            with _bus(device_h.value, int(baud_h.value)) as srv:
                for sid in SOARM100_IDS:
                    write1(srv, sid, STS_TORQUE_ENABLE, 0, "torque off")
            status_md.content = "Torque OFF — all joints."
        except Exception as exc:
            status_md.content = f"**Error**: {exc}"


# ---------------------------------------------------------------------------


def _build_tab_reconfigure(
    server: viser.ViserServer,
    device_h: "viser.GuiTextHandle",
    baud_h: "viser.GuiNumberHandle",
) -> None:
    server.gui.add_markdown("## Reconfigure")

    # ── Calibration ───────────────────────────────────────────────────────
    with server.gui.add_folder("Calibration"):
     server.gui.add_markdown(
        "Configure servo IDs, angle limits, acceleration, speed, "
        "mode, torque, and baud rate in a single run."
     )

     scan_range_h = server.gui.add_text("Scan range", initial_value="1-10")
     unlock_h = server.gui.add_checkbox(
         "Unlock EEPROM before changes", initial_value=False
     )
     lock_h = server.gui.add_checkbox(
         "Lock EEPROM after changes", initial_value=True
     )

     with server.gui.add_folder("ID Remapping (OLD:NEW per line)"):
         assign_h = server.gui.add_text(
             "Assign IDs", initial_value="", multiline=True, hint="1:11\n2:12"
         )
     with server.gui.add_folder("Angle Limits (ID:MIN:MAX per line)"):
         angle_h = server.gui.add_text(
             "Angle limits",
             initial_value="",
             multiline=True,
             hint="1:512:3584",
         )
     with server.gui.add_folder("Motion Parameters"):
         acc_h = server.gui.add_text(
             "Acceleration (ID:ACC)", initial_value="", multiline=True
         )
         speed_h = server.gui.add_text(
             "Speed (ID:SPEED)", initial_value="", multiline=True
         )
     with server.gui.add_folder("Mode & Torque"):
         torque_h = server.gui.add_text(
             "Torque (ID:on/off)", initial_value="", multiline=True
         )
         mode_h = server.gui.add_text(
             "Mode (ID:0/1  0=servo, 1=wheel)", initial_value="", multiline=True
         )
         baud_map_h = server.gui.add_text(
             "Baud code (ID:CODE)", initial_value="", multiline=True
         )

     run_btn = server.gui.add_button("Run Calibration", color="green")
     cal_log_md = server.gui.add_markdown(
         "*Fill in the fields above and press Run.*"
     )

    def _parse_text_lines(raw: str) -> List[str]:
        return [s.strip() for s in raw.splitlines() if s.strip()]

    @run_btn.on_click
    def _do_cal(_: viser.GuiEvent) -> None:
        import argparse as _ap

        device = device_h.value
        baud = int(baud_h.value)
        log_buf: List[str] = []
        args = _ap.Namespace(
            list_ports=False,
            ui=False,
            device=device or "/dev/ttyUSB0",
            baud=baud,
            scan_range=scan_range_h.value or "1-10",
            assign_id=_parse_text_lines(assign_h.value),
            angle_limit=_parse_text_lines(angle_h.value),
            set_acc=_parse_text_lines(acc_h.value),
            set_speed=_parse_text_lines(speed_h.value),
            torque=_parse_text_lines(torque_h.value),
            set_mode=_parse_text_lines(mode_h.value),
            set_baud=_parse_text_lines(baud_map_h.value),
            unlock=unlock_h.value,
            lock=lock_h.value,
        )
        try:
            exit_code = run_calibration(args, log=log_buf.append)
        except Exception as exc:
            log_buf.append(f"Error: {exc}")
            exit_code = 1

        status = "✓ Complete" if exit_code == 0 else "✗ Non-zero exit"
        cal_log_md.content = (
            f"**{status}**\n```\n" + "\n".join(log_buf) + "\n```"
        )

    # ── Config export / import ────────────────────────────────────────────
    with server.gui.add_folder("Config Export / Import"):
        server.gui.add_markdown(
            "Save and restore the full register state of soarm100 servos as JSON."
        )
        with server.gui.add_folder("Export"):
            export_ids_h = server.gui.add_text(
                "IDs to export", initial_value="1-6"
            )
            export_path_h = server.gui.add_text(
                "Output file", initial_value="soarm100_config.json"
            )
            export_btn = server.gui.add_button("Read & Export", color="green")

        with server.gui.add_folder("Import"):
            import_path_h = server.gui.add_text(
                "JSON file to apply", initial_value="soarm100_config.json"
            )
            import_btn = server.gui.add_button(
                "Apply Config from File", color="blue"
            )

        cfg_log_md = server.gui.add_markdown("*Use Export or Import above.*")

    @export_btn.on_click
    def _do_export(_: viser.GuiEvent) -> None:
        device = device_h.value
        baud = int(baud_h.value)
        try:
            ids = _parse_ids(export_ids_h.value)
            diag = read_servo_diagnostics(device, baud, ids)
            snapshot: Dict[str, Any] = {
                "exported_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "device": device,
                "servos": {},
            }
            for sid_e, data in diag.items():
                entry = {
                    k: v for k, v in (data or {}).items() if k != "_errors"
                }
                snapshot["servos"][str(sid_e)] = entry
            out_path = Path(export_path_h.value)
            out_path.write_text(json.dumps(snapshot, indent=2))
            cfg_log_md.content = (
                f"**Exported** {len(diag)} servo(s) → `{out_path.resolve()}`"
            )
        except Exception as exc:
            cfg_log_md.content = f"**Export error**: {exc}"

    @import_btn.on_click
    def _do_import(_: viser.GuiEvent) -> None:
        device = device_h.value
        baud = int(baud_h.value)
        try:
            snap = json.loads(Path(import_path_h.value).read_text())
            servos = snap.get("servos", {})
            log_lines: List[str] = []
            with _bus(device, baud) as srv:
                for sid_str, entry in servos.items():
                    sid_i = int(sid_str)
                    write1(srv, sid_i, STS_LOCK, 0, "unlock")
                    acc_val = entry.get("Acceleration")
                    if acc_val is not None:
                        write1(srv, sid_i, STS_ACC, int(acc_val), "acc")
                        log_lines.append(f"J{sid_i}: ACC={acc_val}")
                    mode_val = entry.get("Mode")
                    if mode_val is not None:
                        write1(srv, sid_i, STS_MODE, int(mode_val), "mode")
                        log_lines.append(f"J{sid_i}: MODE={mode_val}")
                    write1(srv, sid_i, STS_LOCK, 1, "lock")
            log_lines.append("Done.")
            cfg_log_md.content = (
                "**Import complete.**\n```\n" + "\n".join(log_lines) + "\n```"
            )
        except Exception as exc:
            cfg_log_md.content = f"**Import error**: {exc}"


# ---------------------------------------------------------------------------


def _build_tab_monitor(
    server: viser.ViserServer,
    device_h: "viser.GuiTextHandle",
    baud_h: "viser.GuiNumberHandle",
    chart_window: int = _CHART_WINDOW,
) -> "tuple[viser.GuiMarkdownHandle, Any, Any, viser.GuiMarkdownHandle]":
    """Combined Monitor tab: live charts + table, health, and servo inspector."""
    import viser.uplot as _uplot  # local import — optional dep

    # ── Live charts ───────────────────────────────────────────────────────
    server.gui.add_markdown("## Monitor")

    with server.gui.add_folder("Live Position & Speed"):
        _init_t = np.linspace(0.0, chart_window * 0.1, chart_window)
        _zeros = np.zeros(chart_window)

        pos_chart = server.gui.add_uplot(
            data=(_init_t, *[_zeros.copy() for _ in SOARM100_IDS]),
            series=(
                _uplot.Series(label="time"),
                *[
                    _uplot.Series(
                        label=f"J{sid}", stroke=_CHART_COLORS[i], width=2
                    )
                    for i, sid in enumerate(SOARM100_IDS)
                ],
            ),
            title="Position (ticks)",
            axes=(
                _uplot.Axis(label="time (s)"),
                _uplot.Axis(label="ticks", side=3),
            ),
            legend=_uplot.Legend(show=True),
            aspect=2.5,
        )
        spd_chart = server.gui.add_uplot(
            data=(_init_t, *[_zeros.copy() for _ in SOARM100_IDS]),
            series=(
                _uplot.Series(label="time"),
                *[
                    _uplot.Series(
                        label=f"J{sid}", stroke=_CHART_COLORS[i], width=2
                    )
                    for i, sid in enumerate(SOARM100_IDS)
                ],
            ),
            title="Speed (ticks/s)",
            axes=(
                _uplot.Axis(label="time (s)"),
                _uplot.Axis(label="ticks/s", side=3),
            ),
            legend=_uplot.Legend(show=True),
            aspect=2.5,
        )
        telem_md = server.gui.add_markdown("*Waiting for hardware data…*")

    # ── Health ────────────────────────────────────────────────────────────
    with server.gui.add_folder("Health (temp / current)"):
        server.gui.add_markdown(
            "Sampled every 5 poll cycles (~1 s at 200 ms interval)."
        )
        health_md = server.gui.add_markdown("*Waiting for hardware data…*")

    # ── Inspector ─────────────────────────────────────────────────────────
    with server.gui.add_folder("Servo Inspector"):
        server.gui.add_markdown(
            "Full register snapshot: position, speed, load, voltage, "
            "current, temperature, mode, acceleration, correction, status."
        )
        sid_h = server.gui.add_number(
            "Servo ID", initial_value=1, min=1, max=253, step=1
        )
        read_btn = server.gui.add_button("Read Registers", color="blue")
        insp_html = server.gui.add_html(
            "<i>Select a servo ID and press Read.</i>"
        )

    def _diag_to_html(sid: int, details: Dict[str, Any]) -> str:
        errors: Dict[str, Any] = (
            details.pop("_errors", {}) if "_errors" in details else {}
        )
        rows = ""
        for label, value in details.items():
            err_flag = " ⚠" if label in errors else ""
            val_str = "—" if value is None else str(value)
            bg = " style='background:#fff3cd'" if label in errors else ""
            rows += (
                f"<tr{bg}><td style='padding:3px 8px;font-weight:600'>"
                f"{label}{err_flag}</td>"
                f"<td style='padding:3px 8px'>{val_str}</td></tr>"
            )
        return (
            f"<b>Servo J{sid}</b>"
            f"<table style='font-size:12px;width:100%;border-collapse:collapse'>"
            f"<tr style='background:#eee'>"
            f"<th style='padding:3px 8px;text-align:left'>Register</th>"
            f"<th style='padding:3px 8px;text-align:left'>Value</th></tr>"
            f"{rows}</table>"
        )

    @read_btn.on_click
    def _do_read(_: viser.GuiEvent) -> None:
        device = device_h.value
        baud = int(baud_h.value)
        sid = int(sid_h.value)
        insp_html.content = f"<i>Reading J{sid}…</i>"
        try:
            diag = read_servo_diagnostics(device, baud, [sid])
            details = dict(diag.get(sid, {}))
            if not details:
                insp_html.content = f"<b>No response from J{sid}.</b>"
                return
            insp_html.content = _diag_to_html(sid, details)
        except Exception as exc:
            insp_html.content = (
                f"<b style='color:red'>Error reading J{sid}: {exc}</b>"
            )

    return telem_md, pos_chart, spd_chart, health_md


# ---------------------------------------------------------------------------


def _build_tab_recorder(
    server: viser.ViserServer,
    device_h: "viser.GuiTextHandle",
    baud_h: "viser.GuiNumberHandle",
) -> None:
    server.gui.add_markdown("## Position Recorder & Replayer")
    server.gui.add_markdown(
        "Record demonstration trajectories from soarm100 joints and replay them."
    )

    ids_h = server.gui.add_text("Joint IDs to record", initial_value="1-6")
    rec_btn = server.gui.add_button("Start Recording", color="green")
    stop_rec_btn = server.gui.add_button(
        "Stop & Save", color="red", disabled=True
    )
    status_md = server.gui.add_markdown("*Not recording.*")

    server.gui.add_markdown("---\n**Replay**")
    replay_path_h = server.gui.add_text(
        "CSV path", initial_value="trajectory.csv"
    )
    speed_scale_h = server.gui.add_slider(
        "Speed scale", min=0.1, max=5.0, step=0.1, initial_value=1.0
    )
    loop_h = server.gui.add_number(
        "Loops", initial_value=1, min=1, max=20, step=1
    )
    replay_btn = server.gui.add_button("Replay Trajectory", color="blue")

    @rec_btn.on_click
    def _start_rec(_: viser.GuiEvent) -> None:
        try:
            rec_ids = set(_parse_ids(ids_h.value))
        except Exception:
            rec_ids = set(SOARM100_IDS)
        with _lock:
            _recording_state["active"] = True
            _recording_state["data"] = []
            _recording_state["ids"] = rec_ids
        rec_btn.disabled = True
        stop_rec_btn.disabled = False
        status_md.content = f"**Recording…** (J{sorted(rec_ids)})"

    @stop_rec_btn.on_click
    def _stop_rec(_: viser.GuiEvent) -> None:
        with _lock:
            _recording_state["active"] = False
            data = list(_recording_state["data"])
            _recording_state["data"] = []
        rec_btn.disabled = False
        stop_rec_btn.disabled = True
        if data:
            ts = time.strftime("%Y%m%d_%H%M%S")
            save_path = Path(f"trajectory_{ts}.csv")
            with save_path.open("w", newline="") as f:
                writer = csv.DictWriter(
                    f, fieldnames=["timestamp", "id", "position", "speed"]
                )
                writer.writeheader()
                writer.writerows(data)
            status_md.content = (
                f"**Saved** {len(data)} samples → `{save_path.resolve()}`"
            )
        else:
            status_md.content = "Stopped. No data recorded."

    @replay_btn.on_click
    def _do_replay(_: viser.GuiEvent) -> None:
        device = device_h.value
        baud = int(baud_h.value)
        csv_path = Path(replay_path_h.value)
        if not csv_path.exists():
            status_md.content = f"**Error**: File not found: `{csv_path}`"
            return
        try:
            with csv_path.open() as f:
                rows = list(csv.DictReader(f))
        except Exception as exc:
            status_md.content = f"**Error reading CSV**: {exc}"
            return

        speed_scale = float(speed_scale_h.value)
        loops = int(loop_h.value)

        # Group rows by timestamp
        groups: List[List[dict]] = []
        current_group: List[dict] = []
        last_t: Optional[float] = None
        for row in sorted(rows, key=lambda r: float(r["timestamp"])):
            t = float(row["timestamp"])
            if last_t is None or abs(t - last_t) < 0.005:
                current_group.append(row)
            else:
                if current_group:
                    groups.append(current_group)
                current_group = [row]
            last_t = t
        if current_group:
            groups.append(current_group)

        status_md.content = f"Replaying {len(rows)} samples × {loops} loop(s)…"
        try:
            with _bus(device, baud) as srv:
                for _ in range(loops):
                    prev_t: Optional[float] = None
                    for group in groups:
                        gt = float(group[0]["timestamp"])
                        if prev_t is not None:
                            dt = max(0.0, (gt - prev_t) / speed_scale)
                            time.sleep(dt)
                        prev_t = gt
                        srv.groupSyncWrite.clearParam()
                        for row in group:
                            sid = int(row["id"])
                            pos = int(row["position"])
                            raw_spd = abs(float(row.get("speed", 300)))
                            spd = max(0, min(3000, int(raw_spd * speed_scale)))
                            srv.SyncWritePosEx(sid, pos, spd, 50)
                        srv.groupSyncWrite.txPacket()
                        srv.groupSyncWrite.clearParam()
            status_md.content = "**Replay complete.**"
        except Exception as exc:
            status_md.content = f"**Replay error**: {exc}"


# ---------------------------------------------------------------------------

_STEP_POLL_S = 0.05  # 20 Hz position polling during step response


def _compute_step_metrics(
    t: List[float],
    actual: List[float],
    start: float,
    target: float,
) -> str:
    """Return a Markdown table of standard step-response metrics."""
    if len(actual) < 3:
        return "*Not enough samples for metrics.*"

    arr = np.array(actual, dtype=np.float64)
    t_arr = np.array(t, dtype=np.float64) - t[0]

    step_size = target - start
    if abs(step_size) < 1:
        return "*Step too small for metrics (< 1 tick).*"

    direction = 1.0 if step_size > 0 else -1.0

    # Steady-state: mean of last 20 % of samples
    ss_n = max(1, len(arr) // 5)
    ss_val = float(np.mean(arr[-ss_n:]))
    ss_error = abs(ss_val - target)
    ss_error_pct = ss_error / abs(step_size) * 100.0

    # Overshoot: peak excursion beyond target in the step direction
    if direction > 0:
        peak_val = float(np.max(arr))
        overshoot_ticks = max(0.0, peak_val - target)
        peak_idx = int(np.argmax(arr))
    else:
        peak_val = float(np.min(arr))
        overshoot_ticks = max(0.0, target - peak_val)
        peak_idx = int(np.argmin(arr))
    overshoot_pct = overshoot_ticks / abs(step_size) * 100.0
    t_peak = float(t_arr[peak_idx])

    # Rise time: 10 % → 90 % of step amplitude
    lo = start + 0.10 * step_size
    hi = start + 0.90 * step_size
    if direction > 0:
        idx_lo = np.where(arr >= lo)[0]
        idx_hi = np.where(arr >= hi)[0]
    else:
        idx_lo = np.where(arr <= lo)[0]
        idx_hi = np.where(arr <= hi)[0]
    t_rise = (
        float(t_arr[idx_hi[0]]) - float(t_arr[idx_lo[0]])
        if len(idx_lo) and len(idx_hi)
        else float("nan")
    )

    # Settling time: last time |pos − target| > 2 % of step amplitude
    band = 0.02 * abs(step_size)
    outside = np.where(np.abs(arr - target) > band)[0]
    t_settle = float(t_arr[outside[-1]]) if len(outside) else 0.0

    def _f(v: float, unit: str = "", d: int = 2) -> str:
        return "—" if np.isnan(v) else f"{v:.{d}f}{unit}"

    rows = [
        (
            "Steady-state error",
            f"{_f(ss_error, ' ticks', 1)} ({_f(ss_error_pct, ' %', 1)})",
        ),
        (
            "Overshoot",
            f"{_f(overshoot_ticks, ' ticks', 1)} ({_f(overshoot_pct, ' %', 1)})",
        ),
        ("Peak time", _f(t_peak, " s")),
        ("Rise time (10→90 %)", _f(t_rise, " s")),
        ("Settling time (±2 %)", _f(t_settle, " s")),
        ("Samples collected", str(len(arr))),
    ]
    lines = ["| Metric | Value |", "|--------|-------|"] + [
        f"| {lbl} | {val} |" for lbl, val in rows
    ]
    return "\n".join(lines)


def _build_tab_pid(
    server: viser.ViserServer,
    device_h: "viser.GuiTextHandle",
    baud_h: "viser.GuiNumberHandle",
) -> None:
    """PID gain tuning for STS3215 servos with live step-response chart."""
    import viser.uplot as _uplot

    server.gui.add_markdown("## PID Gain Tuning")
    server.gui.add_markdown(
        "Read and write the **P / D / I** coefficients stored in EEPROM.  \n"
        "Typical defaults: P=32, D=32, I=0.  \n"
        "⚠ Writing modifies EEPROM — changes persist after power-off."
    )

    servo_id_h = server.gui.add_number(
        "Servo ID", initial_value=1, min=1, max=253, step=1
    )

    with server.gui.add_folder("Current gains (read from hardware)"):
        read_p_md = server.gui.add_markdown("P: —")
        read_d_md = server.gui.add_markdown("D: —")
        read_i_md = server.gui.add_markdown("I: —")

    read_btn = server.gui.add_button("Read Gains", color="blue")

    with server.gui.add_folder("New gains to write"):
        new_p_h = server.gui.add_number(
            "P (proportional)", initial_value=32, min=0, max=254, step=1
        )
        new_d_h = server.gui.add_number(
            "D (derivative)", initial_value=32, min=0, max=254, step=1
        )
        new_i_h = server.gui.add_number(
            "I (integral)", initial_value=0, min=0, max=254, step=1
        )

    write_pid_btn = server.gui.add_button("Write Gains to EEPROM", color="red")
    pid_log_md = server.gui.add_markdown("*Select a servo and press Read.*")

    # ── Step Response ──────────────────────────────────────────────────────────────
    with server.gui.add_folder("Step Response"):
        server.gui.add_markdown(
            "Command a position step and record the 20 Hz response.  \n"
            "Set the target within the servo’s homed range."
        )
        step_target_h = server.gui.add_slider(
            "Step target (ticks)", min=0, max=4095, step=1, initial_value=2048
        )
        step_speed_h = server.gui.add_number(
            "Speed (ticks/s)", initial_value=500, min=50, max=4000, step=50
        )
        step_acc_h = server.gui.add_number(
            "Acceleration", initial_value=50, min=0, max=254, step=1
        )
        step_dur_h = server.gui.add_number(
            "Duration (s)", initial_value=3.0, min=0.5, max=10.0, step=0.5
        )
        step_btn = server.gui.add_button("Send Step", color="green")
        step_stop_btn = server.gui.add_button(
            "Stop", color="red", disabled=True
        )
        step_status_md = server.gui.add_markdown(
            "*Configure above and press Send Step.*"
        )

        # Flat placeholder so the chart renders before the first step
        _init_t = np.linspace(0.0, 3.0, 60, dtype=np.float64)
        _init_z = np.zeros(60, dtype=np.float64)
        step_chart = server.gui.add_uplot(
            data=(_init_t, _init_z.copy(), _init_z.copy()),
            series=(
                _uplot.Series(label="time"),
                _uplot.Series(label="Reference", stroke="#e74c3c", width=2),
                _uplot.Series(label="Actual", stroke="#3498db", width=2),
            ),
            title="Step Response",
            axes=(
                _uplot.Axis(label="time (s)"),
                _uplot.Axis(label="position (ticks)", side=3),
            ),
            legend=_uplot.Legend(show=True),
            aspect=3.0,
        )
        metrics_md = server.gui.add_markdown("")

    # Shared stop-event slot; replaced on each new step run
    _step_stop: Dict[str, Any] = {"event": threading.Event()}

    # ── Callbacks ───────────────────────────────────────────────────────────────

    @read_btn.on_click
    def _do_read(_: viser.GuiEvent) -> None:
        device = device_h.value
        baud = int(baud_h.value)
        sid = int(servo_id_h.value)
        try:
            with _bus(device, baud) as srv:

                def _read1(addr: int, label: str) -> int:
                    val, result, _ = srv.read1ByteTxRx(sid, addr)
                    if result != COMM_SUCCESS:
                        raise IOError(f"Read {label} failed (result={result})")
                    return val

                p_val = _read1(STS_P_COEF, "P")
                d_val = _read1(STS_D_COEF, "D")
                i_val = _read1(STS_I_COEF, "I")

            read_p_md.content = f"**P:** {p_val}"
            read_d_md.content = f"**D:** {d_val}"
            read_i_md.content = f"**I:** {i_val}"
            new_p_h.value = float(p_val)  # type: ignore[assignment]
            new_d_h.value = float(d_val)  # type: ignore[assignment]
            new_i_h.value = float(i_val)  # type: ignore[assignment]
            pid_log_md.content = (
                f"**Read OK** — J{sid}: P={p_val}  D={d_val}  I={i_val}"
            )
        except Exception as exc:
            pid_log_md.content = f"**Read error (J{sid}):** {exc}"

    @write_pid_btn.on_click
    def _do_write(_: viser.GuiEvent) -> None:
        device = device_h.value
        baud = int(baud_h.value)
        sid = int(servo_id_h.value)
        p = int(new_p_h.value)
        d = int(new_d_h.value)
        i = int(new_i_h.value)
        try:
            with _bus(device, baud) as srv:
                write1(srv, sid, STS_LOCK, 0, "unlock EEPROM")
                write1(srv, sid, STS_P_COEF, p, "P gain")
                write1(srv, sid, STS_D_COEF, d, "D gain")
                write1(srv, sid, STS_I_COEF, i, "I gain")
                write1(srv, sid, STS_LOCK, 1, "lock EEPROM")
            read_p_md.content = f"**P:** {p}"
            read_d_md.content = f"**D:** {d}"
            read_i_md.content = f"**I:** {i}"
            pid_log_md.content = (
                f"**Write OK** — J{sid}: P={p}  D={d}  I={i} (saved to EEPROM)"
            )
        except Exception as exc:
            pid_log_md.content = f"**Write error (J{sid}):** {exc}"

    @step_btn.on_click
    def _do_step(_: viser.GuiEvent) -> None:
        device = device_h.value
        baud = int(baud_h.value)
        sid = int(servo_id_h.value)
        target = int(step_target_h.value)
        speed = int(step_speed_h.value)
        acc = int(step_acc_h.value)
        duration = float(step_dur_h.value)

        stop_ev = threading.Event()
        _step_stop["event"] = stop_ev
        step_btn.disabled = True
        step_stop_btn.disabled = False
        step_status_md.content = (
            f"*Running step → J{sid} target={target} ticks…*"
        )
        metrics_md.content = ""

        def _run() -> None:
            t_samples: List[float] = []
            actual_samples: List[float] = []
            start_pos: Optional[float] = None
            try:
                with _bus(device, baud) as srv:
                    # Snapshot current position before the step
                    p0, r0, _ = srv.ReadPos(sid)
                    start_pos = float(p0) if r0 == COMM_SUCCESS else None

                    # Ensure torque on, send step command
                    write1(srv, sid, STS_TORQUE_ENABLE, 1, "torque on")
                    srv.WritePosEx(sid, target, speed, acc)
                    t0 = time.monotonic()

                    while not stop_ev.is_set():
                        t_loop = time.monotonic()
                        now = t_loop - t0

                        p, r, _ = srv.ReadPos(sid)
                        if r == COMM_SUCCESS:
                            t_samples.append(now)
                            actual_samples.append(float(p))

                        # Push live chart update every sample
                        if len(t_samples) >= 2:
                            t_np = np.array(t_samples, dtype=np.float64)
                            a_np = np.array(actual_samples, dtype=np.float64)
                            r_np = np.full_like(t_np, float(target))
                            step_chart.data = (t_np, r_np, a_np)

                        if now >= duration:
                            break

                        # Pace to _STEP_POLL_S; wait() also serves as abort check
                        sleep_s = _STEP_POLL_S - (time.monotonic() - t_loop)
                        if sleep_s > 0:
                            stop_ev.wait(timeout=sleep_s)

            except Exception as exc:
                step_status_md.content = f"**Step error**: {exc}"
                step_btn.disabled = False
                step_stop_btn.disabled = True
                return

            # Final chart push (in case last sample wasn't pushed)
            n = len(t_samples)
            if n >= 2:
                t_np = np.array(t_samples, dtype=np.float64)
                a_np = np.array(actual_samples, dtype=np.float64)
                r_np = np.full_like(t_np, float(target))
                step_chart.data = (t_np, r_np, a_np)

            # Compute and display metrics
            if start_pos is not None and n >= 3:
                metrics_md.content = _compute_step_metrics(
                    t_samples, actual_samples, start_pos, float(target)
                )
                actual_hz = n / max(t_samples[-1], 1e-3)
                step_status_md.content = (
                    f"**Done** — J{sid}: {int(start_pos)}→{target} ticks, "
                    f"{n} samples @ ~{actual_hz:.0f} Hz"
                )
            else:
                step_status_md.content = (
                    "*Step complete — insufficient samples for metrics.*"
                )

            step_btn.disabled = False
            step_stop_btn.disabled = True

        threading.Thread(target=_run, daemon=True).start()

    @step_stop_btn.on_click
    def _do_stop(_: viser.GuiEvent) -> None:
        _step_stop["event"].set()
        step_stop_btn.disabled = True


# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Viser dashboard for soarm_sdk"
    )
    parser.add_argument("--device", default="", help="Serial device path")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument(
        "--port", type=int, default=8080, help="Viser HTTP port"
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=_DEFAULT_URDF,
        help="URDF file for 3-D visualisation",
    )
    parser.add_argument("--interval-ms", type=int, default=200)
    args = parser.parse_args()

    # Auto-detect serial device if not specified
    device = args.device
    if not device:
        ports = get_available_ports()
        if ports:
            device = ports[0][0]
            print(f"[viser_dashboard] Auto-selected device: {device}")

    # ── Viser server ──────────────────────────────────────────────────────
    server = viser.ViserServer(port=args.port)
    server.scene.world_axes.visible = True

    # ── Load URDF meshes into 3-D scene ───────────────────────────────────
    urdf, mesh_handles = _load_urdf_meshes(server, args.urdf)
    if urdf is not None:
        print(
            f"[viser_dashboard] URDF loaded: {len(mesh_handles)} mesh node(s)."
        )
    else:
        print("[viser_dashboard] Running without 3-D visualisation.")

    # ── Polling thread reference ──────────────────────────────────────────
    stop_event = threading.Event()
    poll_ref: Dict[str, Any] = {
        "thread": None,
        "interval_s": args.interval_ms / 1000.0,
    }

    # ── Sidebar: shared settings only ────────────────────────────────────
    server.gui.add_markdown("# soarm_sdk Dashboard")
    server.gui.add_markdown("---")

    device_h = server.gui.add_text("Serial device", initial_value=device)
    baud_h = server.gui.add_number(
        "Baud rate",
        initial_value=float(args.baud),
        min=9600,
        max=4_000_000,
        step=1000,
    )
    interval_h = server.gui.add_number(
        "Poll interval (ms)",
        initial_value=float(args.interval_ms),
        min=50,
        max=2000,
        step=50,
    )
    conn_status_md = server.gui.add_markdown("*Not connected.*")

    # ── Tab layout ────────────────────────────────────────────────────────
    tab_group = server.gui.add_tab_group()

    # Build FK callback for sim sweep animation
    if urdf is not None and mesh_handles:
        def _fk_cb(positions: Dict[str, Any]) -> None:
            try:
                _update_fk(urdf, positions, mesh_handles)
            except Exception:
                pass
    else:
        _fk_cb = None  # type: ignore[assignment]

    with tab_group.add_tab("Start Up"):
        _build_tab_startup(
            server, device_h, baud_h, interval_h,
            stop_event, poll_ref, conn_status_md,
        )

    with tab_group.add_tab("Homing Wizard"):
        _build_tab_homing(server, device_h, baud_h, stop_event, poll_ref, fk_update_fn=_fk_cb)

    with tab_group.add_tab("PID Tuning"):
        _build_tab_pid(server, device_h, baud_h)

    with tab_group.add_tab("Command Panel"):
        _build_tab_command(server, device_h, baud_h)

    with tab_group.add_tab("Recorder"):
        _build_tab_recorder(server, device_h, baud_h)

    with tab_group.add_tab("Monitor"):
        telem_md, pos_chart, spd_chart, health_md = _build_tab_monitor(
            server, device_h, baud_h
        )

    with tab_group.add_tab("Reconfigure"):
        _build_tab_reconfigure(server, device_h, baud_h)

    print(
        f"[viser_dashboard] Open http://localhost:{args.port} in your browser."
    )

    # ── Rolling chart buffers ─────────────────────────────────────────────
    _t0 = time.monotonic()
    _t_buf: deque = deque(
        [float(i) * 0.1 for i in range(_CHART_WINDOW)], maxlen=_CHART_WINDOW
    )
    _pos_bufs: Dict[int, deque] = {
        sid: deque([0.0] * _CHART_WINDOW, maxlen=_CHART_WINDOW)
        for sid in SOARM100_IDS
    }
    _spd_bufs: Dict[int, deque] = {
        sid: deque([0.0] * _CHART_WINDOW, maxlen=_CHART_WINDOW)
        for sid in SOARM100_IDS
    }

    # ── Display / FK update loop ──────────────────────────────────────────
    # The main thread acts as the display refresh loop: snapshots shared state
    # under _lock, updates Viser markdown handles, and re-runs FK every
    # fk_every iterations (FK is more expensive so we run it less often).
    fk_every = 3  # FK update every ~300 ms at 100 ms sleep
    loop_count = 0
    try:
        while True:
            with _lock:
                snap = JointState(
                    positions=dict(_state.positions),
                    speeds=dict(_state.speeds),
                    temps=dict(_state.temps),
                    currents=dict(_state.currents),
                    connected=_state.connected,
                    poll_error=_state.poll_error,
                    poll_count=_state.poll_count,
                )

            telem_md.content = _format_telem_md(snap)
            health_md.content = _format_health_md(snap)

            # ── Update rolling chart buffers ──────────────────────────────
            now = time.monotonic() - _t0
            _t_buf.append(now)
            for sid in SOARM100_IDS:
                _pos_bufs[sid].append(float(snap.positions.get(sid) or 0))
                _spd_bufs[sid].append(float(snap.speeds.get(sid) or 0))
            t_arr = np.array(_t_buf, dtype=np.float64)
            pos_chart.data = (
                t_arr,
                *[
                    np.array(_pos_bufs[sid], dtype=np.float64)
                    for sid in SOARM100_IDS
                ],
            )
            spd_chart.data = (
                t_arr,
                *[
                    np.array(_spd_bufs[sid], dtype=np.float64)
                    for sid in SOARM100_IDS
                ],
            )

            if (
                urdf is not None
                and mesh_handles
                and snap.positions
                and loop_count % fk_every == 0
            ):
                try:
                    _update_fk(urdf, snap.positions, mesh_handles)
                except Exception:
                    pass  # FK errors must not crash the display loop

            loop_count += 1
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[viser_dashboard] Shutting down.")
        stop_event.set()


if __name__ == "__main__":
    main()
