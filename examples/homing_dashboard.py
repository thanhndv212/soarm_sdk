#!/usr/bin/env python
"""Streamlit dashboard for STServo homing, calibration, and servo development.

Tabs
----
1. Homing Wizard  -- automatic (motor sweep) and manual (hand movement) ROM calibration
2. Command Panel  -- send position/velocity commands to joints (servo & wheel)
3. Calibration    -- scan, assign IDs, set limits/acc/speed/mode/torque/baud
4. Inspector+Telem -- fetch telemetry snapshots and real-time streaming charts
5. Recorder       -- record demonstration trajectories and replay them
6. Health Monitor -- continuous thermal/current/overload monitor
7. Config         -- export/import register snapshots; workspace validation sweep
8. Visual Servoing -- live camera feed with ArUco marker detection
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import sqlite3
import sys
import time
from collections import deque
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Deque, Dict, Generator, List, Optional, Tuple

import pandas as pd

try:  # pragma: no cover
    import plotly.graph_objects as go  # type: ignore[import]
    _PLOTLY_AVAILABLE = True
except ImportError:
    _PLOTLY_AVAILABLE = False

try:  # pragma: no cover
    import cv2  # type: ignore[import]
    import numpy as _np  # used only in visual servoing tab
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

try:  # pragma: no cover - optional dependency import guard
    import streamlit as st  # type: ignore[import]
except ImportError as exc:  # pragma: no cover
    raise RuntimeError(
        "Streamlit is required for the calibration dashboard. "
        "Install it with 'pip install streamlit'."
    ) from exc

# ---------------------------------------------------------------------------
# Path setup -- same guard as homing_calibrate.py
# ---------------------------------------------------------------------------
_src_root = Path(__file__).resolve().parents[1] / "src"
if _src_root.is_dir() and str(_src_root) not in sys.path:
    sys.path.insert(0, str(_src_root))

from homing_calibrate import (  # noqa: E402
    COMM_SUCCESS,
    STS_ACC,
    STS_BAUD_RATE,
    STS_GOAL_SPEED_L,
    STS_ID,
    STS_LOCK,
    STS_MIN_ANGLE_LIMIT_L,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MODE,
    STS_TORQUE_ENABLE,
    discover_servos,
    get_available_ports,
    parse_range,
    read_servo_diagnostics,
    run_calibration,
    write1,
    write2,
)

import importlib as _il  # noqa: E402

_def = _il.import_module("stservo_sdk.stservo_def")
from stservo_sdk import GroupSyncRead, PortHandler, sts as _Sts  # noqa: E402

# Additional register addresses not re-exported by homing_calibrate
STS_OFS_L: int = _def.STS_OFS_L
STS_GOAL_POSITION_L: int = _def.STS_GOAL_POSITION_L
STS_PRESENT_TEMPERATURE: int = _def.STS_PRESENT_TEMPERATURE
STS_STATUS: int = _def.STS_STATUS
STS_PRESENT_CURRENT_L: int = _def.STS_PRESENT_CURRENT_L
STS_PRESENT_POSITION_L: int = _def.STS_PRESENT_POSITION_L
STS_PRESENT_SPEED_L: int = _def.STS_PRESENT_SPEED_L

SOARM100_IDS = [1, 2, 3, 4, 5, 6]
_HIST = 500  # max samples kept in streaming history
_CONFIGS_DIR = Path(__file__).resolve().parents[2] / "configs"

# ---------------------------------------------------------------------------
# SQLite telemetry log helpers
# ---------------------------------------------------------------------------
_DB_SCHEMA = """
CREATE TABLE IF NOT EXISTS telemetry (
    ts          REAL NOT NULL,
    servo_id    INTEGER NOT NULL,
    position    INTEGER,
    speed       INTEGER,
    temperature INTEGER,
    current     REAL,
    voltage     REAL,
    status      INTEGER
);
CREATE INDEX IF NOT EXISTS idx_ts ON telemetry(ts);
CREATE INDEX IF NOT EXISTS idx_sid ON telemetry(servo_id);
"""


def _db_init(path: str) -> sqlite3.Connection:
    """Open (or create) the telemetry database and ensure the schema exists."""
    conn = sqlite3.connect(path, check_same_thread=False)
    conn.executescript(_DB_SCHEMA)
    conn.commit()
    return conn


def _db_insert(conn: sqlite3.Connection, rows: List[dict]) -> None:
    """Batch-insert telemetry rows into the database."""
    if not rows:
        return
    conn.executemany(
        "INSERT INTO telemetry (ts, servo_id, position, speed, temperature, current, voltage, status) "
        "VALUES (:ts, :servo_id, :position, :speed, :temperature, :current, :voltage, :status)",
        rows,
    )
    conn.commit()


def _db_query(
    conn: sqlite3.Connection,
    servo_ids: Optional[List[int]] = None,
    since_ts: float = 0.0,
    limit: int = 2000,
) -> "pd.DataFrame":
    """Query recent telemetry rows and return a DataFrame."""
    conds = ["ts >= ?"]
    params: List[Any] = [since_ts]
    if servo_ids:
        placeholders = ",".join("?" * len(servo_ids))
        conds.append(f"servo_id IN ({placeholders})")
        params.extend(servo_ids)
    where = " AND ".join(conds)
    sql = f"SELECT ts, servo_id, position, speed, temperature, current, voltage, status FROM telemetry WHERE {where} ORDER BY ts DESC LIMIT ?"
    params.append(limit)
    return pd.read_sql_query(sql, conn, params=params)

METRIC_ORDER = [
    "Baudrate",
    "Load",
    "Voltage",
    "Current",
    "Temperature",
    "Acceleration",
    "Mode",
    "Correction",
    "Is Moving",
    "Position",
    "Speed",
    "Status",
]

# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------


@contextmanager
def _bus(device: str, baud: int) -> Generator[_Sts, None, None]:
    """Open the servo bus, yield an sts instance, close on exit."""
    ph = PortHandler(device)
    try:
        if not ph.openPort():
            raise RuntimeError(f"Cannot open port {device!r}")
        if not ph.setBaudRate(baud):
            raise RuntimeError(f"Cannot set baud rate {baud}")
        yield _Sts(ph)
    finally:
        ph.closePort()


def _parse_lines(raw: str) -> List[str]:
    return [s.strip() for s in raw.splitlines() if s.strip()]


def _parse_ids(raw: str) -> List[int]:
    ids: set = set()
    for token in [s.strip() for s in raw.replace(" ", ",").split(",") if s.strip()]:
        if "-" in token:
            for v in parse_range(token):
                ids.add(int(v))
        else:
            ids.add(int(token, 0))
    return sorted(ids)


def _fmt_metric(label: str, val: Any) -> str:
    if val is None:
        return "--"
    try:
        if label == "Baudrate":
            return f"{int(val)} bps"
        if label == "Load":
            return f"{float(val):.1f} %"
        if label == "Voltage":
            return f"{float(val):.2f} V"
        if label == "Current":
            return f"{float(val):.1f} mA"
        if label == "Temperature":
            return f"{float(val):.1f} C"
        if label == "Status":
            return f"0x{int(val):02X}"
        if label == "Is Moving":
            return "Yes" if val else "No"
        return str(val)
    except Exception:
        return str(val)


def _history_to_df(history: "Deque", value_key: str) -> "Optional[pd.DataFrame]":
    """Pivot stream history deque into a wide DataFrame indexed by elapsed time."""
    if not history:
        return None
    rows = list(history)
    t0 = rows[0]["t"]
    data: Dict[str, Dict] = {}
    for row in rows:
        col = f"J{row['id']}"
        t = round(row["t"] - t0, 3)
        data.setdefault(col, {})[t] = row.get(value_key)
    data = {k: v for k, v in data.items() if any(x is not None for x in v.values())}
    if not data:
        return None
    df = pd.DataFrame(data)
    df.index.name = "t (s)"
    return df


def _build_args(**kwargs: Any) -> argparse.Namespace:
    return argparse.Namespace(list_ports=False, ui=False, **kwargs)


def _preview_trajectory(data: List[dict]) -> None:
    """Render an overlaid Plotly time-series of all joint positions from *data*."""
    if not data:
        st.info("No trajectory data to preview.")
        return

    try:
        rows = sorted(data, key=lambda r: float(r["timestamp"]))
        t0 = float(rows[0]["timestamp"])
        by_joint: Dict[int, Dict[str, list]] = {}
        for row in rows:
            sid = int(row["id"])
            t = round(float(row["timestamp"]) - t0, 4)
            by_joint.setdefault(sid, {"t": [], "pos": []})
            by_joint[sid]["t"].append(t)
            by_joint[sid]["pos"].append(int(row["position"]))

        duration = float(rows[-1]["timestamp"]) - t0
        n_frames = len(set(r["timestamp"] for r in rows))

        st.caption(
            f"Duration: **{duration:.2f} s** at 1× | **{n_frames}** time steps | "
            f"**{len(by_joint)}** joints"
        )

        if _PLOTLY_AVAILABLE:
            fig = go.Figure()
            for sid, series in sorted(by_joint.items()):
                fig.add_trace(
                    go.Scatter(
                        x=series["t"],
                        y=series["pos"],
                        mode="lines",
                        name=f"J{sid}",
                    )
                )
            fig.update_layout(
                xaxis_title="Time (s)",
                yaxis_title="Position (ticks)",
                legend_title="Joint",
                height=350,
                margin=dict(l=40, r=20, t=20, b=40),
            )
            st.plotly_chart(fig, width="stretch")
        else:
            # Fallback to Streamlit native chart
            df_preview = pd.DataFrame(
                {
                    f"J{sid}": dict(zip(s["t"], s["pos"]))
                    for sid, s in sorted(by_joint.items())
                }
            )
            df_preview.index.name = "t (s)"
            st.line_chart(df_preview)
    except Exception as exc:
        st.warning(f"Preview failed: {exc}")


# ---------------------------------------------------------------------------
# Session state initialisation
# ---------------------------------------------------------------------------


def _init() -> None:
    ss = st.session_state
    ss.setdefault("ports", get_available_ports())
    ss.setdefault("servo_names", {})
    ss.setdefault("servo_diagnostics", {})
    ss.setdefault("inspector_ids", "1-6")
    ss.setdefault("inspector_ids_pending", None)
    ss.setdefault("detected_servo_ids", [])
    # Streaming
    ss.setdefault("stream_ids_raw", "1-6")
    ss.setdefault("stream_interval_ms", 200)
    ss.setdefault("stream_history", deque(maxlen=_HIST))
    ss.setdefault("telemetry_active", False)
    ss.setdefault("poll_error", None)
    ss.setdefault("poll_count", 0)
    # Recording
    ss.setdefault("recording", False)
    ss.setdefault("recording_data", [])
    ss.setdefault("replay_data", [])
    # Health
    ss.setdefault("health_active", False)
    ss.setdefault("health_history", deque(maxlen=_HIST))
    ss.setdefault("health_temp_thresh", 70)
    ss.setdefault("health_curr_thresh", 1500.0)
    ss.setdefault("health_alerts", deque(maxlen=200))
    # Homing wizard
    ss.setdefault("wizard_step", 0)
    ss.setdefault("wizard_joint_ids", list(SOARM100_IDS))
    ss.setdefault("wizard_raw_positions", {})
    ss.setdefault("wizard_target_ref", 2048)
    ss.setdefault("wizard_computed_offsets", {})
    ss.setdefault("wizard_angle_limits", {sid: (512, 3584) for sid in SOARM100_IDS})
    # ROM Discovery / Manual homing
    ss.setdefault("rom_results", {})
    ss.setdefault("man_results", {})
    ss.setdefault("rom_config_path", str(_CONFIGS_DIR / "soarm100_rom.json"))
    # Command Panel — cached hardware state
    ss.setdefault("cmd_limits", {})    # {sid: (min_ticks, max_ticks)}
    ss.setdefault("cmd_curr_pos", {})  # {sid: ticks}
    # Telemetry DB logging
    ss.setdefault("log_to_db", False)
    ss.setdefault("db_path", str(Path.home() / ".stservo_telemetry.db"))
    ss.setdefault("_db_conn", None)  # sqlite3.Connection or None


# ---------------------------------------------------------------------------
# Background poll -- called once per rerun when any active mode is on
# ---------------------------------------------------------------------------


def _maybe_poll(device: str, baud: int) -> None:
    ss = st.session_state
    if not (
        ss.get("telemetry_active")
        or ss.get("recording")
        or ss.get("health_active")
    ):
        return
    if not device:
        return

    try:
        ids = _parse_ids(ss.get("stream_ids_raw", "1-6"))
    except Exception:
        ids = list(SOARM100_IDS)

    now = time.time()
    poll_count: int = ss.get("poll_count", 0)

    try:
        with _bus(device, baud) as srv:
            # --- GroupSyncRead: one packet reads pos+speed for all IDs ---
            gsr = srv.groupSyncRead  # GroupSyncRead(STS_PRESENT_POSITION_L, 4)
            gsr.clearParam()
            for sid in ids:
                gsr.addParam(sid)
            sync_result = gsr.txRxPacket()

            for sid in ids:
                if sync_result == COMM_SUCCESS:
                    avail, _ = gsr.isAvailable(sid, STS_PRESENT_POSITION_L, 2)
                    if avail:
                        raw_pos = gsr.getData(sid, STS_PRESENT_POSITION_L, 2)
                        raw_spd = gsr.getData(sid, STS_PRESENT_SPEED_L, 2)
                        pos = srv.sts_tohost(raw_pos, 15)
                        speed = srv.sts_tohost(raw_spd, 15)
                    else:
                        continue
                else:
                    # Fallback to individual read if sync failed
                    pos, speed, r, _ = srv.ReadPosSpeed(sid)
                    if r != COMM_SUCCESS:
                        continue

                entry = {"t": now, "id": sid, "pos": pos, "speed": speed}
                ss["stream_history"].append(entry)

                # Enqueue for DB logging
                if ss.get("log_to_db"):
                    ss.setdefault("_db_pending", []).append(
                        {
                            "ts": now,
                            "servo_id": sid,
                            "position": pos,
                            "speed": speed,
                            "temperature": None,
                            "current": None,
                            "voltage": None,
                            "status": None,
                        }
                    )

                if ss.get("recording"):
                    ss["recording_data"].append(
                        {
                            "timestamp": round(now, 4),
                            "id": sid,
                            "position": pos,
                            "speed": speed,
                        }
                    )

                # Health reads every 5th poll (coarser cadence)
                if ss.get("health_active") and poll_count % 5 == 0:
                    temp, r2, _ = srv.ReadTemperature(sid)
                    curr, r3, _ = srv.ReadCurrent(sid)
                    status, r4, _ = srv.ReadStatus(sid)

                    ss["health_history"].append(
                        {
                            "t": now,
                            "id": sid,
                            "temp": temp if r2 == COMM_SUCCESS else None,
                            "current": curr if r3 == COMM_SUCCESS else None,
                            "status": status if r4 == COMM_SUCCESS else None,
                        }
                    )

                    alerts = ss["health_alerts"]
                    ts = time.strftime("%H:%M:%S")
                    if r2 == COMM_SUCCESS and temp is not None and temp > ss["health_temp_thresh"]:
                        alerts.append(
                            f"[{ts}] WARN ID{sid}: TEMP={temp}C > {ss['health_temp_thresh']}C"
                        )
                    if r3 == COMM_SUCCESS and curr is not None and curr > ss["health_curr_thresh"]:
                        alerts.append(
                            f"[{ts}] WARN ID{sid}: CURRENT={curr:.0f}mA > {ss['health_curr_thresh']:.0f}mA"
                        )
                    if r4 == COMM_SUCCESS and status is not None and status != 0:
                        alerts.append(f"[{ts}] WARN ID{sid}: STATUS=0x{status:02X}")

        # Telemetry DB logging — flush queued rows once per poll
        if ss.get("log_to_db") and ss.get("_db_pending"):
            try:
                conn = ss.get("_db_conn")
                if conn is None:
                    conn = _db_init(ss["db_path"])
                    ss["_db_conn"] = conn
                _db_insert(conn, ss["_db_pending"])
            except Exception:
                pass  # DB errors must not stop the poll loop
            finally:
                ss["_db_pending"] = []

        ss["poll_error"] = None
        ss["poll_count"] = poll_count + 1
    except Exception as exc:
        ss["poll_error"] = str(exc)


# ---------------------------------------------------------------------------
# Tab 1: Calibration
# ---------------------------------------------------------------------------


def _tab_calibration(
    device: str, baud: int, scan_range: str, unlock: bool, lock: bool
) -> None:
    st.header("Calibration")
    st.write(
        "Configure servo IDs, angle limits, acceleration, speed, mode, "
        "torque, and baud rate in a single run."
    )

    col_assign, col_limits = st.columns(2)
    with col_assign:
        st.subheader("ID Remapping")
        assign_text = st.text_area(
            "Assign IDs -- one OLD:NEW per line",
            placeholder="1:11\n2:12",
            key="cal_assign",
        )
    with col_limits:
        st.subheader("Angle Limits")
        angle_text = st.text_area(
            "Angle limits -- one ID:MIN:MAX per line",
            placeholder="1:512:3584\n2:512:3584",
            key="cal_angle",
        )

    col_motion, col_mode = st.columns(2)
    with col_motion:
        st.subheader("Motion Parameters")
        acc_text = st.text_area(
            "Acceleration -- ID:ACC", placeholder="1:50\n2:50", key="cal_acc"
        )
        speed_text = st.text_area(
            "Speed -- ID:SPEED", placeholder="1:300\n2:300", key="cal_speed"
        )
    with col_mode:
        st.subheader("Mode & Torque")
        torque_text = st.text_area(
            "Torque -- ID:on/off", placeholder="1:on\n2:on", key="cal_torque"
        )
        mode_text = st.text_area(
            "Mode -- ID:0/1  (0=servo, 1=wheel)", placeholder="1:0\n2:0", key="cal_mode"
        )
        baud_text = st.text_area(
            "Baud code -- ID:CODE", placeholder="1:0\n2:0", key="cal_baud"
        )

    run_col, log_col = st.columns([1, 2])
    with run_col:
        start_run = st.button("Run Calibration", type="primary", key="cal_run")

    if start_run:
        log_buf: List[str] = []
        with st.spinner("Running..."):
            args = _build_args(
                device=device or "/dev/ttyUSB0",
                baud=baud,
                scan_range=scan_range or "1-10",
                assign_id=_parse_lines(assign_text),
                angle_limit=_parse_lines(angle_text),
                set_acc=_parse_lines(acc_text),
                set_speed=_parse_lines(speed_text),
                torque=_parse_lines(torque_text),
                set_mode=_parse_lines(mode_text),
                set_baud=_parse_lines(baud_text),
                unlock=unlock,
                lock=lock,
            )
            try:
                exit_code = run_calibration(args, log=log_buf.append)
            except Exception as exc:
                log_buf.append(f"Error: {exc}")
                exit_code = 1

        if exit_code == 0:
            log_col.success("Calibration completed successfully.")
        else:
            log_col.warning("Calibration returned a non-zero exit code.")
        if log_buf:
            st.code("\n".join(log_buf), language="text")


# ---------------------------------------------------------------------------
# Tab: Inspector + Live Telemetry (combined)
# ---------------------------------------------------------------------------


def _tab_inspector_telemetry(device: str, baud: int, scan_range: str) -> None:
    ss = st.session_state
    st.header("Inspector & Live Telemetry")

    inner = st.tabs(["Inspector", "Live Telemetry"])

    with inner[0]:
        st.write("Fetch full telemetry for selected servo IDs.")

        if ss.get("inspector_ids_pending") is not None:
            ss["inspector_ids"] = ss["inspector_ids_pending"]
            ss["inspector_ids_pending"] = None

        ids_col, btn_col = st.columns([3, 1])
        ids_input = ids_col.text_input(
            "Servo IDs",
            key="inspector_ids",
            placeholder="e.g. 1,2,3 or 1-6",
        )
        scan_clicked = btn_col.button("Scan Bus", width="stretch", key="insp_scan")
        fetch_clicked = st.button("Fetch Telemetry", type="primary", key="insp_fetch")
        status_ph = st.empty()

        if scan_clicked:
            try:
                id_range = parse_range(scan_range or "1-10")
                detected = discover_servos(device, baud, id_range)
                found_ids = sorted(detected.keys())
                ss["detected_servo_ids"] = found_ids
                ss["servo_diagnostics"] = {}
                ss["inspector_ids_pending"] = ",".join(map(str, found_ids))
                if found_ids:
                    status_ph.success(f"Detected: {', '.join(map(str, found_ids))}")
                else:
                    status_ph.info("No servos found in range.")
            except Exception as exc:
                status_ph.error(str(exc))

        if fetch_clicked:
            target_ids: List[int] = []
            try:
                target_ids = _parse_ids(ids_input)
            except Exception as exc:
                status_ph.error(str(exc))
            if target_ids:
                try:
                    ss["servo_diagnostics"] = read_servo_diagnostics(device, baud, target_ids)
                    status_ph.success("Telemetry fetched.")
                except Exception as exc:
                    status_ph.error(str(exc))

        diag: dict = ss.get("servo_diagnostics", {})
        servo_names: Dict[int, str] = ss.get("servo_names", {})
        for sid in sorted(diag):
            details = diag[sid] or {}
            errors = details.get("_errors", {})
            nick = servo_names.get(sid, "")
            title = f"Servo {sid} -- {nick}" if nick else f"Servo {sid}"
            with st.expander(title, expanded=False):
                name_key = f"servo_name_{sid}"
                ss.setdefault(name_key, nick)
                new_name = st.text_input("Friendly name", key=name_key).strip()
                if new_name:
                    servo_names[sid] = new_name
                else:
                    servo_names.pop(sid, None)
                cols = st.columns(2)
                for i, label in enumerate(METRIC_ORDER):
                    if label not in details:
                        continue
                    cols[i % 2].markdown(f"**{label}:** {_fmt_metric(label, details[label])}")
                if errors:
                    st.caption(
                        "Errors: "
                        + "; ".join(
                            f"{k} (r={r}, e={e})" for k, (r, e) in errors.items()
                        )
                    )
        ss["servo_names"] = servo_names

    with inner[1]:
        st.write("Real-time position and speed streaming for soarm100 joints (IDs 1-6).")

        cfg_col, ctrl_col = st.columns([2, 1])
        with cfg_col:
            new_ids = st.text_input(
                "Joint IDs to stream",
                value=ss.get("stream_ids_raw", "1-6"),
                key="telem_ids",
            )
            ss["stream_ids_raw"] = new_ids
            new_interval = st.slider(
                "Poll interval (ms)",
                min_value=100,
                max_value=2000,
                value=ss.get("stream_interval_ms", 200),
                step=50,
                key="telem_interval",
            )
            ss["stream_interval_ms"] = new_interval

        with ctrl_col:
            st.markdown("&nbsp;", unsafe_allow_html=True)
            if ss.get("telemetry_active"):
                if st.button("Stop", type="secondary", width="stretch", key="telem_stop"):
                    ss["telemetry_active"] = False
                    st.rerun()
            else:
                if st.button(
                    "Start Streaming", type="primary", width="stretch", key="telem_start"
                ):
                    ss["telemetry_active"] = True
                    ss["stream_history"] = deque(maxlen=_HIST)
                    st.rerun()

        if ss.get("poll_error"):
            st.error(f"Poll error: {ss['poll_error']}")

        if ss.get("telemetry_active"):
            st.success("Streaming active")
        else:
            st.info("Streaming stopped.")

        history = ss.get("stream_history", deque())
        pos_df = _history_to_df(history, "pos")
        speed_df = _history_to_df(history, "speed")

        if pos_df is not None and not pos_df.empty:
            st.subheader("Position (counts)")
            st.line_chart(pos_df)
        else:
            st.caption("No position data yet.")

        if speed_df is not None and not speed_df.empty:
            st.subheader("Speed (counts/s)")
            st.line_chart(speed_df)

        if st.button("Clear History", key="telem_clear"):
            ss["stream_history"] = deque(maxlen=_HIST)
            st.rerun()


# ---------------------------------------------------------------------------
# Tab: Command Panel
# ---------------------------------------------------------------------------


def _tab_command(device: str, baud: int) -> None:
    ss = st.session_state
    st.header("Command Panel")
    st.write(
        "Send position or velocity commands to individual joints or all joints "
        "simultaneously via a single sync packet."
    )

    mode = st.radio(
        "Control mode",
        ["Servo (position + speed + acc)", "Wheel (continuous speed)"],
        horizontal=True,
        key="cmd_mode",
    )
    servo_mode = mode.startswith("Servo")

    ids_raw = st.text_input("Joint IDs", value="1-6", key="cmd_ids_input")
    try:
        cmd_ids = _parse_ids(ids_raw)
    except Exception:
        cmd_ids = list(SOARM100_IDS)
        st.warning("Invalid IDs -- defaulting to 1-6.")

    # Latest positions from stream history for feedback overlay
    latest_pos: Dict[int, int] = {}
    for entry in ss.get("stream_history", []):
        latest_pos[entry["id"]] = entry["pos"]

    # ---- Auto-load EEPROM limits + current position for any uncached joint ----
    _missing = [
        _sid for _sid in cmd_ids
        if _sid not in ss["cmd_limits"] or _sid not in ss["cmd_curr_pos"]
    ]
    if _missing and device:
        try:
            with _bus(device, baud) as _srv:
                for _sid in _missing:
                    _p, _r, _ = _srv.ReadPos(_sid)
                    if _r == COMM_SUCCESS:
                        ss["cmd_curr_pos"][_sid] = _p
                    _mn = _srv.read2ByteTxRx(_sid, STS_MIN_ANGLE_LIMIT_L)
                    _mx = _srv.read2ByteTxRx(_sid, STS_MAX_ANGLE_LIMIT_L)
                    if _mn.result == COMM_SUCCESS and _mx.result == COMM_SUCCESS:
                        ss["cmd_limits"][_sid] = (_mn.data[0], _mx.data[0])
        except Exception:
            pass  # silently ignore — sliders fall back to 0-4095

    # ---- Manual refresh button (force re-read all selected joints) ----
    if st.button("Refresh Positions & Limits", key="cmd_refresh"):
        try:
            with _bus(device, baud) as _srv:
                for _sid in cmd_ids:
                    _p, _r, _ = _srv.ReadPos(_sid)
                    if _r == COMM_SUCCESS:
                        ss["cmd_curr_pos"][_sid] = _p
                    _mn = _srv.read2ByteTxRx(_sid, STS_MIN_ANGLE_LIMIT_L)
                    _mx = _srv.read2ByteTxRx(_sid, STS_MAX_ANGLE_LIMIT_L)
                    if _mn.result == COMM_SUCCESS and _mx.result == COMM_SUCCESS:
                        ss["cmd_limits"][_sid] = (_mn.data[0], _mx.data[0])
            st.success("Positions and limits refreshed.")
        except Exception as _exc:
            st.error(str(_exc))

    st.subheader("Per-Joint Controls")
    for sid in cmd_ids:
        with st.expander(f"Joint {sid}", expanded=True):
            # Resolve limits: 0/0 means no EEPROM limit → use full range
            _lmin, _lmax = ss["cmd_limits"].get(sid, (0, 4095))
            if _lmin == 0 and _lmax == 0:
                _lmin, _lmax = 0, 4095
            _lmin_deg = round(_lmin * 360 / 4096, 1)
            _lmax_deg = round(_lmax * 360 / 4096, 1)

            # Current position: prefer hardware-refreshed; fall back to stream
            _curr = ss["cmd_curr_pos"].get(sid, latest_pos.get(sid, 2048))
            _curr = max(_lmin, min(_lmax, _curr))  # clamp to valid range

            if servo_mode:
                c1, c2, c3 = st.columns(3)
                c1.slider(
                    "Position (ticks)",
                    _lmin, _lmax,
                    value=_curr,
                    key=f"cmd_pos_{sid}",
                    help=(
                        f"Angle limits: {_lmin}–{_lmax} ticks "
                        f"({_lmin_deg}°–{_lmax_deg}°)"
                    ),
                )
                _pos_now = ss.get(f"cmd_pos_{sid}", _curr)
                c1.caption(
                    f"{round(_pos_now * 360 / 4096, 1)}° "
                    f"| limits {_lmin_deg}°–{_lmax_deg}°"
                )
                c2.slider("Speed (ticks/s)", 0, 3000, value=300, key=f"cmd_spd_{sid}")
                c3.slider("Acc (ticks/s²)", 0, 254, value=50, key=f"cmd_acc_{sid}")
            else:
                c1, c2 = st.columns(2)
                c1.slider(
                    "Speed (ticks/s, ±)", -3000, 3000, value=0,
                    key=f"cmd_wspd_{sid}",
                )
                c2.slider("Acc (ticks/s²)", 0, 254, value=50, key=f"cmd_wacc_{sid}")

            _actual = ss["cmd_curr_pos"].get(sid, latest_pos.get(sid))
            if _actual is not None:
                st.caption(
                    f"Hardware position: **{_actual}** ticks "
                    f"({round(_actual * 360 / 4096, 1)}°)"
                )

            if st.button(f"Send J{sid}", key=f"cmd_send_{sid}"):
                try:
                    with _bus(device, baud) as srv:
                        if servo_mode:
                            pos = ss.get(f"cmd_pos_{sid}", 2048)
                            spd = ss.get(f"cmd_spd_{sid}", 300)
                            acc = ss.get(f"cmd_acc_{sid}", 50)
                            srv.WritePosEx(sid, pos, spd, acc)
                            st.success(f"J{sid} -> pos={pos}, spd={spd}, acc={acc}")
                        else:
                            spd = ss.get(f"cmd_wspd_{sid}", 0)
                            acc = ss.get(f"cmd_wacc_{sid}", 50)
                            srv.WheelMode(sid)
                            srv.WriteSpec(sid, spd, acc)
                            st.success(f"J{sid} wheel -> spd={spd}, acc={acc}")
                except Exception as exc:
                    st.error(str(exc))

    st.divider()
    send_all = st.button("Send All (Sync Packet)", type="primary", key="cmd_send_all")
    if send_all:
        try:
            with _bus(device, baud) as srv:
                srv.groupSyncWrite.clearParam()
                if servo_mode:
                    for sid in cmd_ids:
                        pos = ss.get(f"cmd_pos_{sid}", 2048)
                        spd = ss.get(f"cmd_spd_{sid}", 300)
                        acc = ss.get(f"cmd_acc_{sid}", 50)
                        srv.SyncWritePosEx(sid, pos, spd, acc)
                    srv.groupSyncWrite.txPacket()
                    srv.groupSyncWrite.clearParam()
                    st.success(f"Sync position packet sent to joints {cmd_ids}.")
                else:
                    for sid in cmd_ids:
                        spd = ss.get(f"cmd_wspd_{sid}", 0)
                        acc = ss.get(f"cmd_wacc_{sid}", 50)
                        srv.WheelMode(sid)
                        srv.WriteSpec(sid, spd, acc)
                    st.success(f"Wheel mode commands sent to joints {cmd_ids}.")
        except Exception as exc:
            st.error(str(exc))


# ---------------------------------------------------------------------------
# Automatic ROM sweep helper (used by Tab 5: Homing Wizard)
# ---------------------------------------------------------------------------

_STALL_POLL_S = 0.08  # seconds between position reads during ROM sweep


def _run_rom_sweep_auto(
    device: str,
    baud: int,
    joint_ids: List[int],
    sweep_speed: int,
    stall_thr: int,
    stall_win: int,
    timeout_s: float,
    max_range_ticks: int = 0,
) -> Dict[int, dict]:
    """Drive each joint in wheel mode to both mechanical limits.

    For each joint the function:
    1. Enables torque and unlocks EEPROM.
    2. Enters wheel mode (STS_MODE = 1).
    3. Sweeps in the positive direction until stall or timeout; records *pos_max*.
    4. Immediately reverses; sweeps in the negative direction; records *pos_min*.
    5. Stops, restores servo mode, and moves the joint to the midpoint.

    Stall is detected when the spread of the last *stall_win* position samples
    is at most *stall_thr* ticks.

    If *max_range_ticks* > 0 the sweep in each direction stops as soon as the
    joint has travelled that many ticks from its sweep-start position, even if
    no stall is detected.  Use this to protect joints whose physical range is
    known and smaller than the full encoder range.

    Writes progress via ``st.write()``; call inside an ``st.status`` block for
    live streaming output.

    Returns
    -------
    dict
        ``{servo_id: {pos_min, pos_max, zero, range_ticks,
                      stalled_fwd, stalled_rev}}``
    """
    results: Dict[int, dict] = {}
    n = len(joint_ids)

    with _bus(device, baud) as srv:
        for idx, sid in enumerate(joint_ids):
            st.write(
                f"**J{sid}** ({idx + 1}/{n}) — enabling torque, "
                "entering wheel mode..."
            )
            write1(srv, sid, STS_TORQUE_ENABLE, 1, "torque on")
            write1(srv, sid, STS_LOCK, 0, "unlock EEPROM")
            srv.WheelMode(sid)

            # ---- Forward sweep (positive direction) ----
            st.write(
                f"**J{sid}** sweeping → positive limit (speed +{sweep_speed})..."
            )
            srv.WriteSpec(sid, sweep_speed, 5)
            recent: deque = deque(maxlen=stall_win)
            pos_max = 0
            stalled_fwd = False
            # Read start position; don't arm stall check until joint has moved
            # at least min_travel ticks away from it (prevents false stall at
            # the initial stationary position).
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
                    # Only start filling the stall window once moving
                    if abs(p - _start_fwd) >= _min_travel:
                        recent.append(p)
                    # Hard travel cap: stop early if max_range_ticks set
                    if max_range_ticks > 0 and abs(p - _start_fwd) >= max_range_ticks:
                        stalled_fwd = False  # cap reached, not a stall
                        break
                if len(recent) >= stall_win and (
                    max(recent) - min(recent)
                ) <= stall_thr:
                    stalled_fwd = True
                    break
                time.sleep(_STALL_POLL_S)
            srv.WriteSpec(sid, 0, 5)  # stop
            time.sleep(0.4)
            st.write(
                f"**J{sid}** (+) {'stalled ✓' if stalled_fwd else 'timed-out'}"
                f" at **{pos_max}** ticks"
            )

            # ---- Reverse sweep (negative direction) ----
            st.write(
                f"**J{sid}** sweeping ← negative limit (speed −{sweep_speed})..."
            )
            recent.clear()
            srv.WriteSpec(sid, -sweep_speed, 5)
            pos_min = 4095
            stalled_rev = False
            # Same guard: arm stall detection only after actual movement
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
                    # Hard travel cap
                    if max_range_ticks > 0 and abs(p - _start_rev) >= max_range_ticks:
                        stalled_rev = False
                        break
                if len(recent) >= stall_win and (
                    max(recent) - min(recent)
                ) <= stall_thr:
                    stalled_rev = True
                    break
                time.sleep(_STALL_POLL_S)
            srv.WriteSpec(sid, 0, 5)  # stop
            time.sleep(0.4)
            st.write(
                f"**J{sid}** (−) {'stalled ✓' if stalled_rev else 'timed-out'}"
                f" at **{pos_min}** ticks"
            )

            # ---- Restore servo mode and move to midpoint ----
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
            st.write(
                f"**J{sid}** done — min={pos_min}, max={pos_max},"
                f" zero={zero}, range={pos_max - pos_min} ticks"
            )

        # Lock all servos once all sweeps are complete
        for sid in joint_ids:
            write1(srv, sid, STS_LOCK, 1, "lock")

    return results


# ---------------------------------------------------------------------------
# Homing Wizard — shared apply/save helper
# ---------------------------------------------------------------------------

def _homing_apply_save(
    device: str,
    baud: int,
    rom_results: Dict[int, dict],
    target_ref: int,
    speed_label: int = 150,
) -> None:
    """Shared apply + save section used by both Auto and Manual homing modes."""
    ss = st.session_state

    if not rom_results:
        return

    st.subheader("Results")
    _rows = []
    _has_warning = False
    for _sid, _d in sorted(rom_results.items()):
        _range_deg = round(_d["range_ticks"] / 4096 * 360, 1)
        _warn = _range_deg > 300 or _d["pos_min"] < 50 or _d["pos_max"] > 4046
        if _warn:
            _has_warning = True
        _rows.append({
            "Joint": f"J{_sid}",
            "Min (ticks)": _d["pos_min"],
            "Max (ticks)": _d["pos_max"],
            "Zero = midpoint": _d["zero"],
            "Range (ticks)": _d["range_ticks"],
            "Range (°)": _range_deg,
            "Method": _d.get("method", "—"),
            "⚠": "⚠ check" if _warn else "",
        })
    if _has_warning:
        st.warning(
            "One or more joints show a range > 300° or limits near the encoder "
            "boundary (0 / 4095). This usually means the sweep hit an encoder "
            "boundary instead of the physical stop."
        )
    st.dataframe(pd.DataFrame(_rows), width="stretch", hide_index=True)

    st.write(f"**Post-offset preview** (zero → target_ref = {target_ref} ticks):")
    _post_rows = []
    for _sid, _d in sorted(rom_results.items()):
        _off = target_ref - _d["zero"]
        _post_rows.append({
            "Joint": f"J{_sid}",
            "Signed offset": _off,
            "Limit min (post-offset)": max(0, min(4095, _d["pos_min"] + _off)),
            "Limit max (post-offset)": max(0, min(4095, _d["pos_max"] + _off)),
        })
    st.dataframe(
        pd.DataFrame(_post_rows), width="stretch", hide_index=True
    )

    if st.button(
        "Apply: Write Offsets + Limits to EEPROM",
        type="primary",
        key="rom_apply",
    ):
        try:
            with _bus(device, baud) as _srv:
                for _sid, _d in rom_results.items():
                    _off = target_ref - _d["zero"]
                    _reg = (abs(_off) | 0x0800) if _off < 0 else abs(_off)
                    _lmin = max(0, min(4095, _d["pos_min"] + _off))
                    _lmax = max(0, min(4095, _d["pos_max"] + _off))
                    write1(_srv, _sid, STS_LOCK, 0, "unlock")
                    write2(_srv, _sid, STS_OFS_L, _reg, f"offset J{_sid}")
                    write2(_srv, _sid, STS_MIN_ANGLE_LIMIT_L, _lmin, f"min J{_sid}")
                    write2(_srv, _sid, STS_MAX_ANGLE_LIMIT_L, _lmax, f"max J{_sid}")
                    write1(_srv, _sid, STS_LOCK, 1, "lock")
            st.success("Offsets and angle limits written to all joints.")
        except Exception as _exc:
            st.error(str(_exc))

    _cfg_path = st.text_input(
        "Config save path",
        value=ss.get("rom_config_path", str(_CONFIGS_DIR / "soarm100_rom.json")),
        key="rom_cfg_path",
    )
    ss["rom_config_path"] = _cfg_path

    _cfg_data: Dict[str, Any] = {
        "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "device": device,
        "target_ref": target_ref,
        "joints": {},
    }
    for _sid, _d in sorted(rom_results.items()):
        _off = target_ref - _d["zero"]
        _cfg_data["joints"][str(_sid)] = {
            "pos_min": _d["pos_min"],
            "pos_max": _d["pos_max"],
            "zero_midpoint": _d["zero"],
            "range_ticks": _d["range_ticks"],
            "method": _d.get("method", "—"),
            "offset_signed": _off,
            "offset_register": (abs(_off) | 0x0800) if _off < 0 else abs(_off),
            "limit_min": max(0, min(4095, _d["pos_min"] + _off)),
            "limit_max": max(0, min(4095, _d["pos_max"] + _off)),
        }
    _cfg_json = json.dumps(_cfg_data, indent=2)

    _sc1, _sc2 = st.columns(2)
    if _sc1.button("Save Config to Disk", key="rom_save_disk"):
        try:
            _save_path = Path(_cfg_path)
            _save_path.parent.mkdir(parents=True, exist_ok=True)
            _save_path.write_text(_cfg_json)
            st.success(f"Saved to `{_save_path}`")
        except Exception as _exc:
            st.error(str(_exc))
    _sc2.download_button(
        "Download Config JSON",
        data=_cfg_json.encode(),
        file_name="soarm100_rom.json",
        mime="application/json",
        key="rom_dl",
    )
    if st.button("Clear Results", key="rom_clear"):
        ss["rom_results"] = {}
        st.rerun()


# ---------------------------------------------------------------------------
# Tab: Homing Wizard
# ---------------------------------------------------------------------------


def _tab_homing(device: str, baud: int) -> None:
    ss = st.session_state
    st.header("Homing Wizard")

    homing_mode = st.radio(
        "Mode",
        ["Automatic — motor sweep", "Manual — move by hand"],
        horizontal=True,
        key="homing_mode",
    )

    # ── Shared: joint selection and target reference ──────────────────────
    _hc1, _hc2 = st.columns(2)
    homing_ids: List[int] = _hc1.multiselect(
        "Joints",
        options=SOARM100_IDS,
        default=ss.get("wizard_joint_ids", SOARM100_IDS),
        format_func=lambda x: f"J{x}",
        key="homing_ids",
    )
    ss["wizard_joint_ids"] = sorted(homing_ids)
    target_ref = int(_hc2.number_input(
        "Zero reference (ticks)",
        0, 4095,
        value=ss.get("wizard_target_ref", 2048),
        key="homing_target_ref",
        help="Encoder count the midpoint should report after calibration. Default 2048.",
    ))
    ss["wizard_target_ref"] = target_ref

    st.divider()

    # ══════════════════════════════════════════════════════════════════════
    # AUTOMATIC MODE — motor sweep
    # ══════════════════════════════════════════════════════════════════════
    if homing_mode.startswith("Auto"):
        st.subheader("Automatic Range-of-Motion Sweep")
        st.info(
            "Set angle limits to the full range (0–4095) on each joint before "
            "running, and keep the workspace clear."
        )

        _rp1, _rp2, _rp3, _rp4 = st.columns(4)
        rom_speed = int(_rp1.slider(
            "Sweep speed (ticks/s)", 30, 500, value=150, step=10, key="rom_speed",
        ))
        rom_stall_thr = int(_rp2.number_input(
            "Stall threshold (ticks)", 1, 50, value=5, key="rom_stall_thr",
        ))
        rom_stall_win = int(_rp3.number_input(
            "Stall window (samples)", 3, 20, value=8, key="rom_stall_win",
        ))
        rom_timeout = float(_rp4.number_input(
            "Timeout/direction (s)", 5, 120, value=30, key="rom_timeout",
        ))
        rom_max_range = int(st.number_input(
            "Max travel per direction (ticks, 0 = unlimited)",
            min_value=0, max_value=4096, value=0, step=50,
            key="rom_max_range",
            help=(
                "Safety cap: stops sweep after this many ticks from start. "
                "Set to ~half the expected range to prevent encoder-boundary contact."
            ),
        ))

        def _do_rom_sweep(ids: List[int]) -> None:
            if not ids:
                st.warning("No joints selected.")
                return
            with st.status(f"Sweeping J{ids}...", expanded=True) as _sw_st:
                try:
                    _res = _run_rom_sweep_auto(
                        device, baud, ids,
                        sweep_speed=rom_speed,
                        stall_thr=rom_stall_thr,
                        stall_win=rom_stall_win,
                        timeout_s=rom_timeout,
                        max_range_ticks=rom_max_range,
                    )
                    for _sid, _d in _res.items():
                        _d["method"] = "auto-sweep"
                    ss["rom_results"].update(_res)
                    _sw_st.update(
                        label=f"Sweep complete — {len(_res)} joint(s)",
                        state="complete", expanded=False,
                    )
                except Exception as _exc:
                    _sw_st.update(label="Sweep failed", state="error")
                    st.error(str(_exc))

        if homing_ids:
            _btn_cols = st.columns(len(homing_ids) + 1)
            for _ci, _sid in enumerate(sorted(homing_ids)):
                if _btn_cols[_ci].button(f"Sweep J{_sid}", key=f"rom_sweep_{_sid}"):
                    _do_rom_sweep([_sid])
            if _btn_cols[-1].button("Sweep All", type="primary", key="rom_sweep_all"):
                _do_rom_sweep(sorted(homing_ids))
        else:
            st.info("Select at least one joint above.")

    # ══════════════════════════════════════════════════════════════════════
    # MANUAL MODE — move by hand
    # ══════════════════════════════════════════════════════════════════════
    else:
        st.subheader("Manual Limit Recording")
        st.write(
            "Torque is disabled so you can move each joint by hand. "
            "Bring each joint to its **minimum** limit and click **Record Min**, "
            "then bring it to its **maximum** limit and click **Record Max**. "
            "The midpoint is computed as the zero pose."
        )

        if st.button("Disable Torque on Selected Joints", key="man_torque_off"):
            if not homing_ids:
                st.warning("Select joints first.")
            else:
                try:
                    with _bus(device, baud) as _srv:
                        for _sid in homing_ids:
                            write1(_srv, _sid, STS_TORQUE_ENABLE, 0, "torque off")
                    st.success(f"Torque OFF on J{homing_ids}")
                except Exception as _exc:
                    st.error(str(_exc))

        st.write("---")
        man_results: Dict[int, dict] = ss.setdefault("man_results", {})

        for _sid in sorted(homing_ids):
            _mr = man_results.setdefault(_sid, {"pos_min": None, "pos_max": None})
            _cols = st.columns([1, 2, 2, 2])
            _cols[0].markdown(f"**J{_sid}**")

            if _cols[1].button(f"Record Min", key=f"man_min_{_sid}"):
                try:
                    with _bus(device, baud) as _srv:
                        _p, _r, _ = _srv.ReadPos(_sid)
                    if _r == COMM_SUCCESS:
                        _mr["pos_min"] = _p
                        st.toast(f"J{_sid} min = {_p}")
                    else:
                        st.error(f"J{_sid}: read failed")
                except Exception as _exc:
                    st.error(str(_exc))

            if _cols[2].button(f"Record Max", key=f"man_max_{_sid}"):
                try:
                    with _bus(device, baud) as _srv:
                        _p, _r, _ = _srv.ReadPos(_sid)
                    if _r == COMM_SUCCESS:
                        _mr["pos_max"] = _p
                        st.toast(f"J{_sid} max = {_p}")
                    else:
                        st.error(f"J{_sid}: read failed")
                except Exception as _exc:
                    st.error(str(_exc))

            _mn = _mr["pos_min"]
            _mx = _mr["pos_max"]
            _cols[3].caption(
                f"min={_mn if _mn is not None else '—'}  "
                f"max={_mx if _mx is not None else '—'}"
            )

        # Build rom_results from manual recordings
        _ready = [
            _sid for _sid in homing_ids
            if man_results.get(_sid, {}).get("pos_min") is not None
            and man_results.get(_sid, {}).get("pos_max") is not None
        ]
        if _ready:
            if st.button("Compute from Recorded Limits", type="primary", key="man_compute"):
                for _sid in _ready:
                    _mn = man_results[_sid]["pos_min"]
                    _mx = man_results[_sid]["pos_max"]
                    if _mn > _mx:
                        _mn, _mx = _mx, _mn  # swap if user recorded in reverse order
                    _zero = (_mn + _mx) // 2
                    ss["rom_results"][_sid] = {
                        "pos_min": _mn,
                        "pos_max": _mx,
                        "zero": _zero,
                        "range_ticks": _mx - _mn,
                        "method": "manual",
                    }
                st.success(f"Computed results for J{_ready}.")
                st.rerun()
        else:
            st.info("Record both min and max for at least one joint to proceed.")

    # ══════════════════════════════════════════════════════════════════════
    # SHARED: apply + save
    # ══════════════════════════════════════════════════════════════════════
    st.divider()
    _homing_apply_save(device, baud, ss.get("rom_results", {}), target_ref)


# ---------------------------------------------------------------------------
# Tab 6: Recorder
# ---------------------------------------------------------------------------


def _tab_recorder(device: str, baud: int) -> None:
    ss = st.session_state
    st.header("Position Recorder & Replayer")
    st.write(
        "Record demonstration trajectories from soarm100 joints, "
        "download as CSV, and replay them via sync packets."
    )

    ids_raw = st.text_input(
        "Joint IDs to record",
        value=ss.get("stream_ids_raw", "1-6"),
        key="rec_ids",
    )
    ss["stream_ids_raw"] = ids_raw

    rec_col, stat_col = st.columns([1, 2])
    recording = ss.get("recording", False)

    if recording:
        if rec_col.button("Stop Recording", type="secondary", key="rec_stop"):
            ss["recording"] = False
            st.rerun()
        stat_col.success(f"Recording... {len(ss.get('recording_data', []))} samples")
    else:
        if rec_col.button("Start Recording", type="primary", key="rec_start"):
            ss["recording"] = True
            ss["recording_data"] = []
            ss["stream_history"] = deque(maxlen=_HIST)
            st.rerun()
        n = len(ss.get("recording_data", []))
        if n:
            stat_col.info(f"{n} samples ready for download.")

    rec_data = ss.get("recording_data", [])
    if rec_data:
        buf = io.StringIO()
        writer = csv.DictWriter(buf, fieldnames=["timestamp", "id", "position", "speed"])
        writer.writeheader()
        writer.writerows(rec_data)
        st.download_button(
            "Download trajectory.csv",
            data=buf.getvalue().encode(),
            file_name="trajectory.csv",
            mime="text/csv",
            key="rec_dl",
        )
        if st.button("Clear Recording", key="rec_clear"):
            ss["recording_data"] = []
            st.rerun()

    st.divider()
    st.subheader("Replay")
    uploaded = st.file_uploader("Upload trajectory CSV", type=["csv"], key="rec_upload")
    if uploaded is not None:
        try:
            ss["replay_data"] = list(
                csv.DictReader(io.StringIO(uploaded.getvalue().decode()))
            )
            st.success(f"Loaded {len(ss['replay_data'])} rows.")
        except Exception as exc:
            st.error(str(exc))

    replay_data: List[dict] = ss.get("replay_data", [])
    if replay_data:
        st.write(f"**{len(replay_data)} samples** ready.")

        # Trajectory preview chart
        st.subheader("Preview")
        _preview_trajectory(replay_data)

        prev_col, _ = st.columns([1, 3])
        if prev_col.button("Dry-Run Preview (no packets sent)", key="rp_dryrun"):
            st.info(
                "Dry-run complete: chart above shows what will be replayed. "
                "No packets were sent to the servos."
            )

        st.subheader("Replay Controls")
        rp1, rp2, rp3 = st.columns(3)
        speed_scale = rp1.slider(
            "Speed scale", 0.1, 5.0, value=1.0, step=0.1, key="rp_scale"
        )
        loop_count = rp2.number_input("Loops", 1, 20, value=1, key="rp_loops")
        inter_delay_ms = rp3.slider(
            "Inter-point delay (ms)", 0, 300, value=20, step=5, key="rp_delay"
        )
        if st.button("Replay Trajectory", type="primary", key="rp_play"):
            _run_replay(
                device, baud, replay_data, speed_scale, int(loop_count), inter_delay_ms
            )


def _run_replay(
    device: str,
    baud: int,
    data: List[dict],
    speed_scale: float,
    loops: int,
    inter_delay_ms: int,
) -> None:
    """Group rows by timestamp and replay as sync packets."""
    try:
        sorted_rows = sorted(data, key=lambda r: float(r["timestamp"]))
    except Exception:
        sorted_rows = data

    groups: List[List[dict]] = []
    current: List[dict] = []
    last_t: Optional[float] = None
    for row in sorted_rows:
        t = float(row["timestamp"])
        if last_t is None or abs(t - last_t) < 0.005:
            current.append(row)
        else:
            if current:
                groups.append(current)
            current = [row]
        last_t = t
    if current:
        groups.append(current)

    total = len(groups) * loops
    progress = st.progress(0.0, text="Replaying...")
    status_ph = st.empty()
    done = 0

    try:
        with _bus(device, baud) as srv:
            for _ in range(loops):
                prev_t: Optional[float] = None
                for group in groups:
                    gt = float(group[0]["timestamp"])
                    if prev_t is not None:
                        dt = max(
                            0.0, (gt - prev_t) / speed_scale - inter_delay_ms / 1000.0
                        )
                        time.sleep(dt)
                    time.sleep(inter_delay_ms / 1000.0)
                    prev_t = gt

                    srv.groupSyncWrite.clearParam()
                    for row in group:
                        sid = int(row["id"])
                        pos = int(row["position"])
                        raw_spd = abs(int(float(row.get("speed", 300))))
                        spd = max(0, min(3000, int(raw_spd * speed_scale)))
                        srv.SyncWritePosEx(sid, pos, spd, 50)
                    srv.groupSyncWrite.txPacket()
                    srv.groupSyncWrite.clearParam()

                    done += 1
                    progress.progress(min(done / total, 1.0))
        status_ph.success("Replay complete.")
    except Exception as exc:
        status_ph.error(str(exc))


# ---------------------------------------------------------------------------
# Tab 7: Health Monitor
# ---------------------------------------------------------------------------


def _tab_health() -> None:
    ss = st.session_state
    st.header("Health Monitor")
    st.write(
        "Continuously monitor temperature, current, and status error flags "
        "for soarm100 joints. Alerts trigger when thresholds are exceeded."
    )

    cfg_col, ctrl_col = st.columns([2, 1])
    with cfg_col:
        ss["health_temp_thresh"] = st.slider(
            "Temperature alert threshold (C)",
            min_value=30,
            max_value=100,
            value=int(ss.get("health_temp_thresh", 70)),
            key="health_temp_thr",
        )
        ss["health_curr_thresh"] = st.slider(
            "Current alert threshold (mA)",
            min_value=100.0,
            max_value=5000.0,
            value=float(ss.get("health_curr_thresh", 1500.0)),
            step=100.0,
            key="health_curr_thr",
        )
    with ctrl_col:
        st.markdown("&nbsp;", unsafe_allow_html=True)
        if ss.get("health_active"):
            if st.button(
                "Stop", type="secondary", width="stretch", key="health_stop"
            ):
                ss["health_active"] = False
                st.rerun()
        else:
            if st.button(
                "Start Monitoring",
                type="primary",
                width="stretch",
                key="health_start",
            ):
                ss["health_active"] = True
                ss["health_history"] = deque(maxlen=_HIST)
                st.rerun()

    if ss.get("health_active"):
        st.success("Monitoring active")

    health_hist = ss.get("health_history", deque())
    latest: Dict[int, dict] = {}
    for row in health_hist:
        latest[row["id"]] = row

    if latest:
        st.subheader("Current Status")
        cols = st.columns(len(latest))
        for i, (sid, data) in enumerate(sorted(latest.items())):
            temp_val = data.get("temp")
            curr_val = data.get("current")
            status_val = data.get("status")
            bad = (
                (temp_val or 0) > ss.get("health_temp_thresh", 70)
                or (curr_val or 0) > ss.get("health_curr_thresh", 1500)
                or ((status_val or 0) != 0)
            )
            badge = "ALERT" if bad else "OK"
            cols[i].metric(
                f"J{sid} [{badge}] Temp",
                f"{temp_val:.1f} C" if temp_val is not None else "N/A",
            )
            cols[i].metric(
                f"J{sid} Current",
                f"{curr_val:.0f} mA" if curr_val is not None else "N/A",
            )
            if status_val is not None:
                cols[i].caption(f"Status: 0x{status_val:02X}")

    # Temperature history chart
    temp_data: Dict[str, Dict] = {}
    if health_hist:
        rows_list = list(health_hist)
        t0 = rows_list[0]["t"]
        for row in rows_list:
            if row.get("temp") is None:
                continue
            col = f"J{row['id']}"
            temp_data.setdefault(col, {})[round(row["t"] - t0, 2)] = row["temp"]
    if temp_data:
        temp_df = pd.DataFrame(temp_data)
        temp_df.index.name = "t (s)"
        st.subheader("Temperature History (C)")
        st.line_chart(temp_df)

    alerts = ss.get("health_alerts", deque())
    if alerts:
        st.subheader(f"Alert Log ({len(alerts)} entries)")
        st.code("\n".join(reversed(list(alerts))), language="text")
        st.download_button(
            "Download Alert Log",
            data="\n".join(list(alerts)).encode(),
            file_name="health_alerts.txt",
            mime="text/plain",
            key="health_dl",
        )
        if st.button("Clear Alerts", key="health_clear"):
            ss["health_alerts"] = deque(maxlen=200)
            st.rerun()

    st.divider()
    st.subheader("Persistent Telemetry Log")
    st.write(
        "Log position and speed of all monitored joints to a SQLite database "
        "for long-term analysis. The DB file is written on the same machine "
        "running the dashboard."
    )

    db_path_input = st.text_input(
        "Database path",
        value=ss.get("db_path", str(Path.home() / ".stservo_telemetry.db")),
        key="db_path_input",
    )
    ss["db_path"] = db_path_input

    log_col, dl_col, clr_col = st.columns(3)

    if ss.get("log_to_db"):
        if log_col.button("Stop Logging", type="secondary", key="db_stop"):
            ss["log_to_db"] = False
            st.rerun()
        st.success("Logging to DB active")
    else:
        if log_col.button("Start Logging", type="primary", key="db_start"):
            # Re-open connection in case path changed
            try:
                if ss.get("_db_conn") is not None:
                    ss["_db_conn"].close()
                ss["_db_conn"] = _db_init(db_path_input)
                ss["log_to_db"] = True
                ss["_db_pending"] = []
                st.rerun()
            except Exception as exc:
                st.error(str(exc))

    db_file = Path(db_path_input)
    if db_file.exists():
        size_kb = db_file.stat().st_size / 1024
        st.caption(f"DB size: {size_kb:.1f} KB — {db_path_input}")
        with open(db_path_input, "rb") as fh:
            dl_col.download_button(
                "Download DB",
                data=fh.read(),
                file_name=db_file.name,
                mime="application/octet-stream",
                key="db_dl",
            )

    if clr_col.button("Clear DB", key="db_clear") and db_file.exists():
        try:
            if ss.get("_db_conn") is not None:
                ss["_db_conn"].close()
                ss["_db_conn"] = None
            db_file.unlink()
            ss["log_to_db"] = False
            st.rerun()
        except Exception as exc:
            st.error(str(exc))

    st.subheader("Query")
    q1, q2, q3 = st.columns(3)
    q_ids = q1.text_input("Servo IDs (blank = all)", value="", key="db_q_ids")
    q_minutes = q2.slider("Last N minutes", 1, 120, value=5, key="db_q_min")
    q_limit = q3.number_input("Row limit", 100, 10000, value=2000, step=100, key="db_q_lim")

    if st.button("Run Query", key="db_query_run") and db_file.exists():
        try:
            conn = ss.get("_db_conn") or _db_init(db_path_input)
            since = time.time() - q_minutes * 60
            filter_ids: Optional[List[int]] = None
            if q_ids.strip():
                filter_ids = _parse_ids(q_ids.strip())
            df = _db_query(conn, filter_ids, since, int(q_limit))
            if df.empty:
                st.info("No rows matching the query.")
            else:
                st.dataframe(df, width="stretch")
        except Exception as exc:
            st.error(str(exc))


# ---------------------------------------------------------------------------
# Tab 8: Config Export / Import + Workspace Sweep
# ---------------------------------------------------------------------------


def _tab_config(device: str, baud: int, scan_range: str) -> None:
    ss = st.session_state
    st.header("Configuration Export / Import")
    st.write(
        "Save and restore the full register state of soarm100 servos as a JSON snapshot. "
        "Integrates with the configs/ folder for reproducible setups."
    )

    exp_col, imp_col = st.columns(2)

    with exp_col:
        st.subheader("Export")
        export_ids = st.text_input("IDs to export", value="1-6", key="cfg_exp_ids")
        if st.button("Read & Export", type="primary", key="cfg_export"):
            try:
                ids = _parse_ids(export_ids)
                diag = read_servo_diagnostics(device, baud, ids)
                snapshot: Dict[str, Any] = {
                    "exported_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "device": device,
                    "servos": {},
                }
                for sid, data in diag.items():
                    entry = {k: v for k, v in (data or {}).items() if k != "_errors"}
                    snapshot["servos"][str(sid)] = entry
                json_bytes = json.dumps(snapshot, indent=2).encode()
                st.download_button(
                    "Download soarm100_config.json",
                    data=json_bytes,
                    file_name="soarm100_config.json",
                    mime="application/json",
                    key="cfg_dl",
                )
                st.success(f"Ready -- {len(diag)} servo(s).")
            except Exception as exc:
                st.error(str(exc))

    with imp_col:
        st.subheader("Import")
        uploaded = st.file_uploader("Upload config JSON", type=["json"], key="cfg_upload")
        if uploaded is not None:
            try:
                snap = json.loads(uploaded.getvalue().decode())
                servos = snap.get("servos", {})
                st.write(
                    f"Snapshot from `{snap.get('exported_at', 'unknown')}` -- "
                    f"**{len(servos)}** servo(s)."
                )
                if st.button("Apply Config", type="primary", key="cfg_apply"):
                    log_buf2: List[str] = []
                    try:
                        with _bus(device, baud) as srv:
                            for sid_str, entry in servos.items():
                                sid = int(sid_str)
                                write1(srv, sid, STS_LOCK, 0, "unlock")
                                acc = entry.get("Acceleration")
                                if acc is not None:
                                    write1(srv, sid, STS_ACC, int(acc), "acc")
                                    log_buf2.append(f"J{sid}: ACC={acc}")
                                mode = entry.get("Mode")
                                if mode is not None:
                                    write1(srv, sid, STS_MODE, int(mode), "mode")
                                    log_buf2.append(f"J{sid}: MODE={mode}")
                                write1(srv, sid, STS_LOCK, 1, "lock")
                        log_buf2.append("Done.")
                        st.code("\n".join(log_buf2))
                        st.success("Config applied.")
                    except Exception as exc:
                        st.error(str(exc))
            except Exception as exc:
                st.error(f"Failed to parse JSON: {exc}")

    st.divider()
    st.subheader("EEPROM Diff")
    st.write(
        "Compare a saved JSON snapshot against the current live register state. "
        "Changed fields are highlighted in the table below."
    )

    diff_upload = st.file_uploader(
        "Upload snapshot JSON for comparison", type=["json"], key="diff_upload"
    )
    diff_ids_raw = st.text_input("IDs to read live", value="1-6", key="diff_ids")

    if st.button("Compare Snapshot vs Live", type="primary", key="diff_run"):
        if diff_upload is None:
            st.warning("Upload a snapshot JSON first.")
        else:
            try:
                snap_data = json.loads(diff_upload.getvalue().decode())
                snap_servos: Dict[str, Any] = snap_data.get("servos", {})
                diff_ids = _parse_ids(diff_ids_raw)
                live_diag = read_servo_diagnostics(device, baud, diff_ids)

                diff_rows: List[Dict[str, Any]] = []
                for sid in diff_ids:
                    snap_entry = snap_servos.get(str(sid), {})
                    live_entry = (live_diag.get(sid) or {})
                    all_keys = sorted(set(snap_entry) | set(live_entry) - {"_errors"})
                    for key in all_keys:
                        snap_val = snap_entry.get(key, "—")
                        live_val = live_entry.get(key, "—")
                        changed = str(snap_val) != str(live_val)
                        diff_rows.append(
                            {
                                "Servo": f"J{sid}",
                                "Register": key,
                                "Snapshot": snap_val,
                                "Live": live_val,
                                "Changed": "YES" if changed else "",
                            }
                        )

                if diff_rows:
                    diff_df = pd.DataFrame(diff_rows)
                    changed_mask = diff_df["Changed"] == "YES"
                    styled = diff_df.style.apply(
                        lambda row: [
                            "background-color: #ffd6d6" if row["Changed"] == "YES" else ""
                        ] * len(row),
                        axis=1,
                    )
                    st.dataframe(styled, width="stretch")
                    n_changed = changed_mask.sum()
                    if n_changed:
                        st.warning(f"{n_changed} field(s) differ from snapshot.")
                    else:
                        st.success("Live state matches the snapshot exactly.")
                else:
                    st.info("No comparable fields found.")
            except Exception as exc:
                st.error(str(exc))

    st.divider()
    st.subheader("Workspace Validation Sweep")
    st.write(
        "Move each joint from its min to max limit in N steps, "
        "recording actual vs commanded positions to validate angle limits."
    )

    sw1, sw2 = st.columns(2)
    sweep_ids = sw1.text_input("Joints to sweep", value="1-6", key="sw_ids")
    sweep_steps = sw2.number_input("Steps per joint", 3, 20, value=5, key="sw_steps")
    sw3, sw4 = st.columns(2)
    sweep_spd = sw3.number_input("Speed", 100, 3000, value=300, key="sw_spd")
    sweep_acc = sw4.number_input("Acc", 10, 254, value=50, key="sw_acc")
    tol = st.number_input("Pass/fail threshold (counts)", 1, 500, value=50, key="sw_tol")

    if st.button("Run Sweep", type="primary", key="sw_run"):
        _run_sweep(
            device, baud, sweep_ids, int(sweep_steps), int(sweep_spd), int(sweep_acc), int(tol)
        )


def _run_sweep(
    device: str,
    baud: int,
    ids_raw: str,
    n_steps: int,
    spd: int,
    acc: int,
    tol: int,
) -> None:
    try:
        ids = _parse_ids(ids_raw)
    except Exception:
        st.error("Invalid joint IDs.")
        return

    results: List[dict] = []
    total = len(ids) * n_steps
    progress = st.progress(0.0, text="Sweeping...")
    done = 0

    try:
        with _bus(device, baud) as srv:
            for sid in ids:
                min_pkt = srv.read2ByteTxRx(sid, STS_MIN_ANGLE_LIMIT_L)
                max_pkt = srv.read2ByteTxRx(sid, STS_MAX_ANGLE_LIMIT_L)
                joint_min = (
                    (min_pkt.data[0] & 0x7FFF)
                    if min_pkt.result == COMM_SUCCESS and min_pkt.data
                    else 512
                )
                joint_max = (
                    (max_pkt.data[0] & 0x7FFF)
                    if max_pkt.result == COMM_SUCCESS and max_pkt.data
                    else 3584
                )

                targets = [
                    joint_min + int((joint_max - joint_min) * i / max(n_steps - 1, 1))
                    for i in range(n_steps)
                ]

                write1(srv, sid, STS_TORQUE_ENABLE, 1, "torque on")
                for target in targets:
                    srv.WritePosEx(sid, target, spd, acc)
                    deadline = time.time() + 3.0
                    while time.time() < deadline:
                        moving, r, _ = srv.IsMoving(sid)
                        if r == COMM_SUCCESS and not moving:
                            break
                        time.sleep(0.05)

                    actual, r, _ = srv.ReadPos(sid)
                    if r == COMM_SUCCESS and actual is not None:
                        error = abs(actual - target)
                        passed = error <= tol
                    else:
                        actual = None
                        error = None
                        passed = False

                    results.append(
                        {
                            "Joint": sid,
                            "Target": target,
                            "Actual": actual if actual is not None else "ERR",
                            "Error": error if error is not None else "N/A",
                            "Pass": "OK" if passed else "FAIL",
                        }
                    )
                    done += 1
                    progress.progress(min(done / total, 1.0))
    except Exception as exc:
        st.error(str(exc))
        return

    if results:
        st.subheader("Sweep Results")
        st.dataframe(pd.DataFrame(results), width="stretch")
        failed = [r for r in results if r["Pass"] == "FAIL"]
        if failed:
            st.warning(f"{len(failed)} point(s) failed the +/-{tol}-count threshold.")
        else:
            st.success("All sweep points passed.")


# ---------------------------------------------------------------------------
# Tab 9: Visual Servoing
# ---------------------------------------------------------------------------


def _tab_visual_servoing(device: str, baud: int) -> None:
    """Live camera feed with ArUco marker detection and joint command panel."""
    ss = st.session_state
    st.header("Visual Servoing Debug")

    if not _CV2_AVAILABLE:
        st.error(
            "OpenCV is required for this tab. "
            "Install it with: `pip install opencv-contrib-python-headless`"
        )
        return

    left_col, right_col = st.columns([3, 2])

    with left_col:
        st.subheader("Camera Feed")
        cam_idx = st.number_input(
            "Camera index", min_value=0, max_value=8, value=int(ss.get("vs_cam_idx", 0)),
            key="vs_cam_idx_input",
        )
        ss["vs_cam_idx"] = cam_idx

        detect_markers = st.checkbox(
            "Detect ArUco markers", value=bool(ss.get("vs_detect", True)), key="vs_detect_cb"
        )
        ss["vs_detect"] = detect_markers

        aruco_dict_name = st.selectbox(
            "ArUco dictionary",
            options=["DICT_4X4_50", "DICT_4X4_100", "DICT_5X5_50", "DICT_6X6_50"],
            index=0,
            key="vs_aruco_dict",
        )

        freeze = st.button("Freeze Frame", key="vs_freeze")
        if freeze:
            ss["vs_frozen"] = True
        if st.button("Unfreeze", key="vs_unfreeze"):
            ss["vs_frozen"] = False

        frame_placeholder = st.empty()
        marker_info_ph = st.empty()

        if not ss.get("vs_frozen", False):
            try:
                cap = cv2.VideoCapture(int(cam_idx))
                ok, frame = cap.read()
                cap.release()
                if not ok or frame is None:
                    frame_placeholder.warning(f"Camera {cam_idx} not available or returned no frame.")
                    frame = None
                else:
                    ss["vs_last_frame"] = frame
            except Exception as exc:
                frame_placeholder.error(str(exc))
                frame = None
        else:
            frame = ss.get("vs_last_frame")
            st.caption("Frame frozen")

        if frame is not None:
            display = frame.copy()
            marker_rows: List[dict] = []

            if detect_markers:
                try:
                    aruco_dict = cv2.aruco.getPredefinedDictionary(
                        getattr(cv2.aruco, aruco_dict_name)
                    )
                    params = cv2.aruco.DetectorParameters()
                    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
                    corners, ids, _ = detector.detectMarkers(frame)
                    if ids is not None:
                        cv2.aruco.drawDetectedMarkers(display, corners, ids)
                        for i, mid in enumerate(ids.flatten()):
                            c = corners[i][0]
                            cx = int(c[:, 0].mean())
                            cy = int(c[:, 1].mean())
                            marker_rows.append({"ID": int(mid), "Center X": cx, "Center Y": cy})
                            cv2.putText(
                                display, f"ID {mid}",
                                (cx - 10, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                            )
                except Exception as exc:
                    st.caption(f"ArUco error: {exc}")

            # Convert BGR -> RGB for st.image
            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            frame_placeholder.image(rgb, channels="RGB", width="stretch")

            if marker_rows:
                marker_info_ph.dataframe(
                    pd.DataFrame(marker_rows), width="stretch", hide_index=True
                )
            elif detect_markers:
                marker_info_ph.caption("No markers detected.")

    with right_col:
        st.subheader("Joint State")
        # Show latest positions from stream history
        hist = ss.get("stream_history", deque())
        latest_pos: Dict[int, int] = {}
        for row in hist:
            latest_pos[row["id"]] = row["pos"]
        if latest_pos:
            for sid in sorted(latest_pos):
                st.metric(f"J{sid} Position", f"{latest_pos[sid]} ticks")
        else:
            st.info("No telemetry yet. Start Live Telemetry to see joint state.")

        st.subheader("Command")
        vs_ids_raw = st.text_input("Joints to command", value="1-6", key="vs_ids")
        st.caption("Per-joint target position (ticks)")
        vs_cmds: Dict[int, int] = {}
        try:
            vs_ids = _parse_ids(vs_ids_raw)
        except Exception:
            vs_ids = list(SOARM100_IDS)
        for sid in vs_ids:
            default_pos = latest_pos.get(sid, 2048)
            vs_cmds[sid] = st.slider(
                f"J{sid}", 0, 4095, value=default_pos, key=f"vs_pos_{sid}"
            )
        vs_speed = st.number_input("Speed (ticks/s)", 50, 3000, value=300, key="vs_speed")
        vs_acc = st.number_input("Acc", 10, 254, value=50, key="vs_acc")

        if st.button("Send All Joints", type="primary", key="vs_send"):
            try:
                with _bus(device, baud) as srv:
                    srv.groupSyncWrite.clearParam()
                    for sid, pos in vs_cmds.items():
                        srv.SyncWritePosEx(sid, pos, int(vs_speed), int(vs_acc))
                    srv.groupSyncWrite.txPacket()
                st.success("Commands sent.")
            except Exception as exc:
                st.error(str(exc))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    st.set_page_config(
        page_title="STServo Dashboard",
        layout="wide",
        initial_sidebar_state="expanded",
    )
    _init()
    ss = st.session_state

    # Sidebar: shared connection settings
    with st.sidebar:
        st.title("STServo Dashboard")
        st.header("Connection")
        if st.button("Refresh Ports", key="sb_refresh"):
            ss["ports"] = get_available_ports()

        ports: List[Tuple[str, str]] = ss.get("ports", [])
        if ports:
            st.markdown("**Detected ports**")
            for dev, desc in ports:
                st.write(f"`{dev}` -- {desc}" if desc else f"`{dev}`")
            default_dev = ports[0][0]
        else:
            st.info("No USB serial ports detected.")
            default_dev = "/dev/ttyUSB0"

        device: str = st.text_input(
            "Serial device",
            value=ss.get("_device") or default_dev,
            key="sb_device",
        )
        ss["_device"] = device

        baud: int = int(
            st.number_input(
                "Baud rate",
                value=ss.get("_baud", 1_000_000),
                step=1_000,
                min_value=1,
                key="sb_baud",
            )
        )
        ss["_baud"] = baud

        scan_range: str = st.text_input("Scan range", value="1-6", key="sb_scan")
        unlock: bool = st.checkbox("Unlock EEPROM before changes", key="sb_unlock")
        lock: bool = st.checkbox("Lock EEPROM after changes", value=True, key="sb_lock")

        st.divider()
        any_active = (
            ss.get("telemetry_active")
            or ss.get("recording")
            or ss.get("health_active")
        )
        if any_active:
            interval_ms = ss.get("stream_interval_ms", 200)
            st.info(f"Polling every {interval_ms} ms")
            if st.button("Stop All", key="sb_stop_all"):
                ss["telemetry_active"] = False
                ss["recording"] = False
                ss["health_active"] = False
                st.rerun()

        st.divider()
        st.subheader("Torque")
        _torque_ids_raw = st.text_input(
            "Joint IDs", value="1-6", key="sb_torque_ids",
            help="IDs to enable/disable torque.",
        )
        try:
            _torque_ids = _parse_ids(_torque_ids_raw)
        except Exception:
            _torque_ids = list(SOARM100_IDS)
        _tb1, _tb2 = st.columns(2)
        if _tb1.button("OFF", key="sb_toff", width="stretch"):
            try:
                with _bus(device, baud) as _srv:
                    for _sid in _torque_ids:
                        write1(_srv, _sid, STS_TORQUE_ENABLE, 0, "torque off")
                st.success("Torque OFF")
            except Exception as _exc:
                st.error(str(_exc))
        if _tb2.button("ON", key="sb_ton", width="stretch"):
            try:
                with _bus(device, baud) as _srv:
                    for _sid in _torque_ids:
                        write1(_srv, _sid, STS_TORQUE_ENABLE, 1, "torque on")
                st.success("Torque ON")
            except Exception as _exc:
                st.error(str(_exc))

    # Tab layout
    tabs = st.tabs(
        [
            "Homing Wizard",
            "Command",
            "Calibration",
            "Inspector & Telemetry",
            "Recorder",
            "Health",
            "Config",
            "Visual Servoing",
        ]
    )

    with tabs[0]:
        _tab_homing(device, baud)
    with tabs[1]:
        _tab_command(device, baud)
    with tabs[2]:
        _tab_calibration(device, baud, scan_range, unlock, lock)
    with tabs[3]:
        _tab_inspector_telemetry(device, baud, scan_range)
    with tabs[4]:
        _tab_recorder(device, baud)
    with tabs[5]:
        _tab_health()
    with tabs[6]:
        _tab_config(device, baud, scan_range)
    with tabs[7]:
        _tab_visual_servoing(device, baud)

    # Background polling + auto-rerun loop
    _maybe_poll(device, baud)
    if ss.get("telemetry_active") or ss.get("recording") or ss.get("health_active"):
        interval_s = ss.get("stream_interval_ms", 200) / 1000.0
        time.sleep(interval_s)
        st.rerun()


if __name__ == "__main__":
    main()
