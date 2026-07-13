"""Monitor: live position/speed charts, health table, and servo inspector."""

from __future__ import annotations

import time
from collections import deque
from typing import Any, Dict

import numpy as np

from ...conversions import ticks_to_radians
from ..app import Panel
from ..context import DashboardContext, JointState
from ..fk import SOARM100_JOINT_NAMES

__all__ = ["build_monitor_panel"]

_CHART_WINDOW = 200  # rolling history length (200 x 100 ms = 20 s)
_CHART_COLORS = ["#e74c3c", "#3498db", "#2ecc71", "#f39c12", "#9b59b6", "#1abc9c"]


def build_monitor_panel() -> Panel:
    """Combined Monitor tab: live charts + table, health, and servo inspector."""
    handles: Dict[str, Any] = {}
    return Panel(
        "Monitor",
        lambda server, ctx: _build_monitor(server, ctx, handles),
        lambda ctx: _on_tick(ctx, handles),
    )


def _joint_name(sid: int) -> str:
    return SOARM100_JOINT_NAMES[sid - 1] if sid <= len(SOARM100_JOINT_NAMES) else str(sid)


def _format_telem_md(state: JointState, joint_ids: list) -> str:
    if not state.connected:
        err = state.poll_error or "no hardware"
        return f"**Status**: Disconnected — *{err}*"

    lines = [
        f"**Poll #{state.poll_count}**\n",
        "| Joint | Name | Pos (ticks) | Pos (rad) | Speed (t/s) |",
        "|-------|------|:-----------:|:---------:|:-----------:|",
    ]
    for sid in joint_ids:
        pos = state.positions.get(sid)
        spd = state.speeds.get(sid)
        rad = f"{ticks_to_radians(pos):.3f}" if pos is not None else "—"
        lines.append(
            f"| J{sid} | {_joint_name(sid)} | {pos if pos is not None else '—'}"
            f" | {rad} | {spd if spd is not None else '—'} |"
        )
    return "\n".join(lines)


def _format_health_md(state: JointState, joint_ids: list) -> str:
    if not state.temps and not state.currents:
        return "*Health data: updates every 5 polls…*"

    lines = ["| Joint | Temp (°C) | Current (mA) |", "|-------|:---------:|:------------:|"]
    for sid in joint_ids:
        temp = state.temps.get(sid)
        curr = state.currents.get(sid)
        t_str = f"{temp:.1f}" if temp is not None else "—"
        c_str = f"{curr:.0f}" if curr is not None else "—"
        lines.append(f"| J{sid} | {t_str} | {c_str} |")
    return "\n".join(lines)


def _build_monitor(server: Any, ctx: DashboardContext, handles: Dict[str, Any]) -> None:
    import viser.uplot as _uplot

    from ... import read_servo_diagnostics

    joint_ids = ctx.joint_ids
    colors = [_CHART_COLORS[i % len(_CHART_COLORS)] for i in range(len(joint_ids))]

    server.gui.add_markdown("## Monitor")

    with server.gui.add_folder("Live Position & Speed"):
        init_t = np.linspace(0.0, _CHART_WINDOW * 0.1, _CHART_WINDOW)
        zeros = np.zeros(_CHART_WINDOW)

        pos_chart = server.gui.add_uplot(
            data=(init_t, *[zeros.copy() for _ in joint_ids]),
            series=(
                _uplot.Series(label="time"),
                *[
                    _uplot.Series(label=f"J{sid}", stroke=colors[i], width=2)
                    for i, sid in enumerate(joint_ids)
                ],
            ),
            title="Position (ticks)",
            axes=(_uplot.Axis(label="time (s)"), _uplot.Axis(label="ticks", side=3)),
            legend=_uplot.Legend(show=True),
            aspect=2.5,
        )
        spd_chart = server.gui.add_uplot(
            data=(init_t, *[zeros.copy() for _ in joint_ids]),
            series=(
                _uplot.Series(label="time"),
                *[
                    _uplot.Series(label=f"J{sid}", stroke=colors[i], width=2)
                    for i, sid in enumerate(joint_ids)
                ],
            ),
            title="Speed (ticks/s)",
            axes=(_uplot.Axis(label="time (s)"), _uplot.Axis(label="ticks/s", side=3)),
            legend=_uplot.Legend(show=True),
            aspect=2.5,
        )
        telem_md = server.gui.add_markdown("*Waiting for hardware data…*")

    with server.gui.add_folder("Health (temp / current)"):
        server.gui.add_markdown("Sampled every 5 poll cycles (~1 s at 200 ms interval).")
        health_md = server.gui.add_markdown("*Waiting for hardware data…*")

    with server.gui.add_folder("Servo Inspector"):
        server.gui.add_markdown(
            "Full register snapshot: position, speed, load, voltage, "
            "current, temperature, mode, acceleration, correction, status."
        )
        sid_h = server.gui.add_number("Servo ID", initial_value=1, min=1, max=253, step=1)
        read_btn = server.gui.add_button("Read Registers", color="blue")
        insp_html = server.gui.add_html("<i>Select a servo ID and press Read.</i>")

    def _diag_to_html(sid: int, details: Dict[str, Any]) -> str:
        errors: Dict[str, Any] = details.pop("_errors", {}) if "_errors" in details else {}
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
    def _do_read(_: Any) -> None:
        sid = int(sid_h.value)
        insp_html.content = f"<i>Reading J{sid}…</i>"
        try:
            diag = read_servo_diagnostics(ctx.device_h.value, int(ctx.baud_h.value), [sid])
            details = dict(diag.get(sid, {}))
            if not details:
                insp_html.content = f"<b>No response from J{sid}.</b>"
                return
            insp_html.content = _diag_to_html(sid, details)
        except Exception as exc:
            insp_html.content = f"<b style='color:red'>Error reading J{sid}: {exc}</b>"

    handles["telem_md"] = telem_md
    handles["health_md"] = health_md
    handles["pos_chart"] = pos_chart
    handles["spd_chart"] = spd_chart
    handles["joint_ids"] = joint_ids
    handles["t0"] = time.monotonic()
    handles["t_buf"] = deque([float(i) * 0.1 for i in range(_CHART_WINDOW)], maxlen=_CHART_WINDOW)
    handles["pos_bufs"] = {sid: deque([0.0] * _CHART_WINDOW, maxlen=_CHART_WINDOW) for sid in joint_ids}
    handles["spd_bufs"] = {sid: deque([0.0] * _CHART_WINDOW, maxlen=_CHART_WINDOW) for sid in joint_ids}


def _on_tick(ctx: DashboardContext, handles: Dict[str, Any]) -> None:
    if "telem_md" not in handles:
        return  # build() hasn't run yet

    with ctx.lock:
        snap = JointState(
            positions=dict(ctx.state.positions),
            speeds=dict(ctx.state.speeds),
            temps=dict(ctx.state.temps),
            currents=dict(ctx.state.currents),
            connected=ctx.state.connected,
            poll_error=ctx.state.poll_error,
            poll_count=ctx.state.poll_count,
        )

    joint_ids = handles["joint_ids"]
    handles["telem_md"].content = _format_telem_md(snap, joint_ids)
    handles["health_md"].content = _format_health_md(snap, joint_ids)

    now = time.monotonic() - handles["t0"]
    t_buf = handles["t_buf"]
    pos_bufs = handles["pos_bufs"]
    spd_bufs = handles["spd_bufs"]
    t_buf.append(now)
    for sid in joint_ids:
        pos_bufs[sid].append(float(snap.positions.get(sid) or 0))
        spd_bufs[sid].append(float(snap.speeds.get(sid) or 0))

    t_arr = np.array(t_buf, dtype=np.float64)
    handles["pos_chart"].data = (
        t_arr,
        *[np.array(pos_bufs[sid], dtype=np.float64) for sid in joint_ids],
    )
    handles["spd_chart"].data = (
        t_arr,
        *[np.array(spd_bufs[sid], dtype=np.float64) for sid in joint_ids],
    )
