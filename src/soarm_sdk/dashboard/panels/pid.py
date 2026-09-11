"""PID Tuning: read/write STS3215 P/D/I EEPROM gains, step-response chart."""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, Optional

import numpy as np

from ... import COMM_SUCCESS, STS_LOCK, STS_TORQUE_ENABLE, write1
from ...protocol.registers import STS_D_COEF, STS_I_COEF, STS_P_COEF
from ..app import Panel
from ..context import DashboardContext

__all__ = ["build_pid_panel"]

_STEP_POLL_S = 0.05  # 20 Hz position polling during step response


def build_pid_panel() -> Panel:
    return Panel("PID Tuning", _build_pid)


def _compute_step_metrics(
    t_samples: List[float],
    actual_samples: List[float],
    start_pos: float,
    target: float,
) -> str:
    delta = target - start_pos
    if abs(delta) < 1e-6:
        return "*Target equals start position — no step to measure.*"

    t = np.array(t_samples)
    a = np.array(actual_samples)

    # Rise time: 10% -> 90% of the way from start to target.
    frac = (a - start_pos) / delta
    rise_lo = np.argmax(frac >= 0.1) if np.any(frac >= 0.1) else None
    rise_hi = np.argmax(frac >= 0.9) if np.any(frac >= 0.9) else None
    rise_str = (
        f"{t[rise_hi] - t[rise_lo]:.3f} s"
        if rise_lo is not None and rise_hi is not None and rise_hi > rise_lo
        else "n/a"
    )

    # Overshoot: how far past target, as a % of the step size.
    if delta > 0:
        overshoot = max(0.0, (a.max() - target) / delta * 100.0)
    else:
        overshoot = max(0.0, (target - a.min()) / delta * 100.0)

    # Settling time: last sample outside ±2% of the step that never returns.
    band = 0.02 * abs(delta)
    outside = np.where(np.abs(a - target) > band)[0]
    settle_str = f"{t[outside[-1]]:.3f} s" if len(outside) else f"{t[0]:.3f} s"

    return (
        f"**Rise time (10–90%):** {rise_str}  \n"
        f"**Overshoot:** {overshoot:.1f}%  \n"
        f"**Settling time (±2%):** {settle_str}"
    )


def _build_pid(server: Any, ctx: DashboardContext) -> None:
    import viser.uplot as _uplot

    server.gui.add_markdown("## PID Gain Tuning")
    server.gui.add_markdown(
        "Read and write the **P / D / I** coefficients stored in EEPROM.  \n"
        "Typical defaults: P=32, D=32, I=0.  \n"
        "⚠ Writing modifies EEPROM — changes persist after power-off."
    )

    servo_id_h = server.gui.add_number("Servo ID", initial_value=1, min=1, max=253, step=1)

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

    with server.gui.add_folder("Step Response"):
        server.gui.add_markdown(
            "Command a position step and record the 20 Hz response.  \n"
            "Set the target within the servo's homed range."
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
        step_stop_btn = server.gui.add_button("Stop", color="red", disabled=True)
        step_status_md = server.gui.add_markdown("*Configure above and press Send Step.*")

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

    _step_stop: Dict[str, Any] = {"event": threading.Event()}

    @read_btn.on_click
    def _do_read(_: Any) -> None:
        sid = int(servo_id_h.value)
        try:
            with ctx.bus() as srv:

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
            new_p_h.value = float(p_val)
            new_d_h.value = float(d_val)
            new_i_h.value = float(i_val)
            pid_log_md.content = f"**Read OK** — J{sid}: P={p_val}  D={d_val}  I={i_val}"
        except Exception as exc:
            pid_log_md.content = f"**Read error (J{sid}):** {exc}"

    @write_pid_btn.on_click
    def _do_write(_: Any) -> None:
        sid = int(servo_id_h.value)
        p = int(new_p_h.value)
        d = int(new_d_h.value)
        i = int(new_i_h.value)
        try:
            with ctx.bus() as srv:
                write1(srv, sid, STS_LOCK, 0, "unlock EEPROM")
                write1(srv, sid, STS_P_COEF, p, "P gain")
                write1(srv, sid, STS_D_COEF, d, "D gain")
                write1(srv, sid, STS_I_COEF, i, "I gain")
                write1(srv, sid, STS_LOCK, 1, "lock EEPROM")
            read_p_md.content = f"**P:** {p}"
            read_d_md.content = f"**D:** {d}"
            read_i_md.content = f"**I:** {i}"
            pid_log_md.content = f"**Write OK** — J{sid}: P={p}  D={d}  I={i} (saved to EEPROM)"
        except Exception as exc:
            pid_log_md.content = f"**Write error (J{sid}):** {exc}"

    @step_btn.on_click
    def _do_step(_: Any) -> None:
        sid = int(servo_id_h.value)
        target = int(step_target_h.value)
        speed = int(step_speed_h.value)
        acc = int(step_acc_h.value)
        duration = float(step_dur_h.value)

        stop_ev = threading.Event()
        _step_stop["event"] = stop_ev
        step_btn.disabled = True
        step_stop_btn.disabled = False
        step_status_md.content = f"*Running step → J{sid} target={target} ticks…*"
        metrics_md.content = ""

        def _run() -> None:
            t_samples: List[float] = []
            actual_samples: List[float] = []
            start_pos: Optional[float] = None
            try:
                with ctx.bus() as srv:
                    p0, r0, _ = srv.ReadPos(sid)
                    start_pos = float(p0) if r0 == COMM_SUCCESS else None

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

                        if len(t_samples) >= 2:
                            t_np = np.array(t_samples, dtype=np.float64)
                            a_np = np.array(actual_samples, dtype=np.float64)
                            r_np = np.full_like(t_np, float(target))
                            step_chart.data = (t_np, r_np, a_np)

                        if now >= duration:
                            break

                        sleep_s = _STEP_POLL_S - (time.monotonic() - t_loop)
                        if sleep_s > 0:
                            stop_ev.wait(timeout=sleep_s)

            except Exception as exc:
                step_status_md.content = f"**Step error**: {exc}"
                step_btn.disabled = False
                step_stop_btn.disabled = True
                return

            n = len(t_samples)
            if n >= 2:
                t_np = np.array(t_samples, dtype=np.float64)
                a_np = np.array(actual_samples, dtype=np.float64)
                r_np = np.full_like(t_np, float(target))
                step_chart.data = (t_np, r_np, a_np)

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
                step_status_md.content = "*Step complete — insufficient samples for metrics.*"

            step_btn.disabled = False
            step_stop_btn.disabled = True

        threading.Thread(target=_run, daemon=True).start()

    @step_stop_btn.on_click
    def _do_stop(_: Any) -> None:
        _step_stop["event"].set()
        step_stop_btn.disabled = True
