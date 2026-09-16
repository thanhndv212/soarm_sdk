"""PID Tuning: read/write STS3215 P/D/I EEPROM gains, step-response chart,
and closed-loop auto-tuning. See ``docs/pid_autotune_plan.md`` for the design.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, Optional

import numpy as np

from ... import COMM_SUCCESS, STS_TORQUE_ENABLE, write1
from ...tuning.acceptance import PIDAcceptanceCriteria
from ...tuning.gains_io import Gains, read_gains, write_gains
from ...tuning.metrics import StepMetrics, compute_step_metrics
from ...tuning.provenance import TuningRecord, append_record
from ...tuning.search import SearchBounds, SearchResult, coordinate_descent_tune
from ...tuning.step_test import Sample, run_step_test
from ...tuning.watchdog import Watchdog, WatchdogLimits
from ..app import Panel
from ..context import DashboardContext

__all__ = ["build_pid_panel"]

_STEP_POLL_S = 0.05  # 20 Hz position polling during step response/tuning trials


def build_pid_panel() -> Panel:
    return Panel("PID Tuning", _build_pid)


def _format_metrics_md(metrics: StepMetrics) -> str:
    rise_str = f"{metrics.rise_time_s:.3f} s" if metrics.rise_time_s is not None else "n/a"
    current_str = (
        f"{metrics.peak_current_mA:.0f} mA" if metrics.peak_current_mA is not None else "n/a"
    )
    temp_str = (
        f"{metrics.peak_temperature_C:.1f} C" if metrics.peak_temperature_C is not None else "n/a"
    )
    lines = [
        f"**Rise time (10-90%):** {rise_str}  ",
        f"**Overshoot:** {metrics.overshoot_pct:.1f}%  ",
        f"**Settling time (+/-2%):** {metrics.settling_time_s:.3f} s  ",
        f"**Steady-state error:** {metrics.steady_state_error_ticks:.1f} ticks  ",
        f"**Oscillation count:** {metrics.oscillation_count}  ",
        f"**Peak current:** {current_str}  ",
        f"**Peak temperature:** {temp_str}",
    ]
    if metrics.aborted_reason is not None:
        lines.append(f"**ABORTED:** {metrics.aborted_reason}")
    return "\n".join(lines)


def _read_sample(srv: Any, sid: int, t0: float) -> Optional[Sample]:
    """One position+current+temperature reading, for the tuning loop.

    Three round trips per sample (position, current, temperature) rather
    than a single sync-read block: this loop only ever watches one servo at
    a time, so the per-transaction overhead the dashboard's Monitor tab
    avoids with GroupSyncRead doesn't apply here, and it keeps this code
    independent of whether streaming mode is active.
    """
    p, r, _ = srv.ReadPos(sid)
    if r != COMM_SUCCESS:
        return None
    current: Optional[float] = None
    temperature: Optional[float] = None
    try:
        c, rc, _ = srv.ReadCurrent(sid)
        if rc == COMM_SUCCESS:
            current = abs(float(c))
    except Exception:
        pass
    try:
        t, rt, _ = srv.ReadTemperature(sid)
        if rt == COMM_SUCCESS:
            temperature = float(t)
    except Exception:
        pass
    return Sample(
        t_s=time.monotonic() - t0,
        position_ticks=float(p),
        current_mA=current,
        temperature_C=temperature,
    )


def _build_pid(server: Any, ctx: DashboardContext) -> None:
    import viser.uplot as _uplot

    server.gui.add_markdown("## PID Gain Tuning")
    server.gui.add_markdown(
        "Read and write the **P / D / I** coefficients stored in EEPROM.  \n"
        "Typical defaults: P=32, D=32, I=0.  \n"
        "Writing modifies EEPROM — changes persist after power-off."
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

    with server.gui.add_folder("Safety limits (applied to every trial below)"):
        server.gui.add_markdown(
            "A trial is aborted immediately — gains restored — if any of "
            "these are crossed mid-trial. Set above the acceptance "
            "thresholds below so a merely-bad trial still finishes and "
            "reports real numbers, while a dangerous one is cut short."
        )
        current_limit_h = server.gui.add_number(
            "Current limit (mA)", initial_value=1500.0, min=100.0, max=6000.0, step=50.0
        )
        temp_limit_h = server.gui.add_number(
            "Temperature limit (C)", initial_value=65.0, min=30.0, max=100.0, step=1.0
        )
        osc_hard_limit_h = server.gui.add_number(
            "Oscillation hard limit", initial_value=8, min=1, max=50, step=1
        )

    with server.gui.add_folder("Step Response"):
        server.gui.add_markdown(
            "Command a position step and record the response.  \n"
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

    with server.gui.add_folder("Auto-Tune"):
        server.gui.add_markdown(
            "Coordinate-descent search over (P, D, I) using the Step "
            "Response settings above as each trial's step. Every trial "
            "writes candidate gains to EEPROM to test them — this is not "
            "a preview. See `docs/pid_autotune_plan.md`."
        )
        with server.gui.add_folder("Search bounds (+/- around current gains)"):
            p_window_h = server.gui.add_number("P window", initial_value=32, min=1, max=127, step=1)
            d_window_h = server.gui.add_number("D window", initial_value=32, min=1, max=127, step=1)
            i_window_h = server.gui.add_number("I window", initial_value=16, min=0, max=127, step=1)
            max_trials_h = server.gui.add_number("Max trials", initial_value=30, min=1, max=100, step=1)
            initial_step_h = server.gui.add_number("Initial step size", initial_value=16, min=1, max=64, step=1)

        with server.gui.add_folder("Acceptance criteria"):
            overshoot_max_h = server.gui.add_number(
                "Overshoot max (%)", initial_value=15.0, min=0.1, max=200.0, step=0.5
            )
            settle_max_h = server.gui.add_number(
                "Settling time max (s)", initial_value=1.0, min=0.05, max=10.0, step=0.05
            )
            ss_error_max_h = server.gui.add_number(
                "Steady-state error max (ticks)", initial_value=10.0, min=0.1, max=200.0, step=0.5
            )
            osc_max_h = server.gui.add_number(
                "Oscillation count max", initial_value=2, min=0, max=20, step=1
            )
            repeat_trials_h = server.gui.add_number(
                "Repeat trials to validate", initial_value=3, min=1, max=10, step=1
            )

        autotune_btn = server.gui.add_button("Run Auto-Tune", color="green")
        autotune_stop_btn = server.gui.add_button("Stop", color="red", disabled=True)
        restore_btn = server.gui.add_button("Restore Pre-Tune Gains", color="orange", disabled=True)
        autotune_status_md = server.gui.add_markdown("*Configure above and press Run Auto-Tune.*")
        autotune_report_md = server.gui.add_markdown("")

    _step_stop: Dict[str, Any] = {"event": threading.Event()}
    _autotune_stop: Dict[str, Any] = {"event": threading.Event()}
    _pre_tune_gains: Dict[str, Optional[Gains]] = {"value": None}

    @read_btn.on_click
    def _do_read(_: Any) -> None:
        sid = int(servo_id_h.value)
        try:
            with ctx.bus() as srv:
                gains = read_gains(srv, sid)
            read_p_md.content = f"**P:** {gains.p}"
            read_d_md.content = f"**D:** {gains.d}"
            read_i_md.content = f"**I:** {gains.i}"
            new_p_h.value = float(gains.p)
            new_d_h.value = float(gains.d)
            new_i_h.value = float(gains.i)
            pid_log_md.content = f"**Read OK** — J{sid}: P={gains.p}  D={gains.d}  I={gains.i}"
        except Exception as exc:
            pid_log_md.content = f"**Read error (J{sid}):** {exc}"

    @write_pid_btn.on_click
    def _do_write(_: Any) -> None:
        sid = int(servo_id_h.value)
        try:
            gains = Gains(p=int(new_p_h.value), d=int(new_d_h.value), i=int(new_i_h.value))
        except ValueError as exc:
            pid_log_md.content = f"**Invalid gains:** {exc}"
            return
        try:
            with ctx.bus() as srv:
                write_gains(srv, sid, gains)
            read_p_md.content = f"**P:** {gains.p}"
            read_d_md.content = f"**D:** {gains.d}"
            read_i_md.content = f"**I:** {gains.i}"
            pid_log_md.content = (
                f"**Write OK** — J{sid}: P={gains.p}  D={gains.d}  I={gains.i} (saved to EEPROM)"
            )
        except Exception as exc:
            pid_log_md.content = f"**Write error (J{sid}):** {exc}"

    def _watchdog_limits() -> WatchdogLimits:
        return WatchdogLimits(
            current_max_mA=float(current_limit_h.value),
            temperature_max_C=float(temp_limit_h.value),
            oscillation_hard_max=int(osc_hard_limit_h.value),
        )

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
        step_status_md.content = f"*Running step -> J{sid} target={target} ticks…*"
        metrics_md.content = ""

        def _run() -> None:
            try:
                with ctx.bus() as srv:
                    start_p, r0, _ = srv.ReadPos(sid)
                    if r0 != COMM_SUCCESS:
                        raise IOError("failed to read starting position")
                    start_ticks = float(start_p)

                    def _on_sample(partial):
                        t_np = np.array(partial.t_s, dtype=np.float64)
                        r_np = np.full_like(t_np, float(target))
                        a_np = np.array(partial.position_ticks, dtype=np.float64)
                        step_chart.data = (t_np, r_np, a_np)

                    t0 = time.monotonic()

                    def command():
                        write1(srv, sid, STS_TORQUE_ENABLE, 1, "torque on")
                        srv.WritePosEx(sid, target, speed, acc)

                    watchdog = Watchdog(_watchdog_limits(), target_ticks=float(target))

                    response = run_step_test(
                        command=command,
                        read_sample=lambda: _read_sample(srv, sid, t0),
                        start_ticks=start_ticks,
                        target_ticks=float(target),
                        duration_s=duration,
                        poll_interval_s=_STEP_POLL_S,
                        watchdog=watchdog,
                        should_stop=stop_ev.is_set,
                        on_sample=_on_sample,
                    )
            except Exception as exc:
                step_status_md.content = f"**Step error**: {exc}"
                step_btn.disabled = False
                step_stop_btn.disabled = True
                return

            n = len(response.t_s)
            if n >= 3:
                try:
                    metrics = compute_step_metrics(response)
                    metrics_md.content = _format_metrics_md(metrics)
                except ValueError as exc:
                    metrics_md.content = f"*{exc}*"
                actual_hz = n / max(response.t_s[-1], 1e-3)
                status = f"**Done** — J{sid}: {int(start_ticks)}->{target} ticks, {n} samples @ ~{actual_hz:.0f} Hz"
                if response.aborted_reason is not None:
                    status = f"**ABORTED** — {response.aborted_reason}"
                step_status_md.content = status
            else:
                step_status_md.content = "*Step complete — insufficient samples for metrics.*"

            step_btn.disabled = False
            step_stop_btn.disabled = True

        threading.Thread(target=_run, daemon=True).start()

    @step_stop_btn.on_click
    def _do_stop(_: Any) -> None:
        _step_stop["event"].set()
        step_stop_btn.disabled = True

    def _criteria() -> PIDAcceptanceCriteria:
        return PIDAcceptanceCriteria(
            overshoot_max_pct=float(overshoot_max_h.value),
            settling_time_max_s=float(settle_max_h.value),
            steady_state_error_max_ticks=float(ss_error_max_h.value),
            oscillation_count_max=int(osc_max_h.value),
            current_peak_max_mA=float(current_limit_h.value),
            temperature_max_C=float(temp_limit_h.value),
            repeat_trials=int(repeat_trials_h.value),
        )

    @autotune_btn.on_click
    def _do_autotune(_: Any) -> None:
        sid = int(servo_id_h.value)
        target = int(step_target_h.value)
        speed = int(step_speed_h.value)
        acc = int(step_acc_h.value)
        duration = float(step_dur_h.value)

        try:
            criteria = _criteria()
        except ValueError as exc:
            autotune_status_md.content = f"**Invalid acceptance criteria:** {exc}"
            return

        stop_ev = threading.Event()
        _autotune_stop["event"] = stop_ev
        autotune_btn.disabled = True
        autotune_stop_btn.disabled = False
        restore_btn.disabled = True
        autotune_report_md.content = ""
        autotune_status_md.content = f"*Reading current gains for J{sid}…*"

        def _run() -> None:
            log_lines: List[str] = []

            try:
                with ctx.bus() as srv:
                    before = read_gains(srv, sid)
                    _pre_tune_gains["value"] = before

                    try:
                        bounds = SearchBounds(
                            p_min=max(0, before.p - int(p_window_h.value)),
                            p_max=min(254, before.p + int(p_window_h.value)),
                            d_min=max(0, before.d - int(d_window_h.value)),
                            d_max=min(254, before.d + int(d_window_h.value)),
                            i_min=max(0, before.i - int(i_window_h.value)),
                            i_max=min(254, before.i + int(i_window_h.value)),
                        )
                    except ValueError as exc:
                        autotune_status_md.content = f"**Invalid search bounds:** {exc}"
                        autotune_btn.disabled = False
                        autotune_stop_btn.disabled = True
                        return

                    trial_count = {"n": 0}

                    def run_trial(gains: Gains) -> StepMetrics:
                        if stop_ev.is_set():
                            raise RuntimeError("stopped by operator")
                        trial_count["n"] += 1
                        write_gains(srv, sid, gains)

                        start_p, r0, _ = srv.ReadPos(sid)
                        if r0 != COMM_SUCCESS:
                            raise IOError("failed to read starting position")

                        t0 = time.monotonic()

                        def command():
                            write1(srv, sid, STS_TORQUE_ENABLE, 1, "torque on")
                            srv.WritePosEx(sid, target, speed, acc)

                        watchdog = Watchdog(_watchdog_limits(), target_ticks=float(target))

                        response = run_step_test(
                            command=command,
                            read_sample=lambda: _read_sample(srv, sid, t0),
                            start_ticks=float(start_p),
                            target_ticks=float(target),
                            duration_s=duration,
                            poll_interval_s=_STEP_POLL_S,
                            watchdog=watchdog,
                            should_stop=stop_ev.is_set,
                        )
                        metrics = compute_step_metrics(response)

                        verdict = "PASS" if criteria.evaluate(metrics).ready else "fail"
                        log_lines.append(
                            f"trial {trial_count['n']}: P={gains.p} D={gains.d} I={gains.i} "
                            f"-> overshoot={metrics.overshoot_pct:.1f}% "
                            f"settle={metrics.settling_time_s:.2f}s "
                            f"sse={metrics.steady_state_error_ticks:.1f} "
                            f"osc={metrics.oscillation_count} [{verdict}]"
                        )
                        autotune_status_md.content = "\n\n".join(log_lines[-8:])
                        return metrics

                    result: SearchResult = coordinate_descent_tune(
                        initial=before,
                        bounds=bounds,
                        run_trial=run_trial,
                        criteria=criteria,
                        max_trials=int(max_trials_h.value),
                        initial_step=int(initial_step_h.value),
                    )

                    # Every trial already wrote candidate gains to test them;
                    # make sure EEPROM ends on the best one found, not
                    # whatever the last (possibly worse) trial left behind.
                    write_gains(srv, sid, result.best.gains)

            except Exception as exc:
                autotune_status_md.content = f"**Auto-tune error:** {exc}"
                autotune_btn.disabled = False
                autotune_stop_btn.disabled = True
                restore_btn.disabled = _pre_tune_gains["value"] is None
                return

            best = result.best
            outcome = "VALIDATED" if result.validated else (
                "stopped by operator" if stop_ev.is_set() else "EXHAUSTED (not validated)"
            )
            summary = (
                f"**{outcome}** after {len(result.trials)} trial(s).  \n"
                f"Before: P={before.p} D={before.d} I={before.i}  \n"
                f"Best:   P={best.gains.p} D={best.gains.d} I={best.gains.i}  \n"
                f"Consecutive passes at best: {result.consecutive_passes_at_best}"
            )
            autotune_status_md.content = summary
            if best.report is not None:
                autotune_report_md.content = best.report.as_markdown()

            record = TuningRecord.from_search_result(
                arm_id=str(ctx.calibration.arm_id) if ctx.calibration is not None else "unknown",
                servo_id=sid,
                joint_name=f"servo_{sid}",
                algorithm="coordinate_descent",
                before=before,
                result=result,
            )
            try:
                append_record(record)
            except Exception:
                pass  # provenance is best-effort; never block on it

            autotune_btn.disabled = False
            autotune_stop_btn.disabled = True
            restore_btn.disabled = False

        threading.Thread(target=_run, daemon=True).start()

    @autotune_stop_btn.on_click
    def _do_autotune_stop(_: Any) -> None:
        _autotune_stop["event"].set()
        autotune_stop_btn.disabled = True

    @restore_btn.on_click
    def _do_restore(_: Any) -> None:
        before = _pre_tune_gains["value"]
        if before is None:
            return
        sid = int(servo_id_h.value)
        try:
            with ctx.bus() as srv:
                write_gains(srv, sid, before)
            read_p_md.content = f"**P:** {before.p}"
            read_d_md.content = f"**D:** {before.d}"
            read_i_md.content = f"**I:** {before.i}"
            autotune_status_md.content = (
                f"**Restored** J{sid} to pre-tune gains: "
                f"P={before.p} D={before.d} I={before.i}"
            )
        except Exception as exc:
            autotune_status_md.content = f"**Restore error:** {exc}"
