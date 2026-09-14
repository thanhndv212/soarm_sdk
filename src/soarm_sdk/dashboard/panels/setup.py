"""Hardware setup panels: Start Up, Homing Wizard, Reconfigure.

"Setting up my own hardware" as its own reusable unit — bring a fresh arm
online (discover its port, home it, set IDs/limits) — decoupled from
ongoing-operation panels (monitoring, manual command, PID tuning,
recording) which are a different concern with a different lifecycle.

Each ``build_*_panel()`` function returns a :class:`~soarm_sdk.dashboard.app.Panel`
ready to hand to :meth:`~soarm_sdk.dashboard.app.DashboardApp.register`.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from ...calibration.recentre import centring_offset, encode_ofs
from ...calibration.rom_sweep import (
    WRAP_SUSPECT_TICKS,
    run_rom_sweep,
    simulate_rom_sweep,
)
from ... import (
    STS_ACC,
    STS_LOCK,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_MODE,
    STS_TORQUE_ENABLE,
    COMM_SUCCESS,
    scan_servos,
    get_available_ports,
    read_servo_diagnostics,
    run_calibration,
    write1,
    write2,
)
from ...protocol.registers import STS_OFS_L
from ..fk import SOARM100_JOINT_NAMES
from ..app import Panel
from ..context import DashboardContext

__all__ = [
    "build_startup_panel",
    "build_homing_panel",
    "build_reconfigure_panel",
    "build_all",
]

def build_all(fk_update_fn: Optional[Callable[[Dict[int, int]], None]] = None) -> List[Panel]:
    """Convenience: all three setup panels, in display order."""
    return [
        build_startup_panel(),
        build_homing_panel(fk_update_fn=fk_update_fn),
        build_reconfigure_panel(),
    ]


# ---------------------------------------------------------------------------
# Start Up
# ---------------------------------------------------------------------------


def build_startup_panel() -> Panel:
    """Connection, quick torque, and scan-servos."""
    return Panel("Start Up", _build_startup)


def _build_startup(server: Any, ctx: DashboardContext) -> None:
    server.gui.add_markdown("## Start Up")

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
        connect_btn = server.gui.add_button("Connect & Start Polling", color="green")
        disconnect_btn = server.gui.add_button("Disconnect", color="red")

    with server.gui.add_folder("Quick Torque"):
        torque_ids_h = server.gui.add_text("Joint IDs", initial_value="1-6")
        qt_on = server.gui.add_button("Torque ON", color="blue")
        qt_off = server.gui.add_button("Torque OFF", color="red")
        torque_status_md = server.gui.add_markdown("")

    with server.gui.add_folder("Scan Servos"):
        server.gui.add_markdown(
            "Ping every ID in the given range and list responding servos."
        )
        scan_range_h = server.gui.add_text("Scan range", initial_value="1-10")
        scan_btn = server.gui.add_button("Scan", color="blue")
        scan_md = server.gui.add_markdown("*Press Scan to discover servos.*")

    @connect_btn.on_click
    def _do_connect(_: Any) -> None:
        ctx.start_polling()

    @disconnect_btn.on_click
    def _do_disconnect(_: Any) -> None:
        ctx.stop_polling()

    @qt_on.on_click
    def _do_qt_on(_: Any) -> None:
        try:
            ids = ctx.parse_ids(torque_ids_h.value)
            with ctx.bus() as srv:
                for sid in ids:
                    write1(srv, sid, STS_TORQUE_ENABLE, 1, "torque on")
            torque_status_md.content = f"Torque ON — J{ids}"
        except Exception as exc:
            torque_status_md.content = f"**Error**: {exc}"

    @qt_off.on_click
    def _do_qt_off(_: Any) -> None:
        try:
            ids = ctx.parse_ids(torque_ids_h.value)
            with ctx.bus() as srv:
                for sid in ids:
                    write1(srv, sid, STS_TORQUE_ENABLE, 0, "torque off")
            torque_status_md.content = f"Torque OFF — J{ids}"
        except Exception as exc:
            torque_status_md.content = f"**Error**: {exc}"

    @scan_btn.on_click
    def _do_scan(_: Any) -> None:
        scan_md.content = "*Scanning…*"
        try:
            ids = ctx.parse_ids(scan_range_h.value)
            # Through ctx.bus(), never discover_servos(): in streaming mode
            # the interface owns the port and every other control here
            # borrows its handle. Scan was the one caller left opening its
            # own PortHandler on the same device — and on macOS a second
            # open of a /dev/cu.* node succeeds rather than failing, so the
            # ping went out on a shared UART and the telemetry thread ate
            # the reply. Nothing errored; the scan just found nothing, which
            # reads exactly like an arm that is powered off.
            with ctx.bus() as srv:
                found = scan_servos(srv, ids)
            if found:
                lines = ["| ID | Status |", "|----|--------|"]
                for sid in sorted(found):
                    lines.append(f"| {sid} | ✓ responding |")
                scan_md.content = "\n".join(lines)
            else:
                scan_md.content = "**No servos found** in the scanned range."
        except Exception as exc:
            scan_md.content = f"**Scan error**: {exc}"


def _run_rom_sweep_auto(
    ctx: DashboardContext,
    joint_ids: List[int],
    sweep_speed: int,
    stall_thr: int,
    stall_win: int,
    timeout_s: float,
    max_range_ticks: int = 0,
    log_fn: Callable[[str], None] = print,
) -> Dict[int, dict]:
    """Drive each joint in wheel mode to discover its mechanical limits.

    Thin wrapper around :func:`soarm_sdk.calibration.rom_sweep.run_rom_sweep`
    binding it to this panel's ``ctx.bus()``; see that function for the
    actual sweep/stall-detection logic.
    """
    return run_rom_sweep(
        ctx.bus,
        joint_ids=joint_ids,
        sweep_speed=sweep_speed,
        stall_thr=stall_thr,
        stall_win=stall_win,
        timeout_s=timeout_s,
        max_range_ticks=max_range_ticks,
        log_fn=log_fn,
    )


def _run_rom_sweep_sim(
    ctx: DashboardContext,
    joint_ids: List[int],
    sweep_speed: int,
    timeout_s: float,
    max_range_ticks: int = 0,
    log_fn: Callable[[str], None] = print,
    fk_update_fn: Optional[Callable[[Dict[int, int]], None]] = None,
) -> Dict[int, dict]:
    """Simulate a ROM sweep without any hardware.

    Thin wrapper around :func:`soarm_sdk.calibration.rom_sweep.simulate_rom_sweep`,
    snapshotting this panel's live joint positions as the sweep's starting point.
    """
    with ctx.lock:
        current = dict(ctx.state.positions)
    return simulate_rom_sweep(
        joint_ids=joint_ids,
        sweep_speed=sweep_speed,
        timeout_s=timeout_s,
        max_range_ticks=max_range_ticks,
        log_fn=log_fn,
        fk_update_fn=fk_update_fn,
        current_positions=current,
    )


# ---------------------------------------------------------------------------
# Homing Wizard
# ---------------------------------------------------------------------------


def build_homing_panel(
    fk_update_fn: Optional[Callable[[Dict[int, int]], None]] = None,
) -> Panel:
    """Automatic ROM sweep or manual limit recording.

    Parameters
    ----------
    fk_update_fn:
        Optional callback ``(positions) -> None`` used to animate the 3-D
        scene during a dry-run/simulated sweep. Pass
        ``DashboardApp.fk_update`` when registering.
    """

    def _build(server: Any, ctx: DashboardContext) -> None:
        _build_homing(server, ctx, fk_update_fn=fk_update_fn)

    return Panel("Homing Wizard", _build)


def _build_homing(
    server: Any,
    ctx: DashboardContext,
    fk_update_fn: Optional[Callable[[Dict[int, int]], None]] = None,
    *,
    heading: bool = True,
    on_endpoints: Optional[Callable[[Dict[str, tuple], bool], None]] = None,
    handles: Optional[Dict[str, Any]] = None,
) -> None:
    """Build the ROM-measurement controls.

    ``heading`` renders the standalone "Homing Wizard" title. The Calibration
    tab embeds these controls inside its own step folder and passes
    ``heading=False``: a second top-level title nested there reads as a
    different tool rather than as that step's contents.

    ``on_endpoints(by_joint_name, simulated)`` is called whenever a complete
    set of travel endpoints is produced, by either mode. Without it these
    controls write only servo EEPROM and ``soarm100_rom.json`` — which is why
    measuring travel here could never satisfy the calibration's own ROM
    acceptance row, whose evidence lives in ``rom_endpoint_samples`` and was
    previously written by nothing but ``soarm-calibrate-rom``.

    ``handles``, if given, receives ``"homing_tick"``: a ``(ctx) -> None``
    the caller should invoke from its panel tick to refresh the live readout.
    """
    if heading:
        server.gui.add_markdown("## Homing Wizard")
        server.gui.add_markdown(
            "Calibrate servo midpoints via automatic motor sweep or manual "
            "hand movement."
        )

    _OPT_AUTO = "Automatic — motor sweep"
    _OPT_MAN = "Manual — move by hand"

    mode_h = server.gui.add_dropdown(
        "Mode", options=[_OPT_AUTO, _OPT_MAN], initial_value=_OPT_AUTO
    )
    joints_h = server.gui.add_text("Joint IDs", initial_value="1-6")
    target_ref_h = server.gui.add_number(
        "Zero reference (ticks)", initial_value=2048, min=0, max=4095, step=1
    )

    sim_mode_h = server.gui.add_checkbox(
        "Dry-run (simulate sweep, no hardware required)", initial_value=False
    )
    sweep_params_folder = server.gui.add_folder("Sweep Parameters")
    with sweep_params_folder:
        speed_h = server.gui.add_slider(
            "Sweep speed (ticks/s)", min=30, max=500, step=10, initial_value=150
        )
        stall_thr_h = server.gui.add_number(
            "Stall threshold (ticks)", initial_value=5, min=1, max=50, step=1
        )
        stall_win_h = server.gui.add_number(
            "Stall window (samples)", initial_value=8, min=3, max=20, step=1
        )
        timeout_h = server.gui.add_number(
            "Timeout/direction (s)", initial_value=30.0, min=5.0, max=120.0, step=1.0
        )
        max_range_h = server.gui.add_number(
            "Max travel/direction (ticks, 0 = unlimited)",
            initial_value=0,
            min=0,
            max=4096,
            step=50,
        )
    sweep_btn = server.gui.add_button("Sweep All Joints", color="green")

    man_header_md = server.gui.add_markdown(
        "Disable torque then move each joint by hand to its limits. The table "
        "below is live — push the joint until the number stops changing, then "
        "record that end."
    )
    torque_off_btn = server.gui.add_button("Disable Torque on Selected Joints")
    man_live_md = server.gui.add_markdown("*Waiting for a servo reading…*")
    man_status_md = server.gui.add_markdown("")
    man_positions: Dict[int, Dict[str, Optional[int]]] = {
        sid: {"min": None, "max": None} for sid in ctx.joint_ids
    }
    # One label plus its own three buttons, per joint — in that order, six
    # times — rather than one combined min/max table followed by eighteen
    # undifferentiated buttons below it. The table already existed
    # (man_live_md carries the same numbers, live); what was missing was
    # the buttons sitting next to the row they act on instead of several
    # screens of buttons away from it, identifiable only by an id number.
    man_row_mds: Dict[int, Any] = {}
    man_min_btns: Dict[int, Any] = {}
    man_max_btns: Dict[int, Any] = {}
    man_reset_btns: Dict[int, Any] = {}
    for sid, name in zip(ctx.joint_ids, SOARM100_JOINT_NAMES):
        man_row_mds[sid] = server.gui.add_markdown(f"**J{sid} {name}** — min —, max —")
        man_min_btns[sid] = server.gui.add_button(f"Record Min J{sid}")
        man_max_btns[sid] = server.gui.add_button(f"Record Max J{sid}")
        man_reset_btns[sid] = server.gui.add_button(f"Reset J{sid}")
    man_compute_btn = server.gui.add_button("Compute from Recorded Limits", color="blue")

    apply_btn = server.gui.add_button("Apply: Write Offsets + Limits to EEPROM", color="blue")
    save_btn = server.gui.add_button("Save Config (soarm100_rom.json)")
    log_md = server.gui.add_markdown("*Select a mode and proceed.*")

    _auto_controls = [sim_mode_h, sweep_params_folder, sweep_btn]
    _man_controls = [
        man_header_md,
        torque_off_btn,
        man_live_md,
        man_status_md,
        man_compute_btn,
        *man_row_mds.values(),
        *man_min_btns.values(),
        *man_max_btns.values(),
        *man_reset_btns.values(),
    ]

    def _apply_visibility(selected: str) -> None:
        is_auto = selected == _OPT_AUTO
        for h in _auto_controls:
            h.visible = is_auto
        for h in _man_controls:
            h.visible = not is_auto

    _apply_visibility(mode_h.value)

    @mode_h.on_update
    def _on_mode_change(ev: Any) -> None:
        _apply_visibility(mode_h.value)

    rom_results: Dict[int, dict] = {}

    def _publish_endpoints(simulated: bool) -> None:
        """Hand a complete endpoint set to the caller, keyed by joint name.

        ``rom_results`` is keyed by servo id and exists to drive EEPROM
        writes; the calibration's evidence is keyed by joint name. Both
        modes funnel through here so neither can quietly skip recording.
        """
        if on_endpoints is None:
            return
        by_name: Dict[str, tuple] = {}
        for sid, name in zip(ctx.joint_ids, SOARM100_JOINT_NAMES):
            d = rom_results.get(sid)
            if d is None:
                continue
            lo, hi = int(d["pos_min"]), int(d["pos_max"])
            by_name[name] = (min(lo, hi), max(lo, hi))
        if by_name:
            on_endpoints(by_name, simulated)

    def _tick(tick_ctx: Any) -> None:
        """Live ticks per joint, beside whatever has been recorded so far.

        Recording a hard stop by hand means pushing until the number stops
        moving — which you cannot do if the number is not on screen. Without
        this the operator pressed Record and hoped.
        """
        with tick_ctx.lock:
            positions = dict(tick_ctx.state.positions)
        cal = getattr(tick_ctx, "calibration", None)
        by_name = {j.name: j for j in cal.joints} if cal is not None else {}
        lines = [
            "| Joint | Live | Angle | Recorded min | Recorded max | Span |",
            "|---|--:|--:|--:|--:|--:|",
        ]
        for sid, name in zip(tick_ctx.joint_ids, SOARM100_JOINT_NAMES):
            ticks = positions.get(sid)
            joint = by_name.get(name)
            ang = (
                f"{math.degrees(joint.to_rad(ticks)):+.1f}°"
                if joint is not None and ticks is not None
                else "—"
            )
            mn = man_positions.get(sid, {}).get("min")
            mx = man_positions.get(sid, {}).get("max")
            span = f"{abs(mx - mn)}" if mn is not None and mx is not None else "—"
            live = f"**{ticks}**" if ticks is not None else "*no reading*"
            lines.append(
                f"| J{sid} {name} | {live} | {ang} "
                f"| {mn if mn is not None else '—'} "
                f"| {mx if mx is not None else '—'} | {span} |"
            )
        man_live_md.content = "\n".join(lines)

    if handles is not None:
        handles["homing_tick"] = _tick

    def _update_man_rows() -> None:
        """Refresh each joint's own label — the confirmation for that joint's
        buttons lives right there, not in a shared status line elsewhere."""
        for sid, name in zip(ctx.joint_ids, SOARM100_JOINT_NAMES):
            mn = man_positions[sid]["min"]
            mx = man_positions[sid]["max"]
            man_row_mds[sid].content = (
                f"**J{sid} {name}** — "
                f"min {mn if mn is not None else '—'}, "
                f"max {mx if mx is not None else '—'}"
            )

    @sweep_btn.on_click
    def _do_sweep(_: Any) -> None:
        simulate = sim_mode_h.value

        if not simulate and not ctx.device_h.value:
            log_md.content = (
                "**Error**: No serial device configured. "
                "Enable *Dry-run* to test without hardware."
            )
            return

        try:
            ids = ctx.parse_ids(joints_h.value)
        except Exception:
            ids = list(ctx.joint_ids)

        if not simulate:
            ctx.stop_polling()

        label = "[SIM] Simulating" if simulate else "Sweeping"
        log_md.content = f"{label} joints {ids}…"
        try:
            if simulate:
                res = _run_rom_sweep_sim(
                    ctx,
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
                    ctx,
                    joint_ids=ids,
                    sweep_speed=int(speed_h.value),
                    stall_thr=int(stall_thr_h.value),
                    stall_win=int(stall_win_h.value),
                    timeout_s=float(timeout_h.value),
                    max_range_ticks=int(max_range_h.value),
                    log_fn=lambda msg: setattr(log_md, "content", msg),
                )
                method = "auto-sweep"
            for _sid, d in res.items():
                d["method"] = method
            rom_results.update(res)
            _publish_endpoints(simulate)
            suffix = (
                " (simulated — Apply will still write to real EEPROM if connected)"
                if simulate
                else ""
            )
            log_md.content = f"**Sweep complete{suffix}.** Check results and press Apply."
        except Exception as exc:
            log_md.content = f"**Sweep error**: {exc}"
        finally:
            if not simulate:
                ctx.start_polling()

    @apply_btn.on_click
    def _do_apply(_: Any) -> None:
        if not rom_results:
            log_md.content = "**Error**: No results yet — run a sweep or manual recording first."
            return
        target_ref = int(target_ref_h.value)

        # A sweep records the smallest and largest position it saw, so a joint
        # whose travel crosses 4095/0 measures as the encoder's range instead
        # of its own. Deriving an offset from that would put the joint's frame
        # somewhere arbitrary, so those joints are reported and left alone —
        # `soarm-calibrate-rom --recentre` measures them by accumulating
        # displacement, which survives the wrap.
        wrapped = [sid for sid, d in rom_results.items()
                   if d["range_ticks"] >= WRAP_SUSPECT_TICKS]
        writable = {sid: d for sid, d in rom_results.items() if sid not in wrapped}
        if not writable:
            log_md.content = (
                f"**Nothing written.** J{', J'.join(str(s) for s in wrapped)} "
                "measured nearly a full encoder turn — the travel wrapped. "
                "Use `soarm-calibrate-rom --recentre` on these."
            )
            return

        try:
            with ctx.bus() as srv:
                for sid, d in writable.items():
                    # The servo reports `raw - STS_OFS`, so the offset that
                    # makes the swept midpoint read `target_ref` is
                    # `zero - target_ref` — NOT the other way round. The sweep
                    # runs in wheel mode, where the reported position is the
                    # raw encoder, so `zero` is already in the raw frame the
                    # register is subtracted from.
                    off, lmin, lmax = centring_offset(
                        d["zero"], d["pos_min"], d["pos_max"], target_ref
                    )
                    write1(srv, sid, STS_LOCK, 0, "unlock")
                    write2(srv, sid, STS_OFS_L, encode_ofs(off), f"offset J{sid}")
                    write2(srv, sid, STS_MIN_ANGLE_LIMIT_L, lmin, f"min J{sid}")
                    write2(srv, sid, STS_MAX_ANGLE_LIMIT_L, lmax, f"max J{sid}")
                    write1(srv, sid, STS_LOCK, 1, "lock")
            done = ", ".join(f"J{sid}" for sid in writable)
            msg = f"**EEPROM updated.** Offsets and limits written to {done}."
            if wrapped:
                skipped = ", ".join(f"J{sid}" for sid in wrapped)
                msg += (
                    f"\n\n**Skipped {skipped}** — travel wrapped past 4095/0, so the "
                    "swept range is the encoder's, not the joint's. Run "
                    "`soarm-calibrate-rom --recentre` on these."
                )
            log_md.content = msg
        except Exception as exc:
            log_md.content = f"**Apply error**: {exc}"

    @save_btn.on_click
    def _do_save(_: Any) -> None:
        if not rom_results:
            log_md.content = "**Error**: No results to save."
            return
        target_ref = int(target_ref_h.value)
        cfg_data: Dict[str, Any] = {
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "device": ctx.device_h.value,
            "target_ref": target_ref,
            "joints": {},
        }
        for sid, d in sorted(rom_results.items()):
            off = target_ref - d["zero"]
            cfg_data["joints"][str(sid)] = {
                "pos_min": d["pos_min"],
                "pos_max": d["pos_max"],
                "zero_midpoint": d["zero"],
                "range_ticks": d["range_ticks"],
                "method": d.get("method", "—"),
                "offset_signed": off,
                "offset_register": (abs(off) | 0x0800) if off < 0 else abs(off),
                "limit_min": max(0, min(4095, d["pos_min"] + off)),
                "limit_max": max(0, min(4095, d["pos_max"] + off)),
            }
        save_path = Path("soarm100_rom.json")
        save_path.write_text(json.dumps(cfg_data, indent=2))
        log_md.content = f"**Saved**: `{save_path.resolve()}`"

    @torque_off_btn.on_click
    def _do_torque_off(_: Any) -> None:
        try:
            ids = ctx.parse_ids(joints_h.value)
        except Exception:
            ids = list(ctx.joint_ids)
        try:
            with ctx.bus() as srv:
                for sid in ids:
                    write1(srv, sid, STS_TORQUE_ENABLE, 0, "torque off")
            man_status_md.content = f"Torque OFF on J{ids}. Move joints by hand."
        except Exception as exc:
            man_status_md.content = f"**Error**: {exc}"

    def _make_man_handlers(sid_cap: int) -> None:
        @man_min_btns[sid_cap].on_click
        def _(_: Any) -> None:
            try:
                with ctx.bus() as srv:
                    p, r, _rest = srv.ReadPos(sid_cap)
                if r == COMM_SUCCESS:
                    man_positions[sid_cap]["min"] = p
                    _update_man_rows()
            except Exception as exc:
                man_status_md.content = f"**Error**: {exc}"

        @man_max_btns[sid_cap].on_click
        def _(_: Any) -> None:
            try:
                with ctx.bus() as srv:
                    p, r, _rest = srv.ReadPos(sid_cap)
                if r == COMM_SUCCESS:
                    man_positions[sid_cap]["max"] = p
                    _update_man_rows()
            except Exception as exc:
                man_status_md.content = f"**Error**: {exc}"

        @man_reset_btns[sid_cap].on_click
        def _(_: Any) -> None:
            # Clears both ends for this joint only — a bad Record Min does
            # not force discarding a good Record Max on the same joint, and
            # this touches nothing on any other joint's row.
            man_positions[sid_cap]["min"] = None
            man_positions[sid_cap]["max"] = None
            man_status_md.content = f"J{sid_cap}: recorded limits cleared."
            _update_man_rows()

    for sid in ctx.joint_ids:
        _make_man_handlers(sid)

    @man_compute_btn.on_click
    def _do_man_compute(_: Any) -> None:
        computed = []
        for sid in ctx.joint_ids:
            mn = man_positions[sid]["min"]
            mx = man_positions[sid]["max"]
            if mn is None or mx is None:
                continue
            if mn > mx:
                mn, mx = mx, mn
            zero = (mn + mx) // 2
            rom_results[sid] = {
                "pos_min": mn,
                "pos_max": mx,
                "zero": zero,
                "range_ticks": mx - mn,
                "method": "manual",
            }
            computed.append(sid)
        if computed:
            _publish_endpoints(False)
            log_md.content = f"Computed from manual limits for J{computed}. Press Apply to write."
        else:
            log_md.content = "**Error**: Record both min and max for at least one joint."


# ---------------------------------------------------------------------------
# Reconfigure
# ---------------------------------------------------------------------------


def build_reconfigure_panel() -> Panel:
    """Calibration (IDs / limits / mode / baud) + config export/import."""
    return Panel("Reconfigure", _build_reconfigure)


def _build_reconfigure(server: Any, ctx: DashboardContext) -> None:
    server.gui.add_markdown("## Reconfigure")

    with server.gui.add_folder("Calibration"):
        server.gui.add_markdown(
            "Configure servo IDs, angle limits, acceleration, speed, "
            "mode, torque, and baud rate in a single run."
        )
        scan_range_h = server.gui.add_text("Scan range", initial_value="1-10")
        unlock_h = server.gui.add_checkbox("Unlock EEPROM before changes", initial_value=False)
        lock_h = server.gui.add_checkbox("Lock EEPROM after changes", initial_value=True)

        with server.gui.add_folder("ID Remapping (OLD:NEW per line)"):
            assign_h = server.gui.add_text(
                "Assign IDs", initial_value="", multiline=True, hint="1:11\n2:12"
            )
        with server.gui.add_folder("Angle Limits (ID:MIN:MAX per line)"):
            angle_h = server.gui.add_text(
                "Angle limits", initial_value="", multiline=True, hint="1:512:3584"
            )
        with server.gui.add_folder("Motion Parameters"):
            acc_h = server.gui.add_text("Acceleration (ID:ACC)", initial_value="", multiline=True)
            speed_h = server.gui.add_text("Speed (ID:SPEED)", initial_value="", multiline=True)
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
        cal_log_md = server.gui.add_markdown("*Fill in the fields above and press Run.*")

    def _parse_text_lines(raw: str) -> List[str]:
        return [s.strip() for s in raw.splitlines() if s.strip()]

    @run_btn.on_click
    def _do_cal(_: Any) -> None:
        import argparse as _ap

        log_buf: List[str] = []
        args = _ap.Namespace(
            list_ports=False,
            ui=False,
            device=ctx.device_h.value or "/dev/ttyUSB0",
            baud=int(ctx.baud_h.value),
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
        cal_log_md.content = f"**{status}**\n```\n" + "\n".join(log_buf) + "\n```"

    with server.gui.add_folder("Config Export / Import"):
        server.gui.add_markdown(
            "Save and restore the full register state of soarm100 servos as JSON."
        )
        with server.gui.add_folder("Export"):
            export_ids_h = server.gui.add_text("IDs to export", initial_value="1-6")
            export_path_h = server.gui.add_text(
                "Output file", initial_value="soarm100_config.json"
            )
            export_btn = server.gui.add_button("Read & Export", color="green")

        with server.gui.add_folder("Import"):
            import_path_h = server.gui.add_text(
                "JSON file to apply", initial_value="soarm100_config.json"
            )
            import_btn = server.gui.add_button("Apply Config from File", color="blue")

        cfg_log_md = server.gui.add_markdown("*Use Export or Import above.*")

    @export_btn.on_click
    def _do_export(_: Any) -> None:
        try:
            ids = ctx.parse_ids(export_ids_h.value)
            diag = read_servo_diagnostics(ctx.device_h.value, int(ctx.baud_h.value), ids)
            snapshot: Dict[str, Any] = {
                "exported_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "device": ctx.device_h.value,
                "servos": {},
            }
            for sid, data in diag.items():
                entry = {k: v for k, v in (data or {}).items() if k != "_errors"}
                snapshot["servos"][str(sid)] = entry
            out_path = Path(export_path_h.value)
            out_path.write_text(json.dumps(snapshot, indent=2))
            cfg_log_md.content = f"**Exported** {len(diag)} servo(s) → `{out_path.resolve()}`"
        except Exception as exc:
            cfg_log_md.content = f"**Export error**: {exc}"

    @import_btn.on_click
    def _do_import(_: Any) -> None:
        try:
            snap = json.loads(Path(import_path_h.value).read_text())
            servos = snap.get("servos", {})
            log_lines: List[str] = []
            with ctx.bus() as srv:
                for sid_str, entry in servos.items():
                    sid = int(sid_str)
                    write1(srv, sid, STS_LOCK, 0, "unlock")
                    acc_val = entry.get("Acceleration")
                    if acc_val is not None:
                        write1(srv, sid, STS_ACC, int(acc_val), "acc")
                        log_lines.append(f"J{sid}: ACC={acc_val}")
                    mode_val = entry.get("Mode")
                    if mode_val is not None:
                        write1(srv, sid, STS_MODE, int(mode_val), "mode")
                        log_lines.append(f"J{sid}: MODE={mode_val}")
                    write1(srv, sid, STS_LOCK, 1, "lock")
            log_lines.append("Done.")
            cfg_log_md.content = "**Import complete.**\n```\n" + "\n".join(log_lines) + "\n```"
        except Exception as exc:
            cfg_log_md.content = f"**Import error**: {exc}"
