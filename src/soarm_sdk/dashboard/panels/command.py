"""Command Panel: send position/velocity commands to individual or all joints."""

from __future__ import annotations

from typing import Any, Dict

from ... import COMM_SUCCESS, STS_TORQUE_ENABLE, write1
from ..app import Panel
from ..context import DashboardContext
from ..fk import SOARM100_JOINT_NAMES

__all__ = ["build_command_panel"]


def build_command_panel() -> Panel:
    return Panel("Command Panel", _build_command)


def _build_command(server: Any, ctx: DashboardContext) -> None:
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

    for sid in ctx.joint_ids:
        joint_name = (
            SOARM100_JOINT_NAMES[sid - 1] if sid <= len(SOARM100_JOINT_NAMES) else str(sid)
        )
        with server.gui.add_folder(f"J{sid} — {joint_name}"):
            pos_handles[sid] = server.gui.add_slider(
                "Position (ticks)", min=0, max=4095, step=1, initial_value=2048
            )
            spd_handles[sid] = server.gui.add_number(
                "Speed (ticks/s)", initial_value=300, min=0, max=3000, step=10
            )
            acc_handles[sid] = server.gui.add_number(
                "Acc", initial_value=50, min=0, max=254, step=1
            )
            send_btns[sid] = server.gui.add_button(f"Send J{sid}", color="blue")

    send_all_btn = server.gui.add_button("Send All (Sync Packet)", color="green")
    torque_on_btn = server.gui.add_button("Torque ON all")
    torque_off_btn = server.gui.add_button("Torque OFF all")

    @refresh_btn.on_click
    def _do_refresh(_: Any) -> None:
        try:
            with ctx.bus() as srv:
                lines = []
                for sid in ctx.joint_ids:
                    p, r, _ = srv.ReadPos(sid)
                    if r == COMM_SUCCESS:
                        pos_handles[sid].value = int(p)
                        lines.append(f"J{sid}={p}")
            status_md.content = "Loaded: " + ", ".join(lines)
        except Exception as exc:
            status_md.content = f"**Error**: {exc}"

    def _make_send_handler(sid_cap: int) -> None:
        @send_btns[sid_cap].on_click
        def _(_: Any) -> None:
            is_servo = mode_h.value.startswith("Servo")
            try:
                with ctx.bus() as srv:
                    if is_servo:
                        pos = int(pos_handles[sid_cap].value)
                        spd = int(spd_handles[sid_cap].value)
                        acc = int(acc_handles[sid_cap].value)
                        srv.WritePosEx(sid_cap, pos, spd, acc)
                        status_md.content = f"J{sid_cap} → pos={pos}, spd={spd}, acc={acc}"
                    else:
                        spd = int(spd_handles[sid_cap].value)
                        acc = int(acc_handles[sid_cap].value)
                        srv.WheelMode(sid_cap)
                        srv.WriteSpec(sid_cap, spd, acc)
                        status_md.content = f"J{sid_cap} wheel spd={spd}, acc={acc}"
            except Exception as exc:
                status_md.content = f"**Error**: {exc}"

    for sid in ctx.joint_ids:
        _make_send_handler(sid)

    @send_all_btn.on_click
    def _do_send_all(_: Any) -> None:
        is_servo = mode_h.value.startswith("Servo")
        try:
            with ctx.bus() as srv:
                srv.groupSyncWrite.clearParam()
                for sid in ctx.joint_ids:
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
    def _do_ton(_: Any) -> None:
        try:
            with ctx.bus() as srv:
                for sid in ctx.joint_ids:
                    write1(srv, sid, STS_TORQUE_ENABLE, 1, "torque on")
            status_md.content = "Torque ON — all joints."
        except Exception as exc:
            status_md.content = f"**Error**: {exc}"

    @torque_off_btn.on_click
    def _do_toff(_: Any) -> None:
        try:
            with ctx.bus() as srv:
                for sid in ctx.joint_ids:
                    write1(srv, sid, STS_TORQUE_ENABLE, 0, "torque off")
            status_md.content = "Torque OFF — all joints."
        except Exception as exc:
            status_md.content = f"**Error**: {exc}"
