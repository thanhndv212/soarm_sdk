"""Position Recorder & Replayer: record demonstration trajectories, replay from CSV."""

from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from ..app import Panel
from ..context import DashboardContext

__all__ = ["build_recorder_panel"]


def build_recorder_panel() -> Panel:
    return Panel("Recorder", _build_recorder)


def _build_recorder(server: Any, ctx: DashboardContext) -> None:
    server.gui.add_markdown("## Position Recorder & Replayer")
    server.gui.add_markdown(
        "Record demonstration trajectories from soarm100 joints and replay them."
    )

    # Recording state lives on this closure, not a module global — each
    # dashboard instance (and each build of this panel) gets its own.
    recording: Dict[str, Any] = {"active": False, "data": [], "ids": set(ctx.joint_ids)}

    def _on_sample(poll_count: int, positions: Dict[int, int], speeds: Dict[int, int]) -> None:
        if not recording["active"]:
            return
        now = round(time.time(), 4)
        for sid, pos in positions.items():
            if sid in recording["ids"]:
                recording["data"].append(
                    {"timestamp": now, "id": sid, "position": pos, "speed": speeds.get(sid, 0)}
                )

    ctx.add_sample_hook(_on_sample)

    ids_h = server.gui.add_text("Joint IDs to record", initial_value="1-6")
    rec_btn = server.gui.add_button("Start Recording", color="green")
    stop_rec_btn = server.gui.add_button("Stop & Save", color="red", disabled=True)
    status_md = server.gui.add_markdown("*Not recording.*")

    server.gui.add_markdown("---\n**Replay**")
    replay_path_h = server.gui.add_text("CSV path", initial_value="trajectory.csv")
    speed_scale_h = server.gui.add_slider(
        "Speed scale", min=0.1, max=5.0, step=0.1, initial_value=1.0
    )
    loop_h = server.gui.add_number("Loops", initial_value=1, min=1, max=20, step=1)
    replay_btn = server.gui.add_button("Replay Trajectory", color="blue")

    @rec_btn.on_click
    def _start_rec(_: Any) -> None:
        try:
            rec_ids = set(ctx.parse_ids(ids_h.value))
        except Exception:
            rec_ids = set(ctx.joint_ids)
        recording["active"] = True
        recording["data"] = []
        recording["ids"] = rec_ids
        rec_btn.disabled = True
        stop_rec_btn.disabled = False
        status_md.content = f"**Recording…** (J{sorted(rec_ids)})"

    @stop_rec_btn.on_click
    def _stop_rec(_: Any) -> None:
        recording["active"] = False
        data = list(recording["data"])
        recording["data"] = []
        rec_btn.disabled = False
        stop_rec_btn.disabled = True
        if data:
            ts = time.strftime("%Y%m%d_%H%M%S")
            save_path = Path(f"trajectory_{ts}.csv")
            with save_path.open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=["timestamp", "id", "position", "speed"])
                writer.writeheader()
                writer.writerows(data)
            status_md.content = f"**Saved** {len(data)} samples → `{save_path.resolve()}`"
        else:
            status_md.content = "Stopped. No data recorded."

    @replay_btn.on_click
    def _do_replay(_: Any) -> None:
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
            with ctx.bus() as srv:
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
