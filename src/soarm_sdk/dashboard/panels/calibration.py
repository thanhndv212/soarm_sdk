"""Calibration: make the 3-D mirror agree with the arm in front of you.

The mirror can disagree with the arm in three different ways, and they
want three different fixes. Telling them apart by looking at the viser
window is close to impossible — every one of them renders as "the model
is bent wrong" — so this tab reports the three separately.

**The zero is off.** ``rad = sign * (ticks - zero) * RADS_PER_TICK``. Get
``zero`` wrong and the member sits at the right-looking angle plus a
constant. Nothing in a tick reading reveals it; only a pose you can
independently verify does. That is what the *Reference pose* section is
for, and what the per-joint nudges are for when the pose gets you close
but not all the way.

**The direction sign is off.** Then the member moves the *wrong way* as
the joint turns, and no zero will fix it. The *Live* table reports the
sign next to each joint so it can be checked the only way it can be
checked: move the joint and watch whether the model follows or opposes.

**The pose is outside what the URDF admits.** This one is not a
calibration error at all, and it is the one that produces the
self-intersecting render that looks worst. The arm's measured travel is
wider than the URDF's joint limits on five of this arm's six joints —
``shoulder_lift`` reaches 26 deg past the model's lower limit — so a pose
the arm holds perfectly happily can be one the model has no valid
configuration for. yourdfpy does not clamp, so the mesh is simply posed
out of range and the links pass through each other. The table flags it as
OUTSIDE rather than leaving it to be misread as a bad zero.
"""

from __future__ import annotations

import logging
import math
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from ...calibration.frame import RobotCalibration, rezero_from_pose
from ...calibration.reference import REFERENCE_POSES, ReferencePose
from ..app import Panel
from ..fk import SOARM100_JOINT_NAMES

logger = logging.getLogger(__name__)

__all__ = ["build_calibration_panel"]

#: so101_new_calib.urdf, the revision this arm actually is. Duplicated from
#: ``soarm_tamp.conventions.URDF_LIMITS`` deliberately: this tab has to work
#: in a dashboard that has no planner installed.
URDF_LIMITS: Dict[str, tuple] = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}

#: How far a nudge can move one joint's zero, in degrees. Wide enough for a
#: badly seeded zero, narrow enough that a slip cannot silently rewrite the
#: calibration into nonsense.
NUDGE_LIMIT_DEG = 45.0

#: Overshoot below this is not reported. A joint resting exactly on a limit
#: crosses it by microdegrees as the servo jitters, and "OUTSIDE by 0.0°"
#: every few frames trains the operator to ignore the flag that matters —
#: this arm really does reach 26 deg past one of them.
LIMIT_TOLERANCE_DEG = 0.1


def build_calibration_panel() -> Panel:
    """The Calibration tab: live model-vs-arm comparison, re-zero, nudge."""
    handles: Dict[str, Any] = {}
    return Panel(
        "Calibration",
        lambda server, ctx: _build(server, ctx, handles),
        lambda ctx: _on_tick(ctx, handles),
    )


# ----------------------------------------------------------------------
# Reporting
# ----------------------------------------------------------------------


def _joint_rows(ctx: Any, positions: Dict[int, int]) -> List[dict]:
    """One row per joint: ticks, the angle they mean, and whether it is legal.

    Returns dicts rather than formatted text so the same computation feeds
    the table, the OUTSIDE banner and the FK call without being redone or,
    worse, redone slightly differently.
    """
    cal: Optional[RobotCalibration] = getattr(ctx, "calibration", None)
    by_name = {j.name: j for j in cal.joints} if cal is not None else {}
    rows: List[dict] = []
    for sid, name in zip(ctx.joint_ids, SOARM100_JOINT_NAMES):
        ticks = positions.get(sid)
        joint = by_name.get(name)
        rad = joint.to_rad(ticks) if (joint is not None and ticks is not None) else None
        lo, hi = URDF_LIMITS.get(name, (float("-inf"), float("inf")))
        over = 0.0
        if rad is not None:
            over = max(lo - rad, rad - hi, 0.0)
            if math.degrees(over) < LIMIT_TOLERANCE_DEG:
                over = 0.0
        rows.append(
            {
                "sid": sid,
                "name": name,
                "ticks": ticks,
                "rad": rad,
                "sign": joint.direction_sign if joint is not None else None,
                "zero": joint.zero_offset_ticks if joint is not None else None,
                "source": joint.zero_source if joint is not None else "—",
                "limits": (lo, hi),
                "over_rad": over,
            }
        )
    return rows


def _format_table(rows: List[dict], cal: Optional[RobotCalibration]) -> str:
    if cal is None:
        return (
            "**No calibration loaded.** The 3-D view is assuming tick 2048 is "
            "zero on every joint, which no real arm has — it will not match. "
            "Start the dashboard with `--calibration PATH`."
        )
    out = [
        "| Joint | Ticks | Angle | Zero | Sign | URDF limit | |",
        "|---|--:|--:|--:|:-:|:-:|---|",
    ]
    for r in rows:
        if r["ticks"] is None:
            out.append(f"| {r['name']} | — | — | — | — | — | *no reading* |")
            continue
        lo, hi = r["limits"]
        flag = "ok"
        if r["over_rad"] > 0:
            flag = f"**OUTSIDE by {math.degrees(r['over_rad']):.1f}°**"
        out.append(
            f"| {r['name']} | {r['ticks']} | {math.degrees(r['rad']):+.1f}° "
            f"| {r['zero']:.0f} | {r['sign']:+d} "
            f"| {math.degrees(lo):+.0f}…{math.degrees(hi):+.0f}° | {flag} |"
        )
    return "\n".join(out)


def _format_members(ctx: Any, rows: List[dict]) -> str:
    """What the model says the arm looks like, in degrees off horizontal.

    The only line in this tab an operator can check without believing any
    of the rest of it: hold a level against the real member and compare.
    """
    urdf = getattr(ctx, "urdf", None)
    if urdf is None:
        return "*No URDF loaded — no member angles to compare against.*"
    cfg = {r["name"]: r["rad"] for r in rows if r["rad"] is not None}
    if not cfg:
        return "*Waiting for a servo reading…*"
    try:
        from ...kinematics.urdf_fk import member_pitches

        pitches = member_pitches(urdf, cfg)
    except Exception:
        logger.exception("member_pitches failed")
        return "*Could not pose the URDF at this configuration.*"
    lines = [
        "The model says these members are at, measured from horizontal —",
        "put a level on the real ones and compare:",
        "",
        "| Member | Model says |",
        "|---|--:|",
    ]
    for name, deg in pitches.items():
        lines.append(f"| {name.replace('_', ' ')} | {deg:+.1f}° |")
    return "\n".join(lines)


#: A travel midpoint this far off the URDF's own midpoint is called out.
#: Below it the estimate is swamped by how roughly the hard stops were
#: captured; above it, something is actually wrong with the zero.
SYMMETRY_WARN_DEG = 8.0


def _format_symmetry(cal: Optional[RobotCalibration]) -> str:
    """Check each zero against the symmetry of the joint's own travel.

    Needs no hardware, no reference pose and no operator: the arm's stops
    were already measured, and the URDF already says where they should sit.
    Express the measured travel in the calibration's frame and its midpoint
    should land on the URDF's midpoint. Where it does not, the gap is the
    zero error, read off without believing anything about the zero.

    It is a cross-check, not a replacement for a reference pose — it assumes
    the stops really are symmetric, which is true of this arm's pitch joints
    and false of the gripper, whose travel is a jaw opening. Joints whose
    measured span disagrees with the URDF are marked, because for those the
    assumption is already shaky.
    """
    if cal is None:
        return ""
    rows = [
        "Each joint's measured travel should straddle zero the way the "
        "URDF's limits do. Where the midpoint has drifted, that gap is the "
        "zero error — measured without trusting the zero:",
        "",
        "| Joint | Travel midpoint | Should be | Off by |",
        "|---|--:|--:|--:|",
    ]
    flagged = False
    for j in cal.joints:
        lo, hi = j.reachable_rad
        mid = math.degrees((lo + hi) / 2)
        u_lo, u_hi = URDF_LIMITS.get(j.name, (0.0, 0.0))
        u_mid = math.degrees((u_lo + u_hi) / 2)
        err = mid - u_mid
        note = f"{err:+.1f}°"
        if j.name == "gripper":
            note += " *(jaw travel — not symmetric, ignore)*"
        elif abs(err) >= SYMMETRY_WARN_DEG:
            note = f"**{err:+.1f}°**"
            flagged = True
        rows.append(f"| {j.name} | {mid:+.1f}° | {u_mid:+.1f}° | {note} |")
    if flagged:
        rows += [
            "",
            "A bold figure means that joint's zero is out by roughly that "
            "much. Confirm it against a reference pose below rather than "
            "subtracting it here — this check assumes the hard stops are "
            "symmetric, which a reference pose does not have to assume.",
        ]
    return "\n".join(rows)


def _on_tick(ctx: Any, handles: Dict[str, Any]) -> None:
    if not handles:
        return
    with ctx.lock:
        positions = dict(ctx.state.positions)
        connected = ctx.state.connected
    if not connected and not positions:
        handles["table_md"].content = "*Not connected — connect in the sidebar.*"
        return
    rows = _joint_rows(ctx, positions)
    handles["rows"] = rows
    handles["symmetry_md"].content = _format_symmetry(getattr(ctx, "calibration", None))
    handles["table_md"].content = _format_table(rows, getattr(ctx, "calibration", None))
    handles["members_md"].content = _format_members(ctx, rows)

    outside = [r["name"] for r in rows if r["over_rad"] > 0]
    handles["outside_md"].content = (
        ""
        if not outside
        else (
            f"⚠️ **{', '.join(outside)}** {'is' if len(outside) == 1 else 'are'} "
            "outside the URDF's joint limits. The arm is fine there — its "
            "measured travel is wider than the model's — but the render is "
            "posed out of range, which is why the links pass through each "
            "other. This is not a bad zero; do not re-zero to fix it."
        )
    )


# ----------------------------------------------------------------------
# Building
# ----------------------------------------------------------------------


def _build(server: Any, ctx: Any, handles: Dict[str, Any]) -> None:
    server.gui.add_markdown(
        "Compare the 3-D mirror against the arm in front of you, and correct "
        "it. Reads the bus; **never commands the arm.**"
    )

    with server.gui.add_folder("Live"):
        handles["outside_md"] = server.gui.add_markdown("")
        handles["table_md"] = server.gui.add_markdown("*Waiting…*")

    with server.gui.add_folder("Model vs. arm"):
        handles["members_md"] = server.gui.add_markdown("*Waiting…*")

    with server.gui.add_folder("Zero sanity check (no hardware needed)"):
        handles["symmetry_md"] = server.gui.add_markdown("*Waiting…*")

    _build_rezero(server, ctx, handles)
    _build_nudge(server, ctx, handles)
    _build_save(server, ctx, handles)

    handles["status_md"] = server.gui.add_markdown("")


def _build_rezero(server: Any, ctx: Any, handles: Dict[str, Any]) -> None:
    with server.gui.add_folder("Re-zero from a reference pose"):
        keys = list(REFERENCE_POSES)
        pose_h = server.gui.add_dropdown(
            "Pose", options=keys, initial_value=keys[0]
        )
        detail_md = server.gui.add_markdown("")
        rezero_btn = server.gui.add_button("Re-zero to this pose", color="orange")

        def _describe(key: str) -> str:
            pose: ReferencePose = REFERENCE_POSES[key]
            covers = ", ".join(pose.covers) if pose.covers else "*nothing*"
            body = [
                f"**{pose.label}**",
                "",
                f"*Set up:* {pose.setup}",
                "",
                f"*Check it by:* {pose.verify_by}",
                "",
                f"*Re-zeros:* {covers}",
            ]
            if not pose.covers:
                body.append("")
                body.append(
                    "This pose constrains nothing you can verify, so re-zeroing "
                    "against it is disabled."
                )
            return "\n".join(body)

        detail_md.content = _describe(pose_h.value)

        @pose_h.on_update
        def _on_pose(_: Any) -> None:
            detail_md.content = _describe(pose_h.value)

        @rezero_btn.on_click
        def _rezero(_: Any) -> None:
            _do_rezero(ctx, handles, REFERENCE_POSES[pose_h.value])


def _do_rezero(ctx: Any, handles: Dict[str, Any], pose: ReferencePose) -> None:
    cal: Optional[RobotCalibration] = getattr(ctx, "calibration", None)
    if cal is None:
        return _say(handles, "no calibration loaded — nothing to re-zero")
    if not pose.covers:
        return _say(handles, f"{pose.key} constrains no joint; refusing to re-zero")

    with ctx.lock:
        positions = dict(ctx.state.positions)
        connected = ctx.state.connected
    ticks = [positions.get(sid) for sid in ctx.joint_ids]
    if not connected or any(t is None for t in ticks):
        # A missing reading is the trap this whole module exists to avoid:
        # the arm is not where the numbers say, and the zero gets pinned to
        # a value nobody measured.
        return _say(handles, "no live reading from every joint — connect first")

    try:
        ref = pose.q_for(tuple(cal.names))
        new = rezero_from_pose(
            cal,
            ticks=[float(t) for t in ticks],
            reference_rad=ref,
            only=pose.covers,
            source=(
                f"re-zeroed from the dashboard against reference pose "
                f"'{pose.key}' ({pose.label})"
            ),
        )
    except Exception as exc:
        logger.exception("re-zero failed")
        return _say(handles, f"re-zero failed: {exc}")

    # The signs were checked physically and are not re-measured here, so
    # they carry over with the claim that established them.
    new.validated = cal.validated
    new.notes = dict(cal.notes)
    new.notes["rezeroed_from_dashboard"] = {
        "pose": pose.key,
        "at": datetime.now(timezone.utc).isoformat(),
        "ticks": {n: t for n, t in zip(cal.names, ticks)},
        "reference_rad": {n: r for n, r in zip(cal.names, ref)},
        "joints": list(pose.covers),
    }
    ctx.calibration = new
    handles["nudge_base"] = new
    for name, slider in handles.get("nudges", {}).items():
        slider.value = 0.0
    # How far each joint's reported angle moved, which is what the operator
    # sees change in the mirror — not the raw shift in the zero, which is in
    # ticks and signed the other way when direction_sign is -1.
    moved = []
    for after, before in zip(new.joints, cal.joints):
        if after.zero_offset_ticks == before.zero_offset_ticks:
            continue
        shift = after.to_rad(0.0) - before.to_rad(0.0)
        moved.append(f"{after.name} {math.degrees(shift):+.1f}°")
    _say(
        handles,
        f"re-zeroed against '{pose.key}' — "
        f"{', '.join(moved) if moved else 'no change'}. "
        "Check the mirror, then **Save** to keep it.",
    )


def _build_nudge(server: Any, ctx: Any, handles: Dict[str, Any]) -> None:
    with server.gui.add_folder("Nudge a single zero"):
        server.gui.add_markdown(
            "Moves what the ticks are taken to *mean*, not the arm. Use it "
            "when a reference pose gets the mirror close but one member is "
            "still visibly off. Applies live; **Save** to keep it."
        )
        nudges: Dict[str, Any] = {}
        for name in SOARM100_JOINT_NAMES:
            nudges[name] = server.gui.add_slider(
                name,
                min=-NUDGE_LIMIT_DEG,
                max=NUDGE_LIMIT_DEG,
                step=0.25,
                initial_value=0.0,
            )
        handles["nudges"] = nudges
        reset_btn = server.gui.add_button("Reset nudges")

        def _apply(_: Any = None) -> None:
            base: Optional[RobotCalibration] = handles.get("nudge_base")
            if base is None:
                return
            deltas = {n: math.radians(h.value) for n, h in nudges.items()}
            joints = [
                j.shifted_by(deltas[j.name]) if deltas.get(j.name) else j
                for j in base.joints
            ]
            live = RobotCalibration(
                joints=joints,
                arm_id=base.arm_id,
                validated=base.validated,
                source=base.source,
                created=base.created,
                notes=dict(base.notes),
            )
            applied = {n: round(v, 2) for n, v in
                       ((n, h.value) for n, h in nudges.items()) if v}
            if applied:
                live.source = f"{base.source} + manual nudge {applied}"
                live.notes["manual_nudge_deg"] = applied
            ctx.calibration = live

        for h in nudges.values():
            h.on_update(_apply)

        @reset_btn.on_click
        def _reset(_: Any) -> None:
            for h in nudges.values():
                h.value = 0.0
            _apply()
            _say(handles, "nudges reset")

    handles["nudge_base"] = getattr(ctx, "calibration", None)


def _build_save(server: Any, ctx: Any, handles: Dict[str, Any]) -> None:
    with server.gui.add_folder("Save"):
        save_btn = server.gui.add_button("Save calibration", color="green")
        reload_btn = server.gui.add_button("Discard and reload from disk")

        @save_btn.on_click
        def _save(_: Any) -> None:
            _do_save(ctx, handles)

        @reload_btn.on_click
        def _reload(_: Any) -> None:
            path = _cal_path(ctx)
            try:
                cal = RobotCalibration.load(path)
            except Exception as exc:
                return _say(handles, f"reload failed: {exc}")
            ctx.calibration = cal
            handles["nudge_base"] = cal
            for h in handles.get("nudges", {}).values():
                h.value = 0.0
            _say(handles, f"reloaded {path.name} — unsaved changes discarded")


def _cal_path(ctx: Any) -> Path:
    from ..fk import DEFAULT_CALIBRATION_PATH

    p = getattr(ctx, "calibration_path", None)
    return Path(p) if p is not None else DEFAULT_CALIBRATION_PATH


def _do_save(ctx: Any, handles: Dict[str, Any]) -> None:
    cal: Optional[RobotCalibration] = getattr(ctx, "calibration", None)
    if cal is None:
        return _say(handles, "nothing to save")
    path = _cal_path(ctx)

    # Every previous calibration on this arm was replaced in place, and the
    # only reason the 2026-09-13 re-zero was recoverable is that someone
    # happened to make a backup by hand. Make it automatic.
    if path.exists():
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = path.with_name(f"{path.stem}.backup-{stamp}{path.suffix}")
        try:
            shutil.copy2(path, backup)
        except Exception as exc:
            return _say(handles, f"refusing to save — could not back up: {exc}")
    else:
        backup = None

    try:
        cal.save(path)
    except Exception as exc:
        logger.exception("calibration save failed")
        return _say(handles, f"save failed: {exc}")

    handles["nudge_base"] = cal
    for h in handles.get("nudges", {}).values():
        h.value = 0.0
    _say(
        handles,
        f"saved {path.name}" + (f" (backup: {backup.name})" if backup else ""),
    )


def _say(handles: Dict[str, Any], msg: str) -> None:
    md = handles.get("status_md")
    if md is not None:
        md.content = f"*{msg}*"
