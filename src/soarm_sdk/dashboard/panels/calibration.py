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
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from ...calibration.frame import RobotCalibration, rezero_from_pose
from ...calibration.pipeline import (
    AcceptanceTolerances,
    CalibrationPipeline,
    PipelineStage,
)
from ...calibration.reference import REFERENCE_POSES, ReferencePose
from ...calibration.rom_sweep import WRAP_SUSPECT_TICKS
from ..app import Panel
from ..fk import SOARM100_JOINT_NAMES

logger = logging.getLogger(__name__)

__all__ = ["build_calibration_panels"]

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

#: Fallback bound for an alignment slider, in degrees, used only for a joint
#: with no declared URDF limit. Real joints are bounded by their own travel
#: instead — see :func:`_alignment_range`.
NUDGE_LIMIT_DEG = 45.0

#: How long to watch before pinning a zero, and how much movement over that
#: window disqualifies the reading.
SETTLE_S = 0.4
SETTLE_TOLERANCE_DEG = 0.5

#: Overshoot below this is not reported. A joint resting exactly on a limit
#: crosses it by microdegrees as the servo jitters, and "OUTSIDE by 0.0°"
#: every few frames trains the operator to ignore the flag that matters —
#: this arm really does reach 26 deg past one of them.
LIMIT_TOLERANCE_DEG = 0.1


def build_calibration_panels(
    fk_update_fn: Optional[Callable[[Dict[int, int]], None]] = None,
    ghost_fn: Optional[Callable[[Optional[Dict[str, float]]], None]] = None,
) -> List[Panel]:
    """The calibration workflow, as four tabs to be worked in order.

    One tab per group of pipeline stages, each ending in its own review and
    save. It was a single tab of six stacked folders, which put the Save
    button several screens below the step that blocked it and left the
    operator scrolling to find out what a click had done.

    All four share one ``handles`` registry and one tick function — the
    live tables appear on more than one tab, so each entry holds a *list*
    of Viser handles and :func:`_set` writes to all of them. Only the first
    panel carries ``on_tick``; it refreshes every tab's copy, and running it
    four times a tick would just repeat the same FK and report work.

    ``ghost_fn`` poses the translucent reference arm in the 3-D scene, or
    hides it when passed ``None``. Pass :meth:`DashboardApp.show_ghost`.
    """
    handles: Dict[str, Any] = {}
    return [
        Panel(
            "1 · Tolerances",
            lambda server, ctx: _tab_tolerances(server, ctx, handles),
            lambda ctx: _on_tick(ctx, handles),
        ),
        Panel(
            "2 · Signs",
            lambda server, ctx: _tab_signs(server, ctx, handles),
        ),
        Panel(
            "3 · Travel",
            lambda server, ctx: _tab_travel(server, ctx, handles, fk_update_fn),
        ),
        Panel(
            "4 · Zeros",
            lambda server, ctx: _tab_zeros(server, ctx, handles, ghost_fn),
        ),
    ]


def _reg(handles: Dict[str, Any], key: str, md: Any) -> Any:
    """Register a Viser markdown handle under *key*; several tabs may share it."""
    handles.setdefault(key, []).append(md)
    return md


def _set(handles: Dict[str, Any], key: str, content: str) -> None:
    """Write *content* to every handle registered under *key*."""
    for md in handles.get(key, ()):
        md.content = content


#: Which acceptance rows each tab is responsible for. The tab reports its own
#: rows at the end, so "did the thing I just did take?" is answered where it
#: was done rather than in a combined table further away.
TAB_STAGES: Dict[str, tuple] = {
    "tolerances": (PipelineStage.ACCEPTANCE,),
    "signs": (PipelineStage.DIRECTION_SIGNS,),
    "travel": (PipelineStage.ROM,),
    "zeros": (PipelineStage.ZERO_PROVENANCE, PipelineStage.REFERENCE_POSE),
}


def _order_banner(n: int, what: str) -> str:
    # Just the position in the order. What Save does is stated once, beside
    # Save, on every tab -- repeating it in four banners is four times the
    # words for none of the information.
    return f"**Step {n} of 4 — {what}.** Tabs run in order."


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
            "**No calibration loaded.** The view assumes tick 2048 is zero on "
            "every joint, which no real arm has. Start with `--calibration PATH`."
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
        "Model angles from horizontal — put a level on the real member:",
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
    """Compare each zero against the symmetry of the joint's own travel.

    Needs no hardware and no operator: the stops were already measured and
    the URDF already says where they should sit, so expressing the measured
    travel in the calibration's frame and looking at its midpoint costs
    nothing.

    **This is a hypothesis, not a measurement.** It is only as good as the
    assumption that the joint's mechanical stops are symmetric about the
    URDF's zero, and that assumption is not free — this arm's shoulder_lift
    travels 126 deg down and 89 deg up, genuinely asymmetric, because the
    structure blocks it one way and not the other. Read as a finding, that
    asymmetry says "the zero is 18.6 deg out". It is not; it is the shape of
    the mechanism. The first version of this check said it was, and it was
    wrong.

    So a zero pinned to a verified reference pose outranks this check and is
    reported as settled rather than flagged — the same precedence
    :attr:`~soarm_sdk.calibration.frame.JointCalibration.suspect` already
    applies to a span mismatch, and for the same reason: a pose that was
    physically checked does not become doubtful because an assumption about
    the hard stops disagrees with it. Only a zero with no such witness gets
    flagged, and then as something to go and confirm.
    """
    if cal is None:
        return ""
    rows = [
        "If the stops were symmetric the travel would straddle zero. A gap "
        "means the zero is out **or** the stops are not symmetric, and this "
        "cannot tell you which — treat it as a question, never a correction:",
        "",
        "| Joint | Travel midpoint | Should be | Gap | Reading |",
        "|---|--:|--:|--:|---|",
    ]
    flagged = False
    for j in cal.joints:
        lo, hi = j.reachable_rad
        mid = math.degrees((lo + hi) / 2)
        u_lo, u_hi = URDF_LIMITS.get(j.name, (0.0, 0.0))
        u_mid = math.degrees((u_lo + u_hi) / 2)
        err = mid - u_mid
        if j.name == "gripper":
            reading = "jaw travel, never symmetric — ignore"
        elif abs(err) < SYMMETRY_WARN_DEG:
            reading = "symmetric, nothing to say"
        elif j.zero_source == "reference_pose":
            # The precedence that matters. A pose-anchored zero was checked
            # against the world; this check was not checked against anything.
            reading = "zero is pose-anchored — so the stops are asymmetric"
        else:
            reading = "**zero may be out by this much — confirm with a pose**"
            flagged = True
        rows.append(
            f"| {j.name} | {mid:+.1f}° | {u_mid:+.1f}° | {err:+.1f}° | {reading} |"
        )
    if flagged:
        rows += [
            "",
            "Those zeros have no verified pose behind them yet. Pin one "
            "above; do **not** subtract the gap.",
        ]
    return "\n".join(rows)


def _format_drift(ctx: Any) -> str:
    """Say when the mirror and the planner are looking at different arms.

    Only the 3-D view reads the in-memory calibration. The planner runs in
    a container and can see nothing but the file; the pose capture, the
    executor and the manifest player each load their own copy from it. An
    unsaved edit is therefore not a pending change, it is a live
    disagreement — and the mirror, being the one thing that shows the edit,
    is the one place it looks like everything is fine.
    """
    try:
        drift = ctx.calibration_drift()
    except Exception:
        return ""
    if not drift:
        return ""
    worst = ", ".join(f"{n} {d:+.1f}°" for n, d in drift)
    return (
        f"🔴 **Unsaved.** The view shows {worst} relative to the file, and the "
        "planner, pose capture and executor all read the file. Save first."
    )


def _format_pinned(handles: Dict[str, Any], rows: List[dict]) -> str:
    """How far the arm has drifted from the pose its zeros were pinned to.

    A re-zero is exact at the instant it happens: the ticks are made to
    mean the pose, so the mirror renders the pose. If the arm then settles
    — and a folded arm held flat by hand settles as soon as the hand goes
    to the mouse — the mirror follows it away, and what the operator sees
    is a mirror that does not show the pose they just pinned. That looks
    exactly like a re-zero that did not take.

    So say which it is. The arm moving after the fact is a fact about the
    arm; it is reported here, against the pose, instead of being left to
    look like a bug.
    """
    pinned = handles.get("pinned")
    if not pinned:
        return ""
    pose = pinned["pose"]
    by_tick = {r["name"]: r["ticks"] for r in rows}
    moved = []
    for name in pose.covers:
        then, now = pinned["ticks"].get(name), by_tick.get(name)
        if then is None or now is None:
            continue
        d = (now - then) * 360.0 / 4096.0
        if abs(d) > SETTLE_TOLERANCE_DEG:
            moved.append(f"{name} {d:+.2f}°")
    if not moved:
        return (
            f"✅ Pinned to **{pose.key}**, and the arm is still there — "
            "the mirror is showing that pose."
        )
    return (
        f"ℹ️ Pinned to **{pose.key}**, but the arm moved since: "
        f"{', '.join(moved)}. The mirror follows the arm, so it no longer "
        "shows the pose — settling, **not** a failed re-zero. If it settled "
        "when you let go, reset it and re-zero where it rests."
    )


def _format_signs(cal: Optional[RobotCalibration]) -> str:
    """Whether the physical direction-sign check has been recorded.

    The sign is the one thing in this tab that no amount of re-zeroing can
    repair, and the only evidence for it is that somebody looked. That
    evidence lived nowhere the tab could write: ``mark_validated`` existed
    on the calibration and nothing in the dashboard ever called it, so the
    acceptance record's sign row was permanently BLOCKED and **Save** could
    never succeed from this tab at all. Step 2 records it; this says what
    is on file.
    """
    if cal is None:
        return ""
    if not cal.validated:
        return (
            "⚠️ **Not confirmed.** Save stays blocked, and anything streaming "
            "a planned trajectory should refuse this file meanwhile."
            + _flipped_note(cal)
        )
    at = str(cal.notes.get("validated_at") or "")[:10]
    out = "✅ **Confirmed**" + (f" {at}" if at else "")
    head, extra = _note_headline(cal.notes.get("validated_by"))
    # Lab notes usually restate their own date; the line already carries it.
    for dup in (f", {at}", f" {at}", f" ({at})"):
        if at and head.endswith(dup):
            head = head[: -len(dup)].rstrip(" ,")
            break
    if head:
        out += f" — {head}"
    if extra:
        out += f" *({extra}; full text in the calibration file)*"
    return out + _flipped_note(cal)


#: How much of a provenance note to show on screen. The note is written for
#: the file and for whoever reads it in a year; the tab needs the headline.
NOTE_HEADLINE_CHARS = 80


def _note_headline(note: Any) -> tuple:
    """``(headline, what_was_left_out)`` for a stored provenance note.

    This arm's ``validated_by`` is 956 characters — longer than every other
    word on the tab put together — because it is a lab record, and the right
    place for a lab record is the file. Printing it whole buried the one
    thing the line exists to say.

    Records joined with ``" | "`` are counted, not concatenated: this file
    has a direction-sign check and an unrelated zero re-measurement sharing
    the field, and running them together read as one sentence about neither.
    """
    if not isinstance(note, str) or not note.strip():
        return "", ""
    parts = [x.strip() for x in note.split(" | ") if x.strip()]
    head = parts[0]
    # A lab note usually opens "<what was done>: <how>"; the half before the
    # colon is the headline already written by whoever recorded it.
    for cut in (head.find(":"), head.find(".")):
        if 0 < cut <= NOTE_HEADLINE_CHARS:
            head = head[:cut]
            break
    else:
        if len(head) > NOTE_HEADLINE_CHARS:
            head = head[:NOTE_HEADLINE_CHARS].rsplit(" ", 1)[0] + "…"
    extra = ""
    if len(parts) > 1:
        extra = f"+{len(parts) - 1} more note{'s' if len(parts) > 2 else ''}"
    elif head != note.strip():
        extra = "truncated"
    return head, extra


def _flipped_note(cal: RobotCalibration) -> str:
    """Name any joint whose sign was flipped and whose zero is now stale."""
    flipped = [j.name for j in cal.joints if j.zero_source == "manual_sign_flip"]
    if not flipped:
        return ""
    return (
        f"\n\n⚠️ Sign flipped on **{', '.join(flipped)}** — the zero was "
        "solved under the old sign, so re-pin it in tab 4."
    )


def _format_stage(report: Any, stages: tuple) -> str:
    """This tab's own acceptance rows, stated plainly."""
    if report is None:
        return "**Blocked:** no calibration loaded."
    rows = [report.stage(st) for st in stages]
    if all(r.passed for r in rows):
        return "✅ **This step is complete.** " + " ".join(r.detail for r in rows)
    return "🔴 **This step is not complete yet.**\n\n" + "\n".join(
        f"- {r.detail}" for r in rows if not r.passed
    )


def _on_tick(ctx: Any, handles: Dict[str, Any]) -> None:
    if not handles:
        return
    cal: Optional[RobotCalibration] = getattr(ctx, "calibration", None)

    # Everything derived from the calibration alone refreshes first, and
    # unconditionally. None of it needs an arm, and the operator has to be
    # able to read which steps are blocked *before* deciding to plug one in.
    _set(handles, "symmetry_md", _format_symmetry(cal))
    _set(handles, "rom_md", _format_rom(cal))
    refresh_discard = handles.get("rom_discard_refresh")
    if refresh_discard is not None:
        refresh_discard()
    _set(handles, "claims_md", _format_claims(cal))
    _set(handles, "signs_md", _format_signs(cal))
    _set(handles, "drift_md", _format_drift(ctx))

    report = CalibrationPipeline(cal).report() if cal is not None else None
    _set(
        handles,
        "pipeline_md",
        report.as_markdown()
        if report is not None
        else "**Blocked:** load a calibration to begin the acceptance pipeline.",
    )
    for md, stages in handles.get("stage_mds", ()):
        md.content = _format_stage(report, stages)

    with ctx.lock:
        positions = dict(ctx.state.positions)
        connected = ctx.state.connected
    if not connected and not positions:
        _set(handles, "table_md", "*Not connected — connect in the sidebar.*")
        return
    rows = _joint_rows(ctx, positions)
    handles["rows"] = rows
    tick = handles.get("homing_tick")
    if tick is not None:
        tick(ctx)
    _set(handles, "pinned_md", _format_pinned(handles, rows))
    _set(handles, "table_md", _format_table(rows, cal))
    _set(handles, "members_md", _format_members(ctx, rows))

    outside = [r["name"] for r in rows if r["over_rad"] > 0]
    _set(
        handles,
        "outside_md",
        ""
        if not outside
        else (
            f"⚠️ **{', '.join(outside)}** outside the URDF's limits. The arm "
            "is fine there; the model cannot draw it, which is why the links "
            "intersect. Not a bad zero — do not re-zero to fix it."
        ),
    )


# ----------------------------------------------------------------------
# The four tabs
#
# One per group of acceptance rows, in the order the work has to happen:
# tolerances gate the grading of everything else, signs cannot be repaired
# by any zero, travel is measured before zeros are pinned inside it, and the
# zeros come last. Each tab ends with its own review and save.
#
# This was one tab of six stacked folders numbered 1..5, and before that the
# same folders emitted out of order (1, 1, 0, 2, 3, 4, 4, 5). The numbering
# got fixed; the length did not. Save lived at the bottom, so the button and
# the row that blocked it were never on screen together.
# ----------------------------------------------------------------------


def _tab_header(server: Any, handles: Dict[str, Any], n: int, title: str,
                what: str, blurb: str) -> None:
    server.gui.add_markdown(f"## {n}. {title}\n{_order_banner(n, what)}\n\n{blurb}")
    _reg(handles, "status_md", server.gui.add_markdown(""))
    _reg(handles, "drift_md", server.gui.add_markdown(""))


def _tab_tolerances(server: Any, ctx: Any, handles: Dict[str, Any]) -> None:
    _tab_header(
        server, handles, 1, "Acceptance tolerances", "required first",
        "Thresholds for this arm. Nothing else can be graded until they "
        "exist, and there are no defaults.",
    )
    _build_acceptance(server, ctx, handles)
    _build_review(server, ctx, handles, tab="tolerances")


def _tab_signs(server: Any, ctx: Any, handles: Dict[str, Any]) -> None:
    _tab_header(
        server, handles, 2, "Direction signs", "check before anything is pinned",
        "No zero can repair a wrong sign, so signs come before zeros.",
    )
    _build_live_state(server, handles)
    _build_signs(server, ctx, handles)
    _build_review(server, ctx, handles, tab="signs")


def _tab_travel(server: Any, ctx: Any, handles: Dict[str, Any],
                fk_update_fn: Optional[Callable[[Dict[int, int]], None]]) -> None:
    _tab_header(
        server, handles, 3, "Travel limits", "measure the mechanism",
        "Where the mechanism stops, as `tick_min`/`tick_max`. A sweep "
        "midpoint is **not** a zero — tab 4 sets those.",
    )
    _build_live_state(server, handles)
    _build_rom(server, ctx, handles, fk_update_fn)
    _build_review(server, ctx, handles, tab="travel")


def _tab_zeros(server: Any, ctx: Any, handles: Dict[str, Any],
               ghost_fn: Optional[Callable[[Optional[Dict[str, float]]], None]]) -> None:
    _tab_header(
        server, handles, 4, "Zeros", "make the mirror agree with the arm",
        "Put the arm in a pose you can **verify**, then tell the model that "
        "is where it is. The zero comes from the pose alone.\n\n"
        "**Do tab 3 first:** re-centring a servo rewrites the raw ticks a "
        "zero is pinned to.",
    )
    _build_live_state(server, handles)
    _build_rezero(server, ctx, handles, ghost_fn)
    _build_review(server, ctx, handles, tab="zeros")


def _build_review(server: Any, ctx: Any, handles: Dict[str, Any], *, tab: str) -> None:
    """This tab's own result, the whole record, and Save — at the end of each tab.

    Save writes the entire file and refuses while *any* row is blocked, so
    the full record is shown next to it: a refusal here can be caused by a
    tab you have not opened yet, and that has to be visible from where the
    button is rather than inferred.
    """
    with server.gui.add_folder("Review & save"):
        # Keyed by tab rather than through _set: each tab's rows differ, so
        # these carry their own stage tuple instead of shared content.
        handles.setdefault("stage_mds", []).append(
            (server.gui.add_markdown("*Waiting…*"), TAB_STAGES[tab])
        )

        server.gui.add_markdown(
            "---\nFull record. **Save** refuses while any row is BLOCKED, "
            "including other tabs'. Previous file is backed up first."
        )
        _reg(handles, "pipeline_md", server.gui.add_markdown("*Waiting…*"))
        save_btn = server.gui.add_button("Save calibration", color="green")
        reload_btn = server.gui.add_button("Discard and reload from disk")
        _reg(handles, "save_md", server.gui.add_markdown(""))

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
            handles["signs_flipped"] = set()
            for h in handles.get("nudges", {}).values():
                h.value = 0.0
            _say(handles, f"reloaded {path.name} — unsaved changes discarded")


def _build_live_state(server: Any, handles: Dict[str, Any]) -> None:
    """What the arm and the model are doing, independent of any step.

    Not a step, and repeated on every tab that needs it: it is the reading
    each step is checked against, and sending the operator to another tab to
    see it would defeat the split.
    """
    with server.gui.add_folder("Live arm state", expand_by_default=False):
        server.gui.add_markdown(
            "*Not a step* — the live reading. **URDF limit** flags a pose "
            "the model cannot draw; that is not a calibration error."
        )
        _reg(handles, "outside_md", server.gui.add_markdown(""))
        _reg(handles, "table_md", server.gui.add_markdown("*Waiting…*"))
        _reg(handles, "pinned_md", server.gui.add_markdown(""))
        _reg(handles, "members_md", server.gui.add_markdown("*Waiting…*"))


#: What the tolerance fields start at. These are *starting values in a form*,
#: not a fallback: nothing is written until the operator presses Record, and
#: :meth:`AcceptanceTolerances.from_notes` still returns ``None`` for a
#: calibration that recorded none, so no code path anywhere infers a
#: threshold that nobody approved. The distinction is the whole point of the
#: "no SDK-wide fallback" rule — it forbids *assuming* a number, not
#: suggesting one the operator has to look at and commit.
DEFAULT_POSE_REPEATABILITY_DEG = 3.0
DEFAULT_MODEL_DEVIATION_DEG = 3.0
DEFAULT_ROM_REPEATABILITY_TICKS = 30.0


def _build_acceptance(server: Any, ctx: Any, handles: Dict[str, Any]) -> None:
    with server.gui.add_folder("Record acceptance tolerances"):
        server.gui.add_markdown(
            "Stored with the calibration, per arm and task class. The values "
            "below are **starting points — review them**; nothing is recorded "
            "until you press the button."
        )
        pose_h = server.gui.add_number(
            "Reference-pose repeatability (deg)",
            initial_value=DEFAULT_POSE_REPEATABILITY_DEG,
            min=0.0,
            step=0.05,
        )
        deviation_h = server.gui.add_number(
            "Model deviation watchdog (deg)",
            initial_value=DEFAULT_MODEL_DEVIATION_DEG,
            min=0.0,
            step=0.1,
        )
        rom_h = server.gui.add_number(
            "ROM endpoint repeatability (ticks)",
            initial_value=DEFAULT_ROM_REPEATABILITY_TICKS,
            min=0.0,
            step=1.0,
        )
        record_btn = server.gui.add_button("Record acceptance tolerances", color="blue")

        cal = getattr(ctx, "calibration", None)
        if cal is not None:
            existing = AcceptanceTolerances.from_notes(cal.notes)
            if existing is not None:
                pose_h.value = math.degrees(existing.pose_repeatability_rad)
                deviation_h.value = math.degrees(existing.model_deviation_rad)
                rom_h.value = existing.rom_endpoint_repeatability_ticks

        @record_btn.on_click
        def _record(_: Any) -> None:
            cal = getattr(ctx, "calibration", None)
            if cal is None:
                return _say(handles, "load a calibration before recording tolerances")
            try:
                endpoint_repeatability = float(rom_h.value)
                if not endpoint_repeatability.is_integer():
                    raise ValueError(
                        "ROM endpoint repeatability must be a whole number of ticks"
                    )
                tolerances = AcceptanceTolerances(
                    pose_repeatability_rad=math.radians(float(pose_h.value)),
                    model_deviation_rad=math.radians(float(deviation_h.value)),
                    rom_endpoint_repeatability_ticks=int(endpoint_repeatability),
                )
            except ValueError as exc:
                return _say(handles, f"invalid acceptance tolerances: {exc}")
            for target in _live_calibrations(ctx, handles):
                target.notes = dict(target.notes)
                target.notes["acceptance_tolerances"] = tolerances.to_dict()
            _say(handles, "acceptance tolerances recorded; save calibration to make them active")


#: Step 2's per-joint verdict. "Opposite" is a finding, not a setting — it
#: blocks the check rather than flipping the sign, because a flipped sign
#: invalidates that joint's zero and the fix is to re-pin it in Step 4.
SIGN_UNCHECKED = "— not checked —"
SIGN_OK = "Correct — model follows"
SIGN_INVERTED = "Opposite — model opposes"


def _build_signs(server: Any, ctx: Any, handles: Dict[str, Any]) -> None:
    with server.gui.add_folder("Verify direction signs"):
        server.gui.add_markdown(
            "One joint at a time, arm clear. Move it: does the model turn "
            "the **same** way? *Opposite* flips the sign at once, so you can "
            "confirm on the same motion. Nothing here commands the arm."
        )
        _reg(handles, "signs_md", server.gui.add_markdown(""))
        verdicts: Dict[str, Any] = {}
        for name in SOARM100_JOINT_NAMES:
            # Defaults to "Correct": every joint's assumed sign (see
            # DEFAULT_DIRECTION_SIGN_OVERRIDES) is expected to check out now,
            # wrist_roll included. This trades away the guard that used to
            # force a look at each dropdown before Confirm would accept —
            # pressing it without touching a control now records a physical
            # check that may not have happened. Deliberate, not an oversight.
            verdicts[name] = server.gui.add_dropdown(
                name,
                options=[SIGN_UNCHECKED, SIGN_OK, SIGN_INVERTED],
                initial_value=SIGN_OK,
            )
        handles["sign_verdicts"] = verdicts
        confirm_btn = server.gui.add_button("Confirm direction signs", color="blue")

        for h in verdicts.values():
            h.on_update(lambda _=None: _apply_signs(ctx, handles))

        @confirm_btn.on_click
        def _confirm(_: Any) -> None:
            if getattr(ctx, "calibration", None) is None:
                return _say(handles, "load a calibration before confirming signs")
            picked = {n: str(h.value) for n, h in verdicts.items()}
            missing = [n for n, v in picked.items() if v == SIGN_UNCHECKED]
            if missing:
                return _say(handles, f"not checked yet: {', '.join(missing)}")
            flipped = sorted(n for n, v in picked.items() if v == SIGN_INVERTED)
            how = "per-joint direction check from the dashboard: " + ", ".join(
                f"{n} {'flipped' if picked[n] == SIGN_INVERTED else 'correct'}"
                for n in picked
            )
            for target in _live_calibrations(ctx, handles):
                target.mark_validated(how)
            _say(
                handles,
                "signs confirmed"
                + (
                    f" — {', '.join(flipped)} flipped, so re-pin "
                    f"{'it' if len(flipped) == 1 else 'them'} in Step 4 "
                    "before saving"
                    if flipped
                    else " — Step 5 to keep it"
                ),
            )


def _apply_signs(ctx: Any, handles: Dict[str, Any]) -> None:
    """Flip the signs the dropdowns currently report as opposite, live.

    Applied on selection rather than on the button so the mirror reverses
    while the operator is still moving the joint — the fix is confirmed by
    the same motion that found the fault, instead of on a later pass.

    Derived from the dropdowns each time rather than accumulated, so
    changing an answer back undoes the flip. Any change invalidates the
    pose provenance: :func:`~soarm_sdk.calibration.frame.rezero_from_pose`
    folds the sign into the zero it solves for, so a pose pinned under the
    previous convention no longer describes this arm.
    """
    verdicts = handles.get("sign_verdicts")
    if not verdicts:
        return
    want = {n for n, h in verdicts.items() if str(h.value) == SIGN_INVERTED}
    delta = want ^ handles.get("signs_flipped", set())
    if not delta:
        return
    cals = _live_calibrations(ctx, handles)
    if not cals:
        return
    for cal in cals:
        for idx, joint in enumerate(cal.joints):
            if joint.name in delta:
                cal.joints[idx] = joint.with_direction_flipped()
        cal.notes = dict(cal.notes)
        cal.notes.pop("rezeroed_from_dashboard", None)
        cal.notes.pop("reference_pose_samples", None)
        # The recorded check described the old signs; it no longer holds.
        cal.validated = False
    handles["signs_flipped"] = want
    handles.pop("pinned", None)
    _say(
        handles,
        f"flipped {', '.join(sorted(delta))} — mirror reversed; "
        + (
            f"now inverted: {', '.join(sorted(want))}. Re-pin in Step 4."
            if want
            else "no joints inverted."
        ),
    )


def _live_calibrations(ctx: Any, handles: Dict[str, Any]) -> List[RobotCalibration]:
    """The calibration objects an edit must reach, deduplicated.

    While a nudge slider is off zero these are two different objects: the
    live one the mirror renders, and the un-nudged base the nudge is
    recomputed from on every slider move. Writing provenance to only the
    live one loses it the next time a slider twitches, because
    :func:`_build_nudge`'s ``_apply`` rebuilds from the base.
    """
    seen: Dict[int, RobotCalibration] = {}
    for cal in (getattr(ctx, "calibration", None), handles.get("nudge_base")):
        if cal is not None:
            seen[id(cal)] = cal
    return list(seen.values())


def _build_rom(
    server: Any,
    ctx: Any,
    handles: Dict[str, Any],
    fk_update_fn: Optional[Callable[[Dict[int, int]], None]] = None,
) -> None:
    with server.gui.add_folder("Measure travel limits"):
        server.gui.add_markdown(
            "**Measure every joint twice** — the row grades repeatability, "
            "so one pass can never satisfy it.\n\n"
            "Once accepted, this travel **replaces** the URDF limits for the "
            "planner and `ServoRobot`. Until then the two are intersected."
        )
        _reg(handles, "rom_md", server.gui.add_markdown("*Waiting…*"))
        from .setup import _build_homing

        _build_homing(
            server,
            ctx,
            fk_update_fn=fk_update_fn,
            heading=False,
            handles=handles,
            on_endpoints=lambda by_name, simulated: _record_endpoints(
                ctx, handles, by_name, simulated
            ),
        )
        _build_discard_pass(server, ctx, handles)


def _pass_label(index: int, entry: Dict[str, Any]) -> str:
    """One recorded pass, as the discard dropdown shows it.

    Carries the numbers, not just an index, so the operator recognises the
    entry from what :func:`_format_rom` already printed rather than having
    to count table rows.
    """
    tag = " (simulated)" if entry.get("simulated") else ""
    return f"{index + 1}: {entry.get('min')}…{entry.get('max')}{tag}"


def _pass_options(cal: Optional[RobotCalibration], joint_name: str) -> List[str]:
    raw = cal.notes.get("rom_endpoint_samples") if cal is not None else None
    entries = [e for e in (raw or {}).get(joint_name, []) if isinstance(e, dict)]
    if not entries:
        return ["—"]
    return [_pass_label(i, e) for i, e in enumerate(entries)]


def _build_discard_pass(server: Any, ctx: Any, handles: Dict[str, Any]) -> None:
    """Remove one recorded pass — a stall or a mis-click otherwise counts as
    real evidence and drags the whole row's repeatability number with it,
    the way one degenerate ``min == max`` sample can make a joint that
    swept cleanly twice report as repeatable to thousands of ticks.

    Discards by position, validated against the *current* list at click
    time (label and index both), because the list can change — a fresh
    sweep pass appending — between when this dropdown was populated and
    when the button is pressed. A stale selection is refused and the
    dropdown refreshed, rather than silently removing whatever now sits at
    that index.
    """
    with server.gui.add_folder("Discard a bad pass", expand_by_default=False):
        server.gui.add_markdown(
            "Remove one recorded pass for one joint. Does not touch the "
            "others, and does not un-measure the joint — record another "
            "pass afterward if this leaves it below two."
        )
        joint_h = server.gui.add_dropdown(
            "Joint", options=list(SOARM100_JOINT_NAMES),
            initial_value=SOARM100_JOINT_NAMES[0],
        )
        pass_h = server.gui.add_dropdown("Pass", options=["—"], initial_value="—")
        discard_btn = server.gui.add_button("Discard this pass", color="red")

        def _refresh() -> None:
            options = _pass_options(getattr(ctx, "calibration", None), joint_h.value)
            if options != pass_h.options:
                pass_h.options = options
            if pass_h.value not in options:
                pass_h.value = options[0]

        _refresh()
        handles["rom_discard_refresh"] = _refresh

        @joint_h.on_update
        def _on_joint(_: Any) -> None:
            _refresh()

        @discard_btn.on_click
        def _discard(_: Any) -> None:
            cal = getattr(ctx, "calibration", None)
            if cal is None:
                return _say(handles, "no calibration loaded")
            name, selected = joint_h.value, pass_h.value
            if selected == "—":
                return _say(handles, f"no recorded passes for {name}")

            raw = cal.notes.get("rom_endpoint_samples")
            entries = raw.get(name) if isinstance(raw, dict) else None
            try:
                idx = int(selected.split(":", 1)[0]) - 1
            except ValueError:
                return _say(handles, "could not read the selected pass")
            if (
                not isinstance(entries, list)
                or not (0 <= idx < len(entries))
                or _pass_label(idx, entries[idx]) != selected
            ):
                _refresh()
                return _say(handles, "recorded passes changed — refreshed, try again")

            removed = entries[idx]
            remaining = len(entries) - 1
            for target in _live_calibrations(ctx, handles):
                t_raw = target.notes.get("rom_endpoint_samples")
                t_entries = t_raw.get(name) if isinstance(t_raw, dict) else None
                if not isinstance(t_entries, list) or idx >= len(t_entries):
                    continue
                new_entries = t_entries[:idx] + t_entries[idx + 1:]
                target.notes = dict(target.notes)
                target.notes["rom_endpoint_samples"] = {**t_raw, name: new_entries}
                if new_entries:
                    last = new_entries[-1]
                    if "min" in last and "max" in last:
                        j = target.names.index(name)
                        target.joints[j] = target.joints[j].with_travel(
                            last["min"], last["max"]
                        )

            _refresh()
            _say(
                handles,
                f"discarded {name} pass {idx + 1} "
                f"({removed.get('min')}…{removed.get('max')}) — "
                f"{remaining} pass(es) remain"
                + ("; measure again for repeatability" if remaining < 2 else ""),
            )


def _record_endpoints(
    ctx: Any, handles: Dict[str, Any], by_name: Dict[str, tuple], simulated: bool
) -> None:
    """Append one travel observation per joint to the calibration's evidence.

    The gap this closes: the sweep controls wrote servo EEPROM and
    ``soarm100_rom.json``, and nothing else. ``rom_endpoint_samples`` — the
    evidence the ROM acceptance row actually grades — was written only by the
    standalone ``soarm-calibrate-rom``, so measuring travel in this tab left
    its own row BLOCKED no matter how carefully it was done. The control and
    the row it was supposed to satisfy were never connected.

    The measured stops are also written onto the joints themselves. They are
    what :meth:`~soarm_sdk.calibration.frame.RobotCalibration.reachable_limits`
    reports and what ``ServoRobot`` clamps commands against, so recording the
    evidence while leaving those stale would be its own quiet disagreement.
    """
    cals = _live_calibrations(ctx, handles)
    if not cals:
        return _say(handles, "no calibration loaded — travel not recorded")
    counts: Dict[str, int] = {}
    for cal in cals:
        prior = cal.notes.get("rom_endpoint_samples")
        samples = (
            {k: list(v) for k, v in prior.items() if isinstance(v, list)}
            if isinstance(prior, dict)
            else {}
        )
        for name, (lo, hi) in by_name.items():
            samples.setdefault(name, []).append(
                {"min": int(lo), "max": int(hi), "simulated": bool(simulated)}
            )
        cal.notes = dict(cal.notes)
        cal.notes["rom_endpoint_samples"] = samples
        for idx, joint in enumerate(cal.joints):
            if joint.name in by_name:
                lo, hi = by_name[joint.name]
                cal.joints[idx] = joint.with_travel(lo, hi)
        counts = {n: len(v) for n, v in samples.items()}
    worst = min(counts.values()) if counts else 0
    _say(
        handles,
        f"recorded travel for {len(by_name)} joint(s)"
        + (" — **simulated**, which cannot be accepted" if simulated else "")
        + f"; every joint now has {worst} pass(es)"
        + (" — measure again for repeatability" if worst < 2 else ""),
    )


def _format_rom(cal: Optional[RobotCalibration]) -> str:
    """How many travel passes each joint has, and what is still missing."""
    if cal is None:
        return "*No calibration loaded.*"
    raw = cal.notes.get("rom_endpoint_samples")
    if not isinstance(raw, dict) or not raw:
        return (
            "**No travel passes yet.** Sweep or hand-record every joint, "
            "then do it again — one pass cannot show repeatability."
        )
    lines = ["| Joint | Passes | Endpoints seen | Reading |", "|---|--:|---|---|"]
    for j in cal.joints:
        entries = [e for e in raw.get(j.name, []) if isinstance(e, dict)]
        seen = ", ".join(f"{e.get('min')}…{e.get('max')}" for e in entries) or "—"
        if any(e.get("simulated") for e in entries):
            reading = "**simulated — cannot be accepted**"
        elif len(entries) < 2:
            reading = "**needs another pass**"
        else:
            lows = [e["min"] for e in entries if "min" in e]
            highs = [e["max"] for e in entries if "max" in e]
            spread = max(max(lows) - min(lows), max(highs) - min(highs))
            reading = f"repeatable to {spread} ticks"
        lines.append(f"| {j.name} | {len(entries)} | {seen} | {reading} |")

    wrapped = [
        j.name
        for j in cal.joints
        for e in raw.get(j.name, [])
        if isinstance(e, dict)
        and "min" in e
        and "max" in e
        and abs(int(e["max"]) - int(e["min"])) >= WRAP_SUSPECT_TICKS
    ]
    if wrapped:
        lines += [
            "",
            f"🔴 **{', '.join(sorted(set(wrapped)))}: travel crosses the "
            "4095/0 wrap**, so this span is the encoder's range, not the "
            "joint's. No tick→radian map fits a coordinate that jumps, and no "
            "calibration zero can fix it — the servo's homing offset must "
            "move: `soarm-calibrate-rom --recentre`.\n\n"
            "Do it **before** pinning zeros in tab 4; it rewrites raw ticks.",
        ]
    return "\n".join(lines)


def _build_rezero(
    server: Any,
    ctx: Any,
    handles: Dict[str, Any],
    ghost_fn: Optional[Callable[[Optional[Dict[str, float]]], None]] = None,
) -> None:
    with server.gui.add_folder("Pin a zero from a reference pose"):
        # This step is one idea: put the arm somewhere whose geometry can be
        # checked without trusting any number, then tell the model that is
        # where it is. Nothing here is derived from the hard stops.
        #
        # The symmetry cross-check used to open this folder, and that framed
        # the step as "find the midpoint of the travel" — which is the one
        # method this step deliberately does not use, and which the arm's own
        # asymmetric stops make wrong. It now sits at the bottom, collapsed,
        # labelled as setting nothing.
        server.gui.add_markdown(
            "1. Pick a pose. The translucent ghost shows it.\n"
            "2. Set the real arm onto the ghost; let it settle where it "
            "**rests**, not where you hold it.\n"
            "3. **Re-zero** — the mirror snaps onto the pose.\n"
            "4. Close any remaining gap with the alignment sliders."
        )

        keys = list(REFERENCE_POSES)
        pose_h = server.gui.add_dropdown(
            "Pose", options=keys, initial_value=keys[0]
        )
        detail_md = server.gui.add_markdown("")
        ghost_h = server.gui.add_checkbox(
            "Show reference ghost in the 3-D view", initial_value=True
        )
        server.gui.add_markdown("Only the joints this pose constrains move.")
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
            # A reference pose is a place the arm can physically be; the
            # URDF's limits are a conservative opinion about travel. Where
            # they disagree the mirror renders the pose out of range and the
            # links intersect — while the operator is holding exactly the
            # pose they were asked for. Say so here, or it reads as the
            # re-zero having broken something.
            past = []
            for name, q in pose.as_cfg().items():
                lo, hi = URDF_LIMITS.get(name, (float("-inf"), float("inf")))
                over = max(lo - q, q - hi, 0.0)
                if math.degrees(over) > LIMIT_TOLERANCE_DEG:
                    past.append(f"{name} by {math.degrees(over):.1f}°")
            if past:
                body += [
                    "",
                    f"⚠️ Outside the URDF's limits ({', '.join(past)}), so the "
                    "render will show links intersecting while you hold it. "
                    "Expected; re-zeroing is unaffected.",
                ]
            if not pose.covers:
                body.append("")
                body.append(
                    "Constrains nothing verifiable — re-zeroing is disabled."
                )
            return "\n".join(body)

        def _refresh_ghost() -> None:
            if ghost_fn is None:
                return
            pose = REFERENCE_POSES[pose_h.value]
            ghost_fn(pose.as_cfg() if ghost_h.value else None)

        detail_md.content = _describe(pose_h.value)
        _refresh_ghost()

        @pose_h.on_update
        def _on_pose(_: Any) -> None:
            detail_md.content = _describe(pose_h.value)
            _refresh_ghost()

        @ghost_h.on_update
        def _on_ghost(_: Any) -> None:
            _refresh_ghost()

        @rezero_btn.on_click
        def _rezero(_: Any) -> None:
            _do_rezero(ctx, handles, REFERENCE_POSES[pose_h.value])

        _build_alignment(server, ctx, handles)
        _build_claims(server, ctx, handles)
        _build_symmetry(server, handles)


def _unsupported_pose_claims(cal: Optional[RobotCalibration]) -> List[str]:
    """Joints claiming a pose-anchored zero that no recorded pose supports.

    ``rezero_from_pose`` only relabels the joints its pose constrains, but a
    file written before that was enforced can have every joint stamped
    ``reference_pose`` regardless — this arm's was, on 2026-09-13, all six
    against a ``folded_flat`` that constrains two. Four of those zeros claim
    evidence that does not exist.

    Neither ``gripper`` nor ``wrist_roll`` is constrained by *any* pose here,
    so for them the claim can never be made true by pinning. It can only be
    withdrawn.
    """
    if cal is None:
        return []
    entry = cal.notes.get("rezeroed_from_dashboard")
    covered: set = set()
    if isinstance(entry, dict):
        pose = REFERENCE_POSES.get(entry.get("pose"))
        if pose is not None:
            covered = set(pose.covers)
    return [
        j.name
        for j in cal.joints
        if j.zero_source == "reference_pose" and j.name not in covered
    ]


def _build_claims(server: Any, ctx: Any, handles: Dict[str, Any]) -> None:
    """Withdraw pose claims that no pose backs.

    The one repair for a false provenance claim, and deliberately the only
    thing here that can touch ``zero_source`` without a pose: it strictly
    removes claims, so it cannot be used to make a zero *look* verified.
    """
    with server.gui.add_folder(
        "Withdraw an unsupported pose claim", expand_by_default=False
    ):
        server.gui.add_markdown(
            "A zero labelled *pinned to a pose* that the recorded pose does "
            "not constrain. Withdrawing marks it **unknown** — the label "
            "changes, the zero does not. No pose covers `gripper` or "
            "`wrist_roll`, so a claim on either can only be withdrawn."
        )
        _reg(handles, "claims_md", server.gui.add_markdown("*Waiting…*"))
        btn = server.gui.add_button("Withdraw unsupported claims", color="orange")

        @btn.on_click
        def _withdraw(_: Any) -> None:
            cals = _live_calibrations(ctx, handles)
            if not cals:
                return _say(handles, "no calibration loaded")
            names = _unsupported_pose_claims(getattr(ctx, "calibration", None))
            if not names:
                return _say(handles, "no unsupported claims to withdraw")
            for cal in cals:
                for idx, joint in enumerate(cal.joints):
                    if joint.name in names:
                        cal.joints[idx] = joint.with_claim_withdrawn()
                cal.notes = dict(cal.notes)
                cal.notes["withdrawn_pose_claims"] = {
                    "joints": sorted(names),
                    "at": datetime.now(timezone.utc).isoformat(),
                }
            _say(
                handles,
                f"withdrew the pose claim on {', '.join(sorted(names))} — "
                "their zeros are unchanged, now marked unknown",
            )


def _format_claims(cal: Optional[RobotCalibration]) -> str:
    if cal is None:
        return "*No calibration loaded.*"
    names = _unsupported_pose_claims(cal)
    if not names:
        return "✅ Every pose-anchored zero is backed by a pose that constrains it."
    entry = cal.notes.get("rezeroed_from_dashboard")
    pose_key = entry.get("pose") if isinstance(entry, dict) else None
    witness = f"`{pose_key}`" if pose_key else "*no pose at all*"
    return (
        f"⚠️ **{', '.join(names)}** "
        f"{'claims' if len(names) == 1 else 'claim'} a reference-pose zero, "
        f"but the recorded witness is {witness}, which does not constrain "
        f"{'it' if len(names) == 1 else 'them'}."
    )


def _build_symmetry(server: Any, handles: Dict[str, Any]) -> None:
    """The hard-stop symmetry cross-check — a hint, and never an input.

    Collapsed, and last. It is the only thing in this tab that reasons from
    the travel rather than from a verified pose, and it has already produced
    one false positive on this arm: ``shoulder_lift`` travels 126 deg one way
    and 89 the other because the structure blocks it, and read as a finding
    that says "the zero is 18.6 deg out". It is not.

    So it is kept for the one thing it is good for — naming joints whose zero
    has no pose witness yet — and kept away from anything that sets a zero.
    """
    with server.gui.add_folder(
        "Cross-check: hard-stop symmetry (sets nothing)", expand_by_default=False
    ):
        server.gui.add_markdown(
            "*A hint, not a way to find a zero.* Where it disagrees with a "
            "pose you verified, **the pose wins.**"
        )
        _reg(handles, "symmetry_md", server.gui.add_markdown("*Waiting…*"))


def _alignment_range(name: str) -> tuple:
    """Slider bounds for one joint, in degrees, from its own range of motion.

    Every slider used to run a flat +-45 deg, which matches no joint on this
    arm: it is far past what the gripper can do (-10 to +100 deg of jaw) and
    nowhere near ``wrist_roll``'s -157 to +163. A slider whose ends mean
    nothing physical invites dialling the mirror somewhere the joint cannot
    go, which then renders as a pose the arm can never hold.

    The bound is the joint's URDF travel, so a slider at its end has moved
    the mirror exactly as far as the joint itself reaches, and no further.
    """
    lo, hi = URDF_LIMITS.get(name, (float("-inf"), float("inf")))
    if not (math.isfinite(lo) and math.isfinite(hi)):
        return (-NUDGE_LIMIT_DEG, NUDGE_LIMIT_DEG)
    return (round(math.degrees(lo), 2), round(math.degrees(hi), 2))


def _build_alignment(server: Any, ctx: Any, handles: Dict[str, Any]) -> None:
    """Per-joint sliders that move the mirror onto the ghost.

    These move what the ticks are taken to *mean*, never the arm — the same
    zero edit the re-zero button makes in one shot, done by eye instead.
    They are the endpoint of the ghost: the operator sets the real arm to
    the translucent target, and where the solid mirror still disagrees, this
    is what closes the gap.

    This used to be its own numbered step, after the pin. Two controls
    editing the same zeros in two places is one control too many, and the
    later one had no visual target to work against — you nudged until it
    "looked right", with nothing in the scene defining right.
    """
    with server.gui.add_folder("Fine alignment — move the mirror onto the ghost"):
        server.gui.add_markdown(
            "For a member still off the ghost after pinning. Moves the "
            "mirror, **not the arm**. A slider off zero discards that joint's "
            "pose provenance — prefer re-pinning. Save below to keep it."
        )
        nudges: Dict[str, Any] = {}
        for name in SOARM100_JOINT_NAMES:
            lo, hi = _alignment_range(name)
            nudges[name] = server.gui.add_slider(
                name,
                min=lo,
                max=hi,
                step=0.25,
                initial_value=0.0,
            )
        handles["nudges"] = nudges
        reset_btn = server.gui.add_button("Reset alignment")

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
                live.notes.pop("rezeroed_from_dashboard", None)
                live.notes.pop("reference_pose_samples", None)
            ctx.calibration = live

        for h in nudges.values():
            h.on_update(_apply)

        @reset_btn.on_click
        def _reset(_: Any) -> None:
            for h in nudges.values():
                h.value = 0.0
            _apply()
            _say(handles, "alignment reset")

    handles["nudge_base"] = getattr(ctx, "calibration", None)


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

    # An arm held in position by hand is not in that position; it is being
    # put there, and it leaves as soon as you let go to press the button.
    # Pin a zero to that and the zero carries the holding force. Sample
    # twice and refuse while anything is still moving — the same check
    # read_pose.capture makes before handing a pose to the planner.
    time.sleep(SETTLE_S)
    with ctx.lock:
        again = dict(ctx.state.positions)
    covered_ids = [
        sid
        for sid, name in zip(ctx.joint_ids, SOARM100_JOINT_NAMES)
        if name in pose.covers
    ]
    drifting = []
    for sid, name in zip(ctx.joint_ids, SOARM100_JOINT_NAMES):
        if sid not in covered_ids:
            continue
        b = again.get(sid)
        if b is None:
            continue
        moved = abs(b - positions[sid]) * 360.0 / 4096.0
        if moved > SETTLE_TOLERANCE_DEG:
            drifting.append(f"{name} {moved:.2f}°")
    if drifting:
        _say(
            handles,
            f"still moving ({', '.join(drifting)} in {SETTLE_S:.1f}s) — let it "
            "settle where it rests; a zero pinned while you hold the arm "
            "leaves with your hand",
        )
        return

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
    new.notes["reference_pose_samples"] = [
        {n: int(t) for n, t in zip(cal.names, ticks)},
        {n: int(again[sid]) for sid, n in zip(ctx.joint_ids, cal.names)},
    ]
    ctx.calibration = new
    handles["nudge_base"] = new
    # So that an arm which settles afterwards reads as an arm that settled,
    # not as a re-zero that failed to take. Without this the mirror simply
    # stops matching the pose and there is nothing to say why.
    handles["pinned"] = {
        "pose": pose,
        "ticks": dict(zip(SOARM100_JOINT_NAMES, ticks)),
    }
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


def _cal_path(ctx: Any) -> Path:
    from ..fk import DEFAULT_CALIBRATION_PATH

    p = getattr(ctx, "calibration_path", None)
    return Path(p) if p is not None else DEFAULT_CALIBRATION_PATH


def _do_save(ctx: Any, handles: Dict[str, Any]) -> None:
    cal: Optional[RobotCalibration] = getattr(ctx, "calibration", None)
    if cal is None:
        return _say_save(handles, "nothing to save")
    report = CalibrationPipeline(cal).report()
    if not report.ready:
        return _say_save(
            handles,
            "🔴 **Not saved.** Steps are still blocked — nothing was "
            "written and the file on disk is untouched:\n\n"
            + report.as_markdown(),
        )
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
            return _say_save(handles, f"refusing to save — could not back up: {exc}")
    else:
        backup = None

    try:
        cal.save(path)
    except Exception as exc:
        logger.exception("calibration save failed")
        return _say_save(handles, f"save failed: {exc}")

    handles["nudge_base"] = cal
    for h in handles.get("nudges", {}).values():
        h.value = 0.0
    _say_save(
        handles,
        f"✅ **Saved** {path.name}"
        + (f" (previous version backed up as {backup.name})" if backup else ""),
    )


def _say(handles: Dict[str, Any], msg: str) -> None:
    """Report the result of the last action, at the top of the tab.

    Single-line messages are italicised as asides. Multi-line ones are not:
    the save refusal is a markdown table, and wrapping a table in ``*...*``
    renders it as literal asterisks around broken rows — so the one message
    the operator most needs to read was the one they could not.
    """
    _set(handles, "status_md", f"*{msg}*" if "\n" not in msg else msg)


def _say_save(handles: Dict[str, Any], msg: str) -> None:
    """Report a save result at the top *and* beside the button that caused it.

    Save sits at the bottom of the tab and its reply went only to the status
    line at the top, several screens up — a refusal (five table rows naming
    what is still blocked) rendered where nobody was looking, so a refused
    save was indistinguishable from a save that worked.
    """
    _say(handles, msg)
    _set(handles, "save_md", msg)
