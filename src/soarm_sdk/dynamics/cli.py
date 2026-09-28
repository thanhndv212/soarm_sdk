#!/usr/bin/env python3
"""Record an excitation run for dynamic (gravity + friction) identification.

    soarm-identify-record --arm-id thanh_arm --out runs/ident01
    soarm-identify-record --dry-run --out runs/ident-dry

RUNS ON THE HOST, with the arm connected, calibrated, and free to move.

Streams a slow, smooth multi-joint excitation (see
:mod:`soarm_sdk.dynamics.excitation`) and records joint angle, velocity,
servo current and load in the URDF joint frame, in the directory format
FIGAROH's ``examples/so101/`` reads (see :mod:`soarm_sdk.dynamics.log`).
Identify from it there, then load the result back with
:class:`soarm_sdk.dynamics.IdentifiedDynamics`.

Before anything moves
---------------------
* The calibration must be ``validated`` (direction signs confirmed on the
  arm), as for any planned motion; ``--allow-unvalidated`` overrides it for
  a first rough look. A wrong sign here flips that joint's gravity torque in
  the fit, which is worse than no model at all.
* The whole planned motion is checked against the URDF: every link-frame
  origin must stay above ``--min-height`` in the base frame. That is a
  coarse check — link origins, not meshes — so keep the table clear anyway.
* The motion stays ``--margin`` inside the effective joint limits (declared
  limits intersected with, or replaced by, the measured travel).
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from ..calibration.frame import RobotCalibration
from ..robot.base import load_robot_config
from .excitation import ExcitationSpec, fourier_excitation
from .gravity import GravityModel
from .record import ExcitationAborted, record_excitation

__all__ = ["main"]

DEFAULT_CALIBRATION = Path.home() / ".soarm_sdk" / "calibration.json"

# Workspace-relative, like the dashboards' default: resolves only inside a
# soarm-ws checkout. Pass --urdf elsewhere.
_DEFAULT_URDF = (
    Path(__file__).resolve().parents[4]
    / "SO-ARM100"
    / "Simulation"
    / "SO101"
    / "so101_new_calib.urdf"
)


class _SimClock:
    """A clock that advances only when slept on, so a dry run is instant."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, s: float) -> None:
        self.now += max(0.0, s)


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--device", default="auto", help="serial port, or 'auto'")
    ap.add_argument("--config", default="so101", help="robot config (joint names, limits)")
    ap.add_argument("--calibration", default=str(DEFAULT_CALIBRATION),
                    help="URDF-frame calibration for this arm")
    ap.add_argument("--allow-unvalidated", action="store_true",
                    help="run even if the calibration's direction signs are unconfirmed")
    ap.add_argument("--arm-id", default=None, help="recorded in the log's metadata")
    ap.add_argument("--urdf", default=str(_DEFAULT_URDF),
                    help="URDF for the clearance check")
    ap.add_argument("--out", default=None,
                    help="output directory (default: identification_runs/<arm>-<utc time>)")
    ap.add_argument("--joints", default=None,
                    help="joints to excite, comma-separated (default: every joint but "
                         "the gripper); the rest hold --center")
    ap.add_argument("--center", default=None,
                    help="comma-separated pose to centre the motion on, rad, URDF frame, "
                         "one value per config joint (default: 0 for every joint, i.e. "
                         "the URDF zero: arm straight out, gripper closed)")
    ap.add_argument("--duration", type=float, default=120.0, help="seconds")
    ap.add_argument("--rate", type=float, default=50.0, help="command/record rate, Hz")
    ap.add_argument("--max-vel", type=float, default=0.5, help="peak joint speed, rad/s")
    ap.add_argument("--amplitude-scale", type=float, default=0.6,
                    help="fraction of each joint's usable half-range to use")
    ap.add_argument("--margin", type=float, default=0.15,
                    help="kept clear of each effective joint limit, rad")
    ap.add_argument("--base-freq", type=float, default=0.05, help="Fourier fundamental, Hz")
    ap.add_argument("--harmonics", type=int, default=3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--min-height", type=float, default=0.03,
                    help="lowest allowed link-origin height above the base frame, m")
    ap.add_argument("--abort-tracking", type=float, default=0.35,
                    help="stop if a joint lags its command by more than this, rad")
    ap.add_argument("--abort-current", type=float, default=1500.0,
                    help="stop if a servo draws more than this, mA")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    ap.add_argument("--dry-run", action="store_true",
                    help="plan and 'record' against an in-memory arm; touches no hardware")
    return ap.parse_args(argv)


def _floats(text: str, n: int, what: str) -> np.ndarray:
    vals = [float(v) for v in text.split(",")]
    if len(vals) != n:
        raise SystemExit(f"{what} needs {n} comma-separated values, got {len(vals)}")
    return np.asarray(vals)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)

    config: Dict[str, Any] = load_robot_config(args.config)
    names: List[str] = list(config["joint_names"])
    n = len(names)

    # -- calibration -----------------------------------------------------
    cal_path = Path(args.calibration).expanduser()
    calibration: Optional[RobotCalibration] = None
    if cal_path.exists():
        calibration = RobotCalibration.load(cal_path)
        if calibration.names != names:
            print(f"error: calibration joints {calibration.names} do not match config "
                  f"joints {names}", file=sys.stderr)
            return 2
    elif not args.dry_run:
        print(f"error: no calibration at {cal_path}. Calibrate first "
              "(soarm-dashboard-calibration), or pass --calibration.", file=sys.stderr)
        return 2
    if calibration is not None and not calibration.validated and not args.dry_run:
        if not args.allow_unvalidated:
            print(f"error: {cal_path} is not validated — its direction signs have not been "
                  "confirmed on the arm, and a wrong sign flips that joint's gravity "
                  "torque in the fit. Validate it, or pass --allow-unvalidated.",
                  file=sys.stderr)
            return 2
        print("warning: calibration NOT validated (--allow-unvalidated)")
    signs = (
        list(calibration.direction_signs)
        if calibration is not None
        else list(config["hardware"]["direction_signs"])
    )

    # -- plan ------------------------------------------------------------
    from ..robot.servo import ServoRobot

    # Constructing does not connect: this is only for the effective limits,
    # so the plan and the per-write clamp agree on what "in range" means.
    lo, hi = ServoRobot("", config, calibration=calibration).effective_joint_limits()

    if args.joints:
        wanted = [s.strip() for s in args.joints.split(",") if s.strip()]
        unknown = [w for w in wanted if w not in names]
        if unknown:
            raise SystemExit(f"unknown joints {unknown}; config has {names}")
        active = np.array([nm in wanted for nm in names])
    else:
        gi = config.get("gripper", {}).get("joint_index", n - 1)
        active = np.array([i != gi for i in range(n)])
    center = _floats(args.center, n, "--center") if args.center else np.zeros(n)
    spec = ExcitationSpec(
        duration_s=args.duration,
        rate_hz=args.rate,
        base_freq_hz=args.base_freq,
        n_harmonics=args.harmonics,
        max_vel_rad_s=args.max_vel,
        margin_rad=args.margin,
        amplitude_scale=args.amplitude_scale,
        seed=args.seed,
    )
    # Inactive joints only need to sit inside their limits, not the margin.
    center = np.where(active, center, np.clip(center, lo, hi))
    try:
        t, q_plan = fourier_excitation(lo, hi, spec, active=active, center=center)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    urdf = Path(args.urdf).expanduser()
    if not urdf.exists():
        print(f"error: URDF not found at {urdf}; the clearance check needs it (--urdf)",
              file=sys.stderr)
        return 2
    model = GravityModel.from_urdf(urdf, names)
    stride = max(1, int(round(args.rate / 10)))
    lowest = min((model.lowest_point(q) for q in q_plan[::stride]), key=lambda p: p[1])

    dq_plan = np.gradient(q_plan, t, axis=0)
    print(f"config         : {args.config} ({n} joints)")
    print(f"calibration    : {cal_path if calibration else '(none — config signs)'}")
    print(f"exciting       : {', '.join(nm for nm, a in zip(names, active) if a)}")
    print(f"duration       : {spec.duration_s:.0f} s at {spec.rate_hz:.0f} Hz "
          f"({t.size} commands)")
    for j, nm in enumerate(names):
        print(f"  {nm:14s} [{q_plan[:, j].min():+.2f}, {q_plan[:, j].max():+.2f}] rad"
              f"   peak {np.abs(dq_plan[:, j]).max():.2f} rad/s"
              f"   limits [{lo[j]:+.2f}, {hi[j]:+.2f}]")
    print(f"lowest point   : {lowest[0]} at z = {lowest[1]:.3f} m")
    if lowest[1] < args.min_height:
        print(f"error: the plan takes {lowest[0]} to z = {lowest[1]:.3f} m, below "
              f"--min-height {args.min_height} m. Raise --center, or lower "
              "--amplitude-scale.", file=sys.stderr)
        return 2

    out = Path(args.out).expanduser() if args.out else Path("identification_runs") / (
        f"{args.arm_id or ('dry-run' if args.dry_run else 'arm')}-"
        f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    )
    meta: Dict[str, Any] = {
        "arm_id": args.arm_id or (calibration.arm_id if calibration else "unknown"),
        "simulated": bool(args.dry_run),
        "calibration": str(cal_path) if calibration else None,
        "calibration_validated": bool(calibration and calibration.validated),
        "direction_signs": signs,
        "active_joints": [nm for nm, a in zip(names, active) if a],
        "excitation": {
            "duration_s": spec.duration_s,
            "base_freq_hz": spec.base_freq_hz,
            "n_harmonics": spec.n_harmonics,
            "max_vel_rad_s": spec.max_vel_rad_s,
            "amplitude_scale": spec.amplitude_scale,
            "margin_rad": spec.margin_rad,
            "seed": spec.seed,
            "center": center.tolist(),
        },
        "recorded_at": datetime.now(timezone.utc).isoformat(),
    }

    # -- run -------------------------------------------------------------
    if args.dry_run:
        from ..robot.null import NullRobot

        robot = NullRobot(config)
        robot.connect()
        clock = _SimClock()
        kwargs: Dict[str, Any] = {"clock": clock, "sleep": clock.sleep}
        print("\nDRY RUN — no hardware. Currents are recorded as zero: the log "
              "exercises the format, not the identification.")
    else:
        from ..calibration.sweep_cli import _resolve_device

        try:
            device = _resolve_device(args.device)
        except RuntimeError as exc:
            print(f"\nerror: {exc}", file=sys.stderr)
            return 2
        if not args.yes:
            print()
            print("The arm will move slowly through every excited joint's range for")
            print(f"{spec.duration_s:.0f} s. Clear the workspace and be ready to cut power.")
            if not input("Proceed? [y/N] ").strip().lower().startswith("y"):
                print("aborted.")
                return 1
        robot = ServoRobot(device, config, calibration=calibration, max_step_rad=0.2)
        robot.connect()
        meta["device"] = device
        kwargs = {}

    try:
        log = record_excitation(
            robot,
            q_plan,
            spec.rate_hz,
            joint_names=names,
            direction_signs=signs,
            abort_tracking_rad=args.abort_tracking,
            abort_current_mA=args.abort_current,
            meta=meta,
            **kwargs,
        )
    except ExcitationAborted as exc:
        print(f"\nABORTED: {exc}", file=sys.stderr)
        if exc.log is not None:
            where = exc.log.resampled(spec.rate_hz).save(out.with_name(out.name + "-partial"))
            print(f"partial log written to {where}", file=sys.stderr)
        return 3
    finally:
        robot.disconnect()

    where = log.resampled(spec.rate_hz).save(out)
    print(f"\nrecorded {log.t.size} samples over {log.t[-1]:.1f} s "
          f"(mean {log.rate_hz:.1f} Hz, {log.meta.get('missed_ticks', 0)} missed bus ticks)")
    print(f"written to     : {where}")
    print("next           : identify it with figaroh-examples/examples/so101/identification.py "
          f"--data-dir {where}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
