"""soarm-monitor: live Rerun telemetry viewer for the SO-101 servo bus.

Read-only by construction (``torque_on_start=False``, and this module never
calls ``set_robot_joint_positions``) — it only ever subscribes to the bus
thread's own telemetry stream (see
:mod:`soarm_sdk.robot.telemetry`/:mod:`soarm_sdk.robot.telemetry_sinks`),
the same object the viser dashboard's ``--stream`` mode and a
:class:`~soarm_sdk.robot.telemetry_sinks.TelemetryRecorder` already consume.

The serial port is still exclusive, though: this holds it open for as long
as it runs, the same as the dashboard's ``--stream`` mode or a teleop
session would. Run it *instead of* a second dashboard/teleop process on the
same device, not alongside one — two independent openers of the same tty
corrupt each other's reads rather than cleanly failing to open.

Launch
------
    soarm-monitor [--device /dev/ttyXXX] [--baud 1000000] \\
        [--joint-ids 1-6] [--calibration PATH] [--serve-web]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

from .. import get_available_ports
from ..dashboard.context import DashboardContext
from ..dashboard.fk import SOARM100_JOINT_NAMES, load_calibration
from ..monitoring.blueprint import build_monitor_blueprint
from ..robot.hardware import ServoHardwareInterface
from ..robot.telemetry_sinks import RerunSink, TelemetryRecorder

__all__ = ["main"]


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Live, read-only Rerun telemetry viewer for the SO-101 servo bus."
    )
    parser.add_argument("--device", default="", help="Serial device path (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument(
        "--joint-ids", default="1-6", help="e.g. '1-6' or '1,3,5' (default: all six)"
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=None,
        help="Calibration file for accurate position_rad (default: "
        "~/.soarm_sdk/calibration.json). Without one, position_rad assumes "
        "tick 2048 is zero on every joint.",
    )
    parser.add_argument("--app-id", default="soarm_monitor")
    parser.add_argument(
        "--serve-web",
        action="store_true",
        help="Serve the viewer over HTTP instead of spawning a native window "
        "(useful over SSH / a headless bench machine).",
    )
    parser.add_argument("--web-port", type=int, default=9090)
    return parser


def _resolve_device(device: str) -> str:
    if device:
        return device
    ports = get_available_ports()
    if not ports:
        raise SystemExit(
            "[soarm-monitor] No serial device found. Pass --device explicitly."
        )
    resolved = ports[0][0]
    print(f"[soarm-monitor] Auto-selected device: {resolved}")
    return resolved


def main(argv: Optional[list] = None) -> None:
    import rerun as rr

    args = _build_parser().parse_args(argv)
    device = _resolve_device(args.device)
    joint_ids = DashboardContext.parse_ids(args.joint_ids)
    joint_names = [SOARM100_JOINT_NAMES[i - 1] for i in joint_ids]

    calibration = load_calibration(args.calibration)
    if calibration is not None and len(calibration.joints) != len(joint_ids):
        print(
            f"[soarm-monitor] calibration has {len(calibration.joints)} joint(s), "
            f"expected {len(joint_ids)} for --joint-ids {args.joint_ids!r} — "
            "ignoring it (position_rad will assume tick 2048 is zero)."
        )
        calibration = None

    blueprint = build_monitor_blueprint(joint_names)
    if args.serve_web:
        rr.init(args.app_id, default_blueprint=blueprint)
        rr.serve_web(web_port=args.web_port, default_blueprint=blueprint)
    else:
        rr.init(args.app_id, spawn=True, default_blueprint=blueprint)

    iface = ServoHardwareInterface(
        port=device,
        baud=args.baud,
        joint_ids=joint_ids,
        calibration=calibration,
        # A monitor must never energise the arm just by watching it.
        torque_on_start=False,
    )
    iface.start()
    recorder = TelemetryRecorder(iface, [RerunSink(joint_names=joint_names)])
    recorder.start()

    print(
        f"[soarm-monitor] streaming {len(joint_ids)} joint(s) from {device} "
        "— Ctrl+C to stop."
    )
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        iface.stop()
        print("[soarm-monitor] stopped.")


if __name__ == "__main__":
    sys.exit(main())
