"""Viser dashboard for soarm_sdk: robot control + real-time 3-D visualisation.

Two entry points, sharing the same panel-building code:

- :func:`main` (``soarm-dashboard``) — the full operator dashboard: Start
  Up, Homing Wizard, Reconfigure, Command Panel, PID Tuning, Monitor,
  Recorder.
- :func:`main_setup` (``soarm-dashboard-setup``) — just the "bring a fresh
  arm online" panels (Start Up, Homing Wizard, Reconfigure), for initial
  hardware setup without the day-to-day operation tabs.

Built entirely on :mod:`soarm_sdk.dashboard` — add a new tab by writing a
``build_*_panel()`` function there rather than forking this module.

Launch
------
    soarm-dashboard [--device /dev/ttyXXX] [--baud 1000000] \\
        [--port 8080] [--urdf PATH] [--interval-ms 200]
    soarm-dashboard-setup [same flags]
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

from .. import get_available_ports
from ..dashboard import DashboardApp
from ..dashboard.panels import command, monitor, pid, recorder, setup

# A URDF is workspace-relative (a sibling SO-ARM100/ checkout), not shipped
# inside this installed package, so this default only resolves when running
# from within the soarm-ws workspace layout. Override with --urdf elsewhere.
_DEFAULT_URDF = (
    Path(__file__).resolve().parents[4] / "SO-ARM100" / "Simulation" / "SO100" / "so100.urdf"
)


def _build_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--device", default="", help="Serial device path")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--port", type=int, default=8080, help="Viser HTTP port")
    parser.add_argument(
        "--urdf", type=Path, default=_DEFAULT_URDF, help="URDF file for 3-D visualisation"
    )
    parser.add_argument("--interval-ms", type=int, default=200)
    return parser


def _resolve_device(args: argparse.Namespace, label: str) -> str:
    device = args.device
    if not device:
        ports = get_available_ports()
        if ports:
            device = ports[0][0]
            print(f"[{label}] Auto-selected device: {device}")
    return device


def main(argv: Optional[list] = None) -> None:
    """Full operator dashboard: setup + command + PID + monitor + recorder."""
    parser = _build_parser("Viser dashboard for soarm_sdk")
    args = parser.parse_args(argv)
    device = _resolve_device(args, "soarm-dashboard")

    app = DashboardApp(
        title="soarm_sdk Dashboard",
        port=args.port,
        device=device,
        baud=args.baud,
        interval_ms=args.interval_ms,
        urdf_path=args.urdf,
    )

    for panel in setup.build_all(fk_update_fn=app.fk_update):
        app.register(panel)
    app.register(command.build_command_panel())
    app.register(pid.build_pid_panel())
    app.register(monitor.build_monitor_panel())
    app.register(recorder.build_recorder_panel())

    app.run()


def main_setup(argv: Optional[list] = None) -> None:
    """Hardware-setup-only dashboard: Start Up, Homing Wizard, Reconfigure."""
    parser = _build_parser("soarm_sdk hardware setup dashboard")
    args = parser.parse_args(argv)
    device = _resolve_device(args, "soarm-dashboard-setup")

    app = DashboardApp(
        title="soarm_sdk — Hardware Setup",
        port=args.port,
        device=device,
        baud=args.baud,
        interval_ms=args.interval_ms,
        urdf_path=args.urdf,
    )
    for panel in setup.build_all(fk_update_fn=app.fk_update):
        app.register(panel)
    app.run()


if __name__ == "__main__":
    main()
