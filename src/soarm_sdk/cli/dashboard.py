"""Viser dashboard for soarm_sdk: robot control + real-time 3-D visualisation.

Two console scripts, one per :class:`~soarm_sdk.dashboard.app.DashboardProfile`
in :data:`PROFILES`, sharing the same panel-building code:

- ``soarm-dashboard-setup`` (:func:`main_setup`, profile ``"setup"``) — every
  tab: Start Up, Homing Wizard, Reconfigure, Command Panel, PID Tuning,
  Monitor, Recorder. The one dashboard for bringing an arm up and running
  it day to day.
- ``soarm-dashboard-calibration`` (:func:`main_calibration`, profile
  ``"calibration"``) — the guided URDF-frame calibration and acceptance
  workflow.

Every dashboard has its own named script; there is deliberately no bare
"just launch something" entry point, so a profile is always the one you
asked for, not a default you have to already know about.

Built entirely on :mod:`soarm_sdk.dashboard` — add a new tab by writing a
``build_*_panel()`` function there, or a new profile by adding an entry to
``PROFILES`` plus a thin ``main_*``, rather than forking this module.

Launch
------
    soarm-dashboard-setup [--device /dev/ttyXXX] [--baud 1000000] \\
        [--port 8080] [--urdf PATH] [--interval-ms 200]
    soarm-dashboard-calibration [same flags]
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Optional

from .. import get_available_ports
from ..dashboard import DashboardApp
from ..dashboard.app import DashboardProfile
from ..dashboard.panels import calibration, command, monitor, pid, recorder, setup

# A URDF is workspace-relative (a sibling SO-ARM100/ checkout), not shipped
# inside this installed package, so this default only resolves when running
# from within the soarm-ws workspace layout. Override with --urdf elsewhere.
_DEFAULT_URDF = (
    Path(__file__).resolve().parents[4]
    / "SO-ARM100"
    / "Simulation"
    / "SO101"
    / "so101_new_calib.urdf"
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
    parser.add_argument(
        "--calibration",
        type=Path,
        default=None,
        help="Tick-to-URDF-frame calibration for the 3-D view "
        "(default: ~/.soarm_sdk/calibration.json). Without one the view "
        "assumes tick 2048 is zero on every joint and will not match the arm.",
    )
    parser.add_argument(
        "--stream",
        action="store_true",
        help="Hold the serial port open with one ServoHardwareInterface and "
        "consume its telemetry stream, instead of reopening the port every "
        "poll. Temperature and current then arrive every tick rather than "
        "every fifth poll, and the periodic 7 ms stall from twelve "
        "per-servo health reads goes away. Opt-in while it beds in.",
    )
    return parser


def _resolve_device(args: argparse.Namespace, label: str) -> str:
    device = args.device
    if not device:
        ports = get_available_ports()
        if ports:
            device = ports[0][0]
            print(f"[{label}] Auto-selected device: {device}")
    return device


def _register_setup(app: DashboardApp) -> None:
    for panel in setup.build_all(fk_update_fn=app.fk_update):
        app.register(panel)
    app.register(command.build_command_panel())
    app.register(pid.build_pid_panel())
    app.register(monitor.build_monitor_panel())
    app.register(recorder.build_recorder_panel())


def _register_calibration(app: DashboardApp) -> None:
    app.register(setup.build_startup_panel())
    for panel in calibration.build_calibration_panels(
        fk_update_fn=app.fk_update, ghost_fn=app.show_ghost
    ):
        app.register(panel)


#: One entry per console script above. A new profile — say, a stripped-down
#: field-ops dashboard — is a new key here plus a new thin ``main_*``, not a
#: new hand-assembled panel list.
PROFILES: Dict[str, DashboardProfile] = {
    "setup": DashboardProfile(
        name="setup",
        title="soarm_sdk Dashboard",
        register=_register_setup,
        description="soarm_sdk dashboard: setup + command + PID + monitor + recorder",
    ),
    "calibration": DashboardProfile(
        name="calibration",
        title="soarm_sdk — Calibration",
        register=_register_calibration,
        description="soarm_sdk calibration dashboard",
    ),
}


def _launch(profile: DashboardProfile, argv: Optional[list], log_label: str) -> None:
    parser = _build_parser(profile.description)
    args = parser.parse_args(argv)
    device = _resolve_device(args, log_label)

    app = DashboardApp(
        title=profile.title,
        port=args.port,
        device=device,
        baud=args.baud,
        interval_ms=args.interval_ms,
        urdf_path=args.urdf,
        use_stream=args.stream,
        calibration_path=args.calibration,
    )
    profile.register(app)
    app.run()


def main_setup(argv: Optional[list] = None) -> None:
    """Every tab: setup + command + PID + monitor + recorder."""
    _launch(PROFILES["setup"], argv, "soarm-dashboard-setup")


def main_calibration(argv: Optional[list] = None) -> None:
    """Dedicated Viser workflow for ROM, zero, provenance, and acceptance."""
    _launch(PROFILES["calibration"], argv, "soarm-dashboard-calibration")


if __name__ == "__main__":
    main_setup()
