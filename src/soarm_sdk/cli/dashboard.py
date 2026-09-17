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

**This module is also the shared CLI layer for dashboards built outside
this package** — ``soarm_tamp``'s plan-and-run dashboard is the existing
example (see ``soarm_tamp.dashboard.__main__``). Its
:class:`~soarm_sdk.dashboard.app.DashboardProfile` already has the right
shape; :func:`launch` (or its two halves, :func:`build_parser` and
:func:`resolve_use_stream`) is what turns that profile into a full CLI
without a second, drifting copy of the device/baud/urdf/stream/rerun
argument definitions and the auto-device-selection logic. Before this was
factored out, ``soarm_tamp``'s dashboard hand-rolled its own parser and
quietly fell behind — it had no ``--rerun`` flag until one was ported over
by hand.

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

__all__ = [
    "PROFILES",
    "build_parser",
    "resolve_device",
    "resolve_use_stream",
    "launch",
    "main_setup",
    "main_calibration",
]

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


def build_parser(
    description: str,
    *,
    default_urdf: Optional[Path] = None,
    default_use_stream: bool = False,
) -> argparse.ArgumentParser:
    """The argument set every soarm_sdk-based dashboard shares.

    *default_urdf* lets a caller outside this package (a different default
    model path than SO-ARM100's) still use this parser rather than forking
    it. *default_use_stream* picks which polarity of the streaming flag to
    expose: ``False`` adds an opt-in ``--stream`` (this package's own
    dashboards, where the legacy poll loop is still the safer default while
    streaming beds in); ``True`` adds an opt-out ``--no-stream`` (a caller
    that already trusts streaming and wants it on by default, e.g.
    ``soarm_tamp``, which needs the persistent interface for TCP/pick-place
    execution anyway).
    """
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--device", default="", help="Serial device path")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--port", type=int, default=8080, help="Viser HTTP port")
    parser.add_argument(
        "--urdf", type=Path, default=default_urdf, help="URDF file for 3-D visualisation"
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
    if default_use_stream:
        parser.add_argument(
            "--no-stream",
            action="store_true",
            help="Use the legacy per-iteration poll instead of holding the "
            "port open with one ServoHardwareInterface.",
        )
    else:
        parser.add_argument(
            "--stream",
            action="store_true",
            help="Hold the serial port open with one ServoHardwareInterface and "
            "consume its telemetry stream, instead of reopening the port every "
            "poll. Temperature and current then arrive every tick rather than "
            "every fifth poll, and the periodic 7 ms stall from twelve "
            "per-servo health reads goes away. Opt-in while it beds in.",
        )
    parser.add_argument(
        "--rerun",
        action="store_true",
        help="Also feed the streaming interface's telemetry to a spawned "
        "Rerun viewer (soarm_sdk.monitoring.blueprint's by-servo/by-channel "
        "layouts) — every control here (torque, joint commands, PID "
        "tuning, scan) stays on this one connection; Rerun just gets a "
        "second, independent subscription to its telemetry for better "
        "multi-channel charts. Overrides the streaming flag above (a "
        "second subscriber needs a persistent interface to subscribe to). "
        "Needs the `telemetry` extra: pip install soarm-sdk[telemetry].",
    )
    return parser


def resolve_device(args: argparse.Namespace, label: str) -> str:
    device = args.device
    if not device:
        ports = get_available_ports()
        if ports:
            device = ports[0][0]
            print(f"[{label}] Auto-selected device: {device}")
    return device


def resolve_use_stream(
    args: argparse.Namespace, *, default_use_stream: bool, log_label: str
) -> bool:
    """Combine the streaming flag with ``--rerun``'s hard requirement on it.

    Reads whichever flag :func:`build_parser` actually added
    (``args.no_stream`` when *default_use_stream* is ``True``, else
    ``args.stream``) — never both, since only one is ever defined on
    *args* for a given *default_use_stream*.
    """
    use_stream = (not args.no_stream) if default_use_stream else args.stream
    if args.rerun and not use_stream:
        flag = "--no-stream" if default_use_stream else "requires --stream"
        print(f"[{log_label}] --rerun overrides {flag}; streaming stays on.")
        use_stream = True
    return use_stream


def launch(
    profile: DashboardProfile,
    argv: Optional[list],
    log_label: str,
    *,
    default_urdf: Optional[Path] = None,
    default_use_stream: bool = False,
) -> None:
    """Parse args for *profile* and run it. The shared body behind every
    ``main_*`` in this module, and the intended entry point for a dashboard
    defined outside this package — see this module's own docstring.
    """
    parser = build_parser(
        profile.description, default_urdf=default_urdf, default_use_stream=default_use_stream
    )
    args = parser.parse_args(argv)
    device = resolve_device(args, log_label)
    use_stream = resolve_use_stream(
        args, default_use_stream=default_use_stream, log_label=log_label
    )

    app = DashboardApp(
        title=profile.title,
        port=args.port,
        device=device,
        baud=args.baud,
        interval_ms=args.interval_ms,
        urdf_path=args.urdf,
        use_stream=use_stream,
        rerun=args.rerun,
        calibration_path=args.calibration,
    )
    profile.register(app)
    app.run()


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


def main_setup(argv: Optional[list] = None) -> None:
    """Every tab: setup + command + PID + monitor + recorder."""
    launch(PROFILES["setup"], argv, "soarm-dashboard-setup", default_urdf=_DEFAULT_URDF)


def main_calibration(argv: Optional[list] = None) -> None:
    """Dedicated Viser workflow for ROM, zero, provenance, and acceptance."""
    launch(
        PROFILES["calibration"], argv, "soarm-dashboard-calibration", default_urdf=_DEFAULT_URDF
    )


if __name__ == "__main__":
    main_setup()
