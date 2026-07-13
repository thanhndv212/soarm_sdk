#!/usr/bin/env python
"""GUI for setting up a fresh SO-ARM100: discover its port, home it, configure it.

Just the three "bring a fresh arm online" panels (Start Up, Homing Wizard,
Reconfigure), built on ``soarm_sdk.dashboard`` so new applications can be
registered the same way instead of forking this script — see
``viser_dashboard.py`` for the larger day-to-day operation dashboard
(monitoring, manual commands, PID tuning, recording).

Launch
------
    python examples/setup_dashboard.py \\
        [--device /dev/ttyXXX] [--baud 1000000] \\
        [--port 8080] [--urdf PATH] [--interval-ms 200]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Path setup — allows running without installing the package.
# ---------------------------------------------------------------------------
_src_root = Path(__file__).resolve().parents[1] / "src"
if _src_root.is_dir() and str(_src_root) not in sys.path:
    sys.path.insert(0, str(_src_root))

from soarm_sdk import get_available_ports  # noqa: E402
from soarm_sdk.dashboard import DashboardApp  # noqa: E402
from soarm_sdk.dashboard.panels import setup  # noqa: E402

_DEFAULT_URDF = (
    Path(__file__).resolve().parents[2] / "SO-ARM100" / "Simulation" / "SO100" / "so100.urdf"
)


def main() -> None:
    parser = argparse.ArgumentParser(description="soarm_sdk hardware setup dashboard")
    parser.add_argument("--device", default="", help="Serial device path")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--port", type=int, default=8080, help="Viser HTTP port")
    parser.add_argument(
        "--urdf", type=Path, default=_DEFAULT_URDF, help="URDF file for 3-D visualisation"
    )
    parser.add_argument("--interval-ms", type=int, default=200)
    args = parser.parse_args()

    device = args.device
    if not device:
        ports = get_available_ports()
        if ports:
            device = ports[0][0]
            print(f"[setup_dashboard] Auto-selected device: {device}")

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
