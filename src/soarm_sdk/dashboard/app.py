"""Extensible Viser dashboard shell for soarm_sdk applications.

The point of this module: a new "application" (a new tab) should be a
:class:`Panel` registered with :class:`DashboardApp`, not a new fork of a
2000-line script. Every panel shares one :class:`DashboardContext`, so
connection handling / bus access / background polling is written once.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional

try:
    import viser
except ImportError as exc:
    raise ImportError(
        "viser is required for soarm_sdk.dashboard. "
        "Install with: pip install soarm-sdk[viser]"
    ) from exc

from .context import DashboardContext
from .fk import SOARM100_IDS, load_calibration, load_urdf_meshes, update_fk

__all__ = ["Panel", "DashboardApp"]


@dataclass
class Panel:
    """A single dashboard tab: a name plus a function that builds its GUI.

    ``build`` is called once, at :meth:`DashboardApp.run` time, with the
    live Viser server and the shared :class:`DashboardContext`.

    ``on_tick``, if given, is called from :meth:`DashboardApp.run`'s main
    loop (~10 Hz) with the shared context — for panels that display live
    polled state (charts, telemetry text) and need to refresh it on the
    main thread, same as the FK refresh already does. Panels that only
    react to button clicks don't need it.
    """

    name: str
    build: Callable[["viser.ViserServer", DashboardContext], None]
    on_tick: Optional[Callable[[DashboardContext], None]] = None


class DashboardApp:
    """Viser dashboard shell: server + shared sidebar/context + panel registry.

    Applications register :class:`Panel` instances; each becomes a tab.

    Example
    -------
    ::

        from soarm_sdk.dashboard import DashboardApp
        from soarm_sdk.dashboard.panels import setup

        app = DashboardApp(title="Hardware Setup")
        for panel in setup.build_all(fk_update_fn=app.fk_update):
            app.register(panel)
        app.run()
    """

    def __init__(
        self,
        *,
        title: str = "soarm_sdk Dashboard",
        port: int = 8080,
        device: str = "",
        baud: int = 1_000_000,
        interval_ms: int = 200,
        joint_ids: Optional[List[int]] = None,
        urdf_path: Optional[Path] = None,
        use_stream: bool = False,
        calibration_path: Optional[Path] = None,
    ) -> None:
        self.server = viser.ViserServer(port=port)
        self.server.scene.world_axes.visible = True
        self.port = port

        self.urdf, self._mesh_handles = (
            load_urdf_meshes(self.server, urdf_path)
            if urdf_path is not None
            else (None, {})
        )
        # Without this the 3-D view renders every joint against a nominal
        # tick-2048 zero, which no real arm has: the model and the robot then
        # disagree by tens of degrees and the view looks broken when it is
        # only uncalibrated.
        self.calibration = (
            load_calibration(calibration_path)
            if self.urdf is not None
            else None
        )

        self.server.gui.add_markdown(f"# {title}")
        self.server.gui.add_markdown("---")
        device_h = self.server.gui.add_text("Serial device", initial_value=device)
        baud_h = self.server.gui.add_number(
            "Baud rate",
            initial_value=float(baud),
            min=9600,
            max=4_000_000,
            step=1000,
        )
        interval_h = self.server.gui.add_number(
            "Poll interval (ms)",
            initial_value=float(interval_ms),
            min=50,
            max=2000,
            step=50,
        )
        conn_status_md = self.server.gui.add_markdown("*Not connected.*")

        self.ctx = DashboardContext(
            device_h=device_h,
            baud_h=baud_h,
            interval_h=interval_h,
            conn_status_md=conn_status_md,
            joint_ids=joint_ids if joint_ids is not None else list(SOARM100_IDS),
            use_stream=use_stream,
        )

        self._panels: List[Panel] = []

    def register(self, panel: Panel) -> "DashboardApp":
        """Register a panel; returns ``self`` so calls can be chained."""
        self._panels.append(panel)
        return self

    def fk_update(self, positions: Dict[int, int]) -> None:
        """Push *positions* (servo_id -> ticks) through FK to the 3-D scene.

        No-op if no URDF was loaded. Safe to pass around as a callback —
        panels that animate a simulated sweep (see the Homing Wizard panel)
        call this directly instead of waiting for the background refresh.
        """
        if self.urdf is not None and self._mesh_handles:
            try:
                update_fk(
                    self.urdf,
                    positions,
                    self._mesh_handles,
                    calibration=self.calibration,
                )
            except Exception:
                pass

    def run(self) -> None:
        """Build all registered panels as tabs, then block.

        While blocked, refreshes the 3-D scene from live polled state (if a
        URDF was loaded) so hardware moves are reflected without every panel
        needing its own refresh loop.
        """
        tab_group = self.server.gui.add_tab_group()
        for panel in self._panels:
            with tab_group.add_tab(panel.name):
                panel.build(self.server, self.ctx)

        print(
            f"[soarm_sdk.dashboard] Open http://localhost:{self.port} in your browser."
        )

        try:
            while True:
                if self.urdf is not None:
                    with self.ctx.lock:
                        positions = dict(self.ctx.state.positions)
                    if positions:
                        self.fk_update(positions)
                for panel in self._panels:
                    if panel.on_tick is not None:
                        panel.on_tick(self.ctx)
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.ctx.stop_polling()
