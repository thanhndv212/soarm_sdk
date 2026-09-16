"""Extensible Viser dashboard shell for soarm_sdk applications.

The point of this module: a new "application" (a new tab) should be a
:class:`Panel` registered with :class:`DashboardApp`, not a new fork of a
2000-line script. Every panel shares one :class:`DashboardContext`, so
connection handling / bus access / background polling is written once.
"""

from __future__ import annotations

import logging
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
from .fk import (
    DEFAULT_CALIBRATION_PATH,
    SOARM100_IDS,
    load_calibration,
    load_ghost_meshes,
    load_urdf_meshes,
    pose_meshes,
    update_fk,
)

logger = logging.getLogger(__name__)

__all__ = ["Panel", "DashboardProfile", "DashboardApp"]


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


@dataclass
class DashboardProfile:
    """A named, reusable set of panels — what a console script (or a future
    ``--profile`` flag) selects, in place of a hand-assembled panel list.

    ``register`` runs once, right after the :class:`DashboardApp` exists —
    not before — so it can hand panels ``app.fk_update`` / ``app.show_ghost``,
    the same callbacks a caller assembling panels by hand would pass.
    """

    name: str
    title: str
    register: Callable[["DashboardApp"], None]
    description: str = ""


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
        rerun: bool = False,
        calibration_path: Optional[Path] = None,
    ) -> None:
        self.server = viser.ViserServer(port=port)
        # Neither viser layout supports free drag-resize (checked its
        # frontend source: FloatingPanel.tsx / SidebarPanel.tsx both take a
        # fixed width, no resize handle) — only the discrete control_width
        # presets. "floating" (viser's own default) at least lets the panel
        # be dragged to a better spot on screen; "large" gives labels like
        # "Steady-state error max (ticks)" room instead of clipping.
        self.server.gui.configure_theme(control_layout="floating", control_width="large")
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

        # A translucent target the operator can aim the real arm at, hidden
        # until a panel asks for it. Registered here rather than in the panel
        # so the scene owns one copy: panels come and go, the scene graph
        # does not, and a second /ghost would silently replace the first.
        self._ghost_handles = load_ghost_meshes(self.server, self.urdf)

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
            rerun=rerun,
        )
        # The 3-D view reads the calibration through the context rather than
        # off this object, so a panel can correct a bad zero and have the
        # mirror follow immediately — the whole point of the Calibration tab.
        self.ctx.calibration = self.calibration
        # The *resolved* path, never the argument. Callers that take the
        # default pass None, and storing that made calibration_drift() report
        # "nothing to compare" forever — so every consistency check downstream
        # silently passed, which is the failure it exists to catch.
        self.ctx.calibration_path = (
            Path(calibration_path)
            if calibration_path is not None
            else DEFAULT_CALIBRATION_PATH
        )
        self.ctx.urdf = self.urdf

        self._panels: List[Panel] = []
        self._fk_error_logged = False

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
        if self.urdf is None or not self._mesh_handles:
            return
        try:
            update_fk(
                self.urdf,
                positions,
                self._mesh_handles,
                joint_ids=list(self.ctx.joint_ids),
                calibration=self.ctx.calibration,
            )
        except Exception:
            # Swallowed per frame, but said once. This ran ~10 Hz inside a
            # bare `except: pass`, so a URDF that could not be posed looked
            # exactly like an arm that was not moving — the mirror simply
            # froze, with nothing anywhere to say why.
            if not self._fk_error_logged:
                self._fk_error_logged = True
                logger.exception("[soarm_sdk.dashboard] FK update failed")

    def show_ghost(self, cfg: Optional[Dict[str, float]]) -> None:
        """Pose and reveal the reference ghost, or hide it when *cfg* is None.

        Safe to pass around as a callback; a no-op when no URDF was loaded.
        """
        if not self._ghost_handles:
            return
        if cfg:
            pose_meshes(self.urdf, cfg, self._ghost_handles)
        for handle in self._ghost_handles.values():
            handle.visible = bool(cfg)

    def fk_update_ghost(self, positions: Dict[int, int]) -> None:
        """Pose the ghost from servo ticks, through the same calibrated FK
        :meth:`fk_update` uses for the live mirror — the tick-driven twin of
        :meth:`show_ghost`, which only takes an explicit joint-radians pose.

        For animating a planned path onto the ghost while the live mirror
        keeps showing wherever the real arm actually is: a manifest player
        and the live-poll loop used to both drive the *same* mesh through
        the same ``fk_update``, so playing a preview fought the live arm's
        own position for the same pixels every tick. Two independent mesh
        sets — this one, plus whatever still calls :meth:`fk_update` — removes
        the race instead of arbitrating it.

        No-op if no URDF was loaded. Reveals the ghost on first use; call
        :meth:`show_ghost` with ``None`` to hide it again.
        """
        if self.urdf is None or not self._ghost_handles:
            return
        try:
            update_fk(
                self.urdf,
                positions,
                self._ghost_handles,
                joint_ids=list(self.ctx.joint_ids),
                calibration=self.ctx.calibration,
            )
        except Exception:
            logger.exception("[soarm_sdk.dashboard] ghost FK update failed")
            return
        for handle in self._ghost_handles.values():
            handle.visible = True

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
                # Gated on ctx.polling, not just "urdf loaded": whoever calls
                # ctx.stop_polling() (capture_start, ExecutionJob) is taking
                # over the mirror mesh for the duration — ExecutionJob drives
                # it itself from the live command trace. Without this gate,
                # this loop kept pushing ctx.state.positions regardless, which
                # a stopped poll thread leaves frozen at whatever it was when
                # polling stopped. Every ~100ms that stale pose overwrote
                # whatever the execution follower had just drawn, and the
                # mirror visibly snapped back and forth between the two.
                if self.urdf is not None and self.ctx.polling:
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
