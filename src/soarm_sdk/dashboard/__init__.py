"""Extensible Viser dashboard for soarm_sdk.

Requires the ``viser`` extra: ``pip install soarm-sdk[viser]``. Not
imported by ``soarm_sdk`` itself — opt in explicitly so the base install
stays free of viser/yourdfpy/trimesh.

::

    from soarm_sdk.dashboard import DashboardApp
    from soarm_sdk.dashboard.panels import setup

    app = DashboardApp(title="Hardware Setup")
    for panel in setup.build_all(fk_update_fn=app.fk_update):
        app.register(panel)
    app.run()
"""

from __future__ import annotations

from .app import DashboardApp, Panel
from .context import DashboardContext, JointState

__all__ = ["DashboardApp", "Panel", "DashboardContext", "JointState"]
