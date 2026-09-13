"""Built-in dashboard panels, grouped by concern.

``setup`` — bring a fresh arm online (Start Up, Homing Wizard, Reconfigure).
``command``, ``pid``, ``monitor``, ``recorder`` — ongoing operation, each a
single ``build_*_panel() -> Panel``.
``calibration`` — reconcile the 3-D mirror with the real arm (re-zero from a
reference pose, nudge one joint's zero, flag poses the URDF cannot represent).
"""
