"""Console-script entry points for soarm_sdk.

Installing the package (``pip install soarm-sdk``) provides these on
``$PATH`` directly — see ``[project.scripts]`` in ``pyproject.toml``:

- ``soarm-calibrate`` — servo EEPROM calibration CLI + interactive UI.
- ``soarm-dashboard`` / ``soarm-dashboard-setup`` — the Viser dashboard.
- ``soarm-seed-calibration`` — seed a URDF-frame calibration offline.

Each also has a matching thin script under ``examples/`` for anyone
running from a checkout without installing the package.
"""

from __future__ import annotations
