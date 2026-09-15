"""Console-script entry points for soarm_sdk.

Installing the package (``pip install soarm-sdk``) provides these on
``$PATH`` directly — see ``[project.scripts]`` in ``pyproject.toml``:

- ``soarm-reconfigure`` — servo EEPROM configuration CLI + interactive UI
  (IDs, angle limits, speed, torque, baud). Not the tick<->URDF-frame
  calibration below, despite the similar name this module used to have.
- ``soarm-dashboard-setup`` — every Viser dashboard tab: setup, command,
  PID, monitor, recorder.
- ``soarm-dashboard-calibration`` — the guided, on-hardware tick<->URDF-frame
  calibration workflow.

Each also has a matching thin script under ``examples/`` for anyone
running from a checkout without installing the package.
"""

from __future__ import annotations
