"""Dynamic identification support: excitation, recording, gravity models.

The identification itself (regressor, base parameters, least squares) is
done by FIGAROH, in its own environment — see ``examples/so101/`` in
figaroh-examples. This package covers the two ends that have to live next
to the hardware:

* **Before:** plan a slow excitation (:func:`fourier_excitation`), stream it
  to the arm and record joint angles, velocities and servo current in the
  URDF joint frame (:func:`record_excitation`), and write it in the format
  the identification side reads (:class:`IdentificationLog`). The
  ``soarm-identify-record`` console script does all three.
* **After:** load the identified parameters back
  (:class:`IdentifiedDynamics`) and evaluate gravity and friction torque —
  or the servo current they imply — at any configuration, for gravity
  compensation or admittance control. :class:`GravityModel` is a
  dependency-free (numpy only) equivalent of pinocchio's
  ``computeGeneralizedGravity`` for a URDF.
"""

from __future__ import annotations

from .excitation import ExcitationSpec, fourier_excitation
from .gravity import GravityModel
from .identified import IdentifiedDynamics
from .log import IdentificationLog
from .record import ExcitationAborted, record_excitation

__all__ = [
    "ExcitationSpec",
    "fourier_excitation",
    "GravityModel",
    "IdentifiedDynamics",
    "IdentificationLog",
    "ExcitationAborted",
    "record_excitation",
]
