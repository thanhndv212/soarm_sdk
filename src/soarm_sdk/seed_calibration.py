"""Deprecated import path — use :mod:`soarm_sdk.calibration.seed`.

Also still runnable as ``python -m soarm_sdk.seed_calibration``, which the
``soarm-seed-calibration`` console script now replaces. Importing this
module emits a :class:`DeprecationWarning`; it is scheduled for removal in
**0.3.0**.
"""

from __future__ import annotations

import sys
import warnings

from .calibration.seed import *  # noqa: F401,F403
from .calibration.seed import (  # noqa: F401
    main,
    SO101_URDF_LIMITS,
    DEFAULT_OUT,
)

warnings.warn(
    "soarm_sdk.seed_calibration is deprecated; import from "
    "soarm_sdk.calibration.seed, or run the soarm-seed-calibration console "
    "script instead of `python -m soarm_sdk.seed_calibration`. "
    "This shim will be removed in 0.3.0.",
    DeprecationWarning,
    stacklevel=2,
)

if __name__ == "__main__":
    sys.exit(main())
