"""URDF loading and forward kinematics, with no dependency on any viewer.

Split out of ``dashboard/fk.py``, which mixed this with Viser scene
registration — a planner or a headless test has no use for a Viser
server, but does need "load a URDF" and "get link transforms for this
joint config" just as much as the dashboard does.

:mod:`soarm_sdk.dashboard.fk` now calls into this module for the math and
only owns pushing the resulting transforms onto Viser mesh handles.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

try:
    import trimesh  # noqa: F401 -- imported for yourdfpy side-effects
    import yourdfpy

    URDF_AVAILABLE = True
except ImportError:
    URDF_AVAILABLE = False

__all__ = [
    "URDF_AVAILABLE",
    "mat3_to_wxyz",
    "load_urdf",
    "link_transforms",
]


def mat3_to_wxyz(R: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a (w, x, y, z) unit quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])


def load_urdf(urdf_path: Path) -> Optional["yourdfpy.URDF"]:
    """Load a URDF via yourdfpy, or ``None`` if unavailable/unloadable.

    No default path lives here deliberately: a URDF is workspace-relative
    (typically a sibling ``SO-ARM100/`` checkout), not something shipped
    inside this installed package, so callers must supply the path.
    """
    if not URDF_AVAILABLE:
        return None
    if not urdf_path.exists():
        return None
    try:
        return yourdfpy.URDF.load(str(urdf_path), load_meshes=True)
    except Exception:
        return None


def link_transforms(
    urdf: "yourdfpy.URDF",
    cfg: Dict[str, float],
) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """Update *urdf* to joint config *cfg* and return per-link world transforms.

    Returns ``{node_name: (wxyz_quaternion, position)}`` for every node in
    the URDF's scene graph with geometry attached.
    """
    urdf.update_cfg(cfg)
    scene = urdf.scene
    out: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    for node_name in scene.graph.nodes_geometry:
        T_world, _ = scene.graph[node_name]
        out[node_name] = (mat3_to_wxyz(T_world[:3, :3]), T_world[:3, 3])
    return out
