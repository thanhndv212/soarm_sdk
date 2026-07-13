"""Forward-kinematics + URDF mesh helpers for the soarm_sdk dashboard.

No default URDF path lives here deliberately: a URDF is workspace-relative
(it comes from a sibling ``SO-ARM100/`` checkout, not from anything shipped
inside this installed package), so callers must supply ``urdf_path``
explicitly — see ``examples/setup_dashboard.py`` for how the example script
computes its own default.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from ..conversions import ticks_to_radians

try:
    import trimesh  # noqa: F401 -- imported for yourdfpy side-effects
    import yourdfpy

    URDF_AVAILABLE = True
except ImportError:
    URDF_AVAILABLE = False

__all__ = [
    "SOARM100_IDS",
    "SOARM100_JOINT_NAMES",
    "URDF_AVAILABLE",
    "mat3_to_wxyz",
    "load_urdf_meshes",
    "update_fk",
]

SOARM100_IDS: List[int] = [1, 2, 3, 4, 5, 6]
SOARM100_JOINT_NAMES: List[str] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
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


def load_urdf_meshes(
    server: Any,
    urdf_path: Path,
) -> Tuple[Any, Dict[str, Any]]:
    """Load the URDF and register all link meshes in the Viser scene.

    Returns ``(urdf_object, mesh_handles)`` where ``mesh_handles`` maps
    scene-graph node name -> ``GlbHandle``. Both are ``None`` / ``{}`` on
    failure so callers can skip FK updates gracefully.
    """
    if not URDF_AVAILABLE:
        print("[soarm_sdk.dashboard] yourdfpy/trimesh not installed - no 3-D view.")
        return None, {}

    if not urdf_path.exists():
        print(f"[soarm_sdk.dashboard] URDF not found: {urdf_path}")
        return None, {}

    try:
        urdf = yourdfpy.URDF.load(str(urdf_path), load_meshes=True)
    except Exception as exc:
        print(f"[soarm_sdk.dashboard] URDF load failed: {exc}")
        return None, {}

    scene = urdf.scene
    mesh_handles: Dict[str, Any] = {}

    for node_name in scene.graph.nodes_geometry:
        T_world, geom_name = scene.graph[node_name]
        geom = scene.geometry.get(geom_name)
        if geom is None:
            continue
        if isinstance(geom, trimesh.Scene):
            geom = geom.dump(concatenate=True)
        if not isinstance(geom, trimesh.Trimesh):
            continue

        wxyz = mat3_to_wxyz(T_world[:3, :3])
        handle = server.scene.add_mesh_trimesh(
            name=f"/robot/{node_name}",
            mesh=geom,
            wxyz=tuple(wxyz),
            position=tuple(T_world[:3, 3]),
        )
        mesh_handles[node_name] = handle

    return urdf, mesh_handles


def update_fk(
    urdf: Any,
    positions: Dict[int, int],
    mesh_handles: Dict[str, Any],
    joint_ids: Optional[List[int]] = None,
    joint_names: Optional[List[str]] = None,
) -> None:
    """Recompute FK from joint positions (ticks) and push transforms to Viser."""
    cfg: Dict[str, float] = {}
    for sid, jname in zip(
        joint_ids if joint_ids is not None else SOARM100_IDS,
        joint_names if joint_names is not None else SOARM100_JOINT_NAMES,
    ):
        ticks = positions.get(sid)
        if ticks is not None:
            cfg[jname] = ticks_to_radians(ticks)

    if not cfg:
        return

    urdf.update_cfg(cfg)
    scene = urdf.scene
    for node_name, handle in mesh_handles.items():
        T_world, _ = scene.graph[node_name]
        handle.wxyz = tuple(mat3_to_wxyz(T_world[:3, :3]))
        handle.position = tuple(T_world[:3, 3])
