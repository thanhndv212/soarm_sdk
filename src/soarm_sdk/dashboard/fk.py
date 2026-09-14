"""Viser scene registration for the soarm_sdk dashboard's 3-D view.

The URDF loading and FK math live in :mod:`soarm_sdk.kinematics.urdf_fk`
(no Viser dependency there); this module only wires that into a Viser
scene — registering meshes, then pushing updated transforms onto their
handles each tick.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ..calibration.frame import RobotCalibration
from ..conversions import ticks_to_radians
from ..kinematics.urdf_fk import URDF_AVAILABLE, link_transforms, load_urdf

try:
    import trimesh
except ImportError:  # pragma: no cover -- guarded by URDF_AVAILABLE below
    trimesh = None  # type: ignore[assignment]

#: Where soarm-calibrate-rom / soarm-seed-calibration write by default.
DEFAULT_CALIBRATION_PATH = Path.home() / ".soarm_sdk" / "calibration.json"

__all__ = [
    "DEFAULT_CALIBRATION_PATH",
    "load_calibration",
    "SOARM100_IDS",
    "SOARM100_JOINT_NAMES",
    "URDF_AVAILABLE",
    "load_urdf_meshes",
    "load_ghost_meshes",
    "pose_meshes",
    "update_fk",
]

#: The reference-pose ghost: a translucent copy of the arm, posed where the
#: operator is being asked to put the real one. Distinct in hue from the
#: solid mirror so the two never read as one object when they overlap.
GHOST_COLOR = (90, 170, 255)
GHOST_OPACITY = 0.28

SOARM100_IDS: List[int] = [1, 2, 3, 4, 5, 6]
SOARM100_JOINT_NAMES: List[str] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


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

    urdf = load_urdf(urdf_path)
    if urdf is None:
        print(f"[soarm_sdk.dashboard] URDF not found or failed to load: {urdf_path}")
        return None, {}

    mesh_handles: Dict[str, Any] = {}
    transforms = link_transforms(urdf, {})

    for node_name, geom in _link_geometries(urdf):
        wxyz, position = transforms.get(node_name, (None, None))
        if wxyz is None:
            continue
        handle = server.scene.add_mesh_trimesh(
            name=f"/robot/{node_name}",
            mesh=geom,
            wxyz=tuple(wxyz),
            position=tuple(position),
        )
        mesh_handles[node_name] = handle

    return urdf, mesh_handles


def _link_geometries(urdf: Any):
    """Yield ``(scene-graph node name, Trimesh)`` for every renderable link."""
    scene = urdf.scene
    for node_name in scene.graph.nodes_geometry:
        _, geom_name = scene.graph[node_name]
        geom = scene.geometry.get(geom_name)
        if geom is None:
            continue
        if isinstance(geom, trimesh.Scene):
            geom = geom.dump(concatenate=True)
        if not isinstance(geom, trimesh.Trimesh):
            continue
        yield node_name, geom


def load_ghost_meshes(
    server: Any,
    urdf: Any,
    *,
    color: Tuple[int, int, int] = GHOST_COLOR,
    opacity: float = GHOST_OPACITY,
) -> Dict[str, Any]:
    """Register a translucent copy of the arm, hidden, under ``/ghost``.

    The target the operator is aiming the *real* arm at. A reference pose
    was previously only prose — "fold the arm flat against itself" — which
    leaves the operator to guess how flat, and a zero pinned to a guessed
    pose is a guessed zero. Showing the pose in the same scene, at the same
    scale, makes it something to match rather than interpret.

    Built with ``add_mesh_simple`` rather than ``add_mesh_trimesh`` because
    only the former takes ``opacity``; the link geometry is shared, so the
    silhouette is identical to the solid mirror.
    """
    if urdf is None:
        return {}
    handles: Dict[str, Any] = {}
    for node_name, geom in _link_geometries(urdf):
        handles[node_name] = server.scene.add_mesh_simple(
            name=f"/ghost/{node_name}",
            vertices=geom.vertices,
            faces=geom.faces,
            color=color,
            opacity=opacity,
            # Two-sided: a translucent shell viewed from inside shows its
            # back faces, and culling them leaves visible holes in the arm.
            side="double",
            cast_shadow=False,
            receive_shadow=False,
            visible=False,
        )
    return handles


def pose_meshes(urdf: Any, cfg: Dict[str, float], handles: Dict[str, Any]) -> None:
    """Pose a mesh handle set at an explicit configuration, in URDF radians.

    No calibration and no servo readings: *cfg* is the configuration itself,
    which is what a reference pose already is.
    """
    if urdf is None or not handles:
        return
    transforms = link_transforms(urdf, cfg)
    for node_name, handle in handles.items():
        wxyz, position = transforms.get(node_name, (None, None))
        if wxyz is None:
            continue
        handle.wxyz = tuple(wxyz)
        handle.position = tuple(position)


def load_calibration(
    path: Optional[Path] = None,
) -> Optional[RobotCalibration]:
    """Load the arm's tick-to-URDF-frame calibration, or ``None`` if absent.

    Without this the 3-D view has to assume every joint reads zero radians at
    tick 2048 and increases in the servo's own direction. Neither holds on a
    real arm: on the arm this was written against, that assumption puts the
    main joints 16-30 degrees out, the gripper 74 degrees out, and turns
    ``wrist_roll`` the wrong way. The mismatch is in the *view*, not the
    robot — the ticks were right all along.
    """
    p = Path(path) if path is not None else DEFAULT_CALIBRATION_PATH
    if not p.exists():
        print(
            f"[soarm_sdk.dashboard] no calibration at {p} — the 3-D view will "
            "assume tick 2048 is zero for every joint and will not match the "
            "real arm. Run soarm-calibrate-rom or soarm-seed-calibration."
        )
        return None
    try:
        calib = RobotCalibration.load(p)
    except Exception as exc:
        print(f"[soarm_sdk.dashboard] could not read {p}: {exc}")
        return None
    if not calib.validated:
        print(
            f"[soarm_sdk.dashboard] calibration {p} is marked validated=false; "
            "the 3-D view may still be mirrored on some joints."
        )
    return calib


def update_fk(
    urdf: Any,
    positions: Dict[int, int],
    mesh_handles: Dict[str, Any],
    joint_ids: Optional[List[int]] = None,
    joint_names: Optional[List[str]] = None,
    calibration: Optional[RobotCalibration] = None,
) -> None:
    """Recompute FK from joint positions (ticks) and push transforms to Viser.

    With *calibration*, ticks are mapped through the arm's measured zero
    offsets and direction signs; without it, through the nominal
    tick-2048-is-zero assumption, which will not match a real arm.
    """
    ids = joint_ids if joint_ids is not None else SOARM100_IDS
    names = joint_names if joint_names is not None else SOARM100_JOINT_NAMES

    by_name = {}
    if calibration is not None:
        by_name = {j.name: j for j in calibration.joints}

    cfg: Dict[str, float] = {}
    for sid, jname in zip(ids, names):
        ticks = positions.get(sid)
        if ticks is None:
            continue
        joint = by_name.get(jname)
        cfg[jname] = (
            joint.to_rad(ticks) if joint is not None else ticks_to_radians(ticks)
        )

    if not cfg:
        return

    transforms = link_transforms(urdf, cfg)
    for node_name, handle in mesh_handles.items():
        wxyz, position = transforms.get(node_name, (None, None))
        if wxyz is None:
            continue
        handle.wxyz = tuple(wxyz)
        handle.position = tuple(position)
