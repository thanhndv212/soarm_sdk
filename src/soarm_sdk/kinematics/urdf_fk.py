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
    "MEMBERS",
    "member_pitches",
]

#: The arm's three visible straight sections, as ``(name, link, body_axis)``.
#:
#: Each is a rigid member whose pitch an operator can measure directly —
#: a phone inclinometer laid on it, or a spirit level. That is the whole
#: point: a joint angle is a number inside the model with no independent
#: witness, but "is the forearm level?" is a question the arm itself
#: answers. Every zero in the calibration was ultimately pinned this way.
#:
#: ``body_axis`` is a unit vector **in the link's own frame**, along the
#: member's long axis — the direction a level laid on it would follow.
#:
#: It used to be the chord between two joint-frame *origins*, and that is
#: not the same line. On the SO-101 the shoulder and elbow origins sit off
#: the upper arm's axis, so the chord runs about 14 deg away from the body:
#: at ``folded_flat`` the chord read level while the member was visibly
#: sloped. Every pose here was solved to make the chords come out right,
#: so every one of them was wrong by that offset — and so was every zero
#: pinned against them. Measured from the shell meshes' oriented bounding
#: boxes; ``tests/test_reference_poses.py`` re-derives them from the URDF.
MEMBERS: Tuple[Tuple[str, str, Tuple[float, float, float]], ...] = (
    ("upper_arm", "upper_arm_link", (-0.999998, -0.000187, -0.001815)),
    ("forearm", "lower_arm_link", (-1.000000, 0.000002, -0.000969)),
    ("wrist_section", "wrist_link", (0.012632, -0.999920, 0.000052)),
)


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


def member_pitches(
    urdf: "yourdfpy.URDF",
    cfg: Dict[str, float],
) -> Dict[str, float]:
    """Pitch above horizontal, in degrees, of each member in :data:`MEMBERS`.

    The bridge between a joint configuration and something an operator can
    check without trusting the calibration: put the arm in a pose, read
    these numbers off the model, and compare them to a level held against
    the real member. A disagreement here is the calibration's zero being
    wrong, stated in the one unit the hardware can be measured in.

    Members whose links the URDF does not carry are omitted rather than
    guessed at.
    """
    urdf.update_cfg({k: float(v) for k, v in cfg.items()})
    scene = urdf.scene
    out: Dict[str, float] = {}
    for name, link, body_axis in MEMBERS:
        try:
            T, _ = scene.graph[link]
        except Exception:
            continue
        v = T[:3, :3] @ np.asarray(body_axis, dtype=float)
        norm = float(np.linalg.norm(v))
        if norm < 1e-9:
            continue
        # Elevation above horizontal, which is what a level or an
        # inclinometer reads. It folds at +-90 deg, exactly as the
        # instrument does: neither can say which side of vertical you are on.
        out[name] = float(np.degrees(np.arctan2(v[2], float(np.hypot(v[0], v[1])))))
    return out
