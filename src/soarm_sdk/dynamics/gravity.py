"""Gravity torque from a URDF, in pure numpy.

Why not pinocchio
-----------------
``pinocchio.computeGeneralizedGravity`` gives the same number, but pinocchio
is a conda-only dependency (see the workspace's AGENTS.md) and this SDK
installs with pip and ships nothing heavier than numpy. Gravity needs only
the joint chain and each link's mass and centre of mass — no inertia tensors,
no velocities — so it is a few dozen lines, and a test pins it against
pinocchio wherever pinocchio happens to be installed.

Frames
------
Every ``q`` here is in the **URDF's joint frame** (the frame
:mod:`soarm_sdk.calibration.frame` maps servo ticks into). Torques are the
*generalized gravity* ``g(q)`` of ``tau = M(q) qdd + C(q, qd) qd + g(q)``:
the torque a joint must apply to hold the arm still, positive along the
joint's URDF axis.

Body parameters
---------------
Gravity torque is linear in each body's mass ``m`` and first moment of mass
``h = m * c`` (``c`` the centre of mass in the body's own link frame). Those
four numbers per body are exactly what a gravity identification can recover,
so they are the unit :meth:`GravityModel.with_body_params` accepts — the
same ``m``/``mx``/``my``/``mz`` names FIGAROH and pinocchio use.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

import numpy as np

__all__ = [
    "STANDARD_GRAVITY",
    "UrdfJoint",
    "UrdfLink",
    "GravityModel",
]

#: m/s^2, pointing down the base frame's -z.
STANDARD_GRAVITY = 9.81


@dataclass(frozen=True)
class UrdfLink:
    name: str
    mass: float = 0.0
    #: Centre of mass in the link's own frame.
    com: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    type: str
    parent: str
    child: str
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: Tuple[float, float, float] = (1.0, 0.0, 0.0)


def _floats(text: Optional[str], default: Tuple[float, ...]) -> Tuple[float, ...]:
    if text is None:
        return default
    return tuple(float(v) for v in text.split())


def _rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw: ``R = Rz(yaw) Ry(pitch) Rx(roll)``."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation about a unit *axis*."""
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def _parse_urdf(source: Union[str, Path]) -> Tuple[Dict[str, UrdfLink], List[UrdfJoint]]:
    text = str(source)
    if text.lstrip().startswith("<"):
        root = ET.fromstring(text)
    else:
        root = ET.parse(text).getroot()

    links: Dict[str, UrdfLink] = {}
    for el in root.findall("link"):
        name = el.get("name")
        inertial = el.find("inertial")
        mass, com = 0.0, (0.0, 0.0, 0.0)
        if inertial is not None:
            m = inertial.find("mass")
            if m is not None:
                mass = float(m.get("value", 0.0))
            origin = inertial.find("origin")
            if origin is not None:
                com = _floats(origin.get("xyz"), (0.0, 0.0, 0.0))  # type: ignore[assignment]
        links[name] = UrdfLink(name=name, mass=mass, com=com)

    joints: List[UrdfJoint] = []
    for el in root.findall("joint"):
        origin = el.find("origin")
        axis = el.find("axis")
        joints.append(
            UrdfJoint(
                name=el.get("name"),
                type=el.get("type"),
                parent=el.find("parent").get("link"),
                child=el.find("child").get("link"),
                xyz=_floats(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0)),  # type: ignore[arg-type]
                rpy=_floats(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0)),  # type: ignore[arg-type]
                axis=_floats(axis.get("xyz") if axis is not None else None, (1.0, 0.0, 0.0)),  # type: ignore[arg-type]
            )
        )
    return links, joints


@dataclass
class GravityModel:
    """Generalized gravity ``g(q)`` for a tree of revolute/fixed joints.

    Build one with :meth:`from_urdf`. ``joint_names`` fixes the order of
    ``q`` and of the returned torques; any movable URDF joint not listed is
    held at zero (or at ``held_positions``), so a model of the five arm
    joints can ignore the gripper jaw while still carrying its mass.
    """

    links: Dict[str, UrdfLink]
    joints: List[UrdfJoint]
    joint_names: List[str]
    gravity: float = STANDARD_GRAVITY
    held_positions: Dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        by_name = {j.name: j for j in self.joints}
        for n in self.joint_names:
            if n not in by_name:
                raise ValueError(f"joint {n!r} is not in the URDF")
            if by_name[n].type not in ("revolute", "continuous"):
                raise ValueError(f"joint {n!r} is {by_name[n].type}, not revolute")
        for j in self.joints:
            if j.type not in ("revolute", "continuous", "fixed"):
                raise ValueError(
                    f"joint {j.name!r} is {j.type}; only revolute/continuous/fixed "
                    "joints are supported"
                )
        children = {j.child for j in self.joints}
        roots = [n for n in self.links if n not in children]
        if len(roots) != 1:
            raise ValueError(f"expected one root link, found {roots}")
        self._root = roots[0]
        # Topological order: parents before children.
        order: List[UrdfJoint] = []
        frontier = [self._root]
        while frontier:
            link = frontier.pop()
            for j in self.joints:
                if j.parent == link:
                    order.append(j)
                    frontier.append(j.child)
        self._ordered = order
        self._index = {n: i for i, n in enumerate(self.joint_names)}
        self._placement = {
            j.name: (_rpy_to_matrix(j.rpy), np.asarray(j.xyz, dtype=float))
            for j in self.joints
        }
        self._axis = {}
        for j in self.joints:
            a = np.asarray(j.axis, dtype=float)
            n = np.linalg.norm(a)
            self._axis[j.name] = a / n if n > 0 else a

    # -- construction ----------------------------------------------------

    @classmethod
    def from_urdf(
        cls,
        urdf: Union[str, Path],
        joint_names: Sequence[str],
        *,
        gravity: float = STANDARD_GRAVITY,
        held_positions: Optional[Mapping[str, float]] = None,
    ) -> "GravityModel":
        """Parse *urdf* (a path, or the XML text itself)."""
        links, joints = _parse_urdf(urdf)
        return cls(
            links=links,
            joints=joints,
            joint_names=list(joint_names),
            gravity=gravity,
            held_positions=dict(held_positions or {}),
        )

    def with_body_params(
        self,
        params: Mapping[str, Mapping[str, float]],
        *,
        locked_joints: Iterable[str] = (),
    ) -> "GravityModel":
        """Replace body masses with identified ones.

        *params* maps a **joint** name to ``{"m", "mx", "my", "mz"}`` — the
        mass and first moment of the rigid body that joint moves, expressed
        in the joint's child-link frame. That body is the child link plus
        everything welded to it through fixed joints or through
        *locked_joints* (e.g. a gripper jaw held still during
        identification), which is exactly the lumping pinocchio applies when
        a model is reduced. Those welded links' own masses are zeroed so the
        identified body is not counted twice.
        """
        locked = set(locked_joints)
        by_parent: Dict[str, List[UrdfJoint]] = {}
        for j in self.joints:
            by_parent.setdefault(j.parent, []).append(j)
        by_name = {j.name: j for j in self.joints}

        links = dict(self.links)
        for jname, p in params.items():
            if jname not in by_name:
                raise ValueError(f"identified body for unknown joint {jname!r}")
            child = by_name[jname].child
            welded = [child]
            stack = [child]
            while stack:
                for j in by_parent.get(stack.pop(), []):
                    if j.type == "fixed" or j.name in locked:
                        welded.append(j.child)
                        stack.append(j.child)
            for name in welded:
                links[name] = replace(links[name], mass=0.0, com=(0.0, 0.0, 0.0))
            m = float(p["m"])
            h = np.array([float(p["mx"]), float(p["my"]), float(p["mz"])])
            com = tuple(h / m) if m > 0 else (0.0, 0.0, 0.0)
            links[child] = UrdfLink(name=child, mass=m, com=com)  # type: ignore[arg-type]
        return replace(self, links=links)

    # -- kinematics ------------------------------------------------------

    def _q_of(self, joint: UrdfJoint, q: np.ndarray) -> float:
        i = self._index.get(joint.name)
        if i is not None:
            return float(q[i])
        return float(self.held_positions.get(joint.name, 0.0))

    def link_frames(self, q: Sequence[float]) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        """World ``(R, p)`` of every link frame at configuration *q*."""
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.joint_names),):
            raise ValueError(f"expected {len(self.joint_names)} joint values, got {q.shape}")
        frames: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
            self._root: (np.eye(3), np.zeros(3))
        }
        for j in self._ordered:
            R_p, p_p = frames[j.parent]
            R_o, p_o = self._placement[j.name]
            R = R_p @ R_o
            p = p_p + R_p @ p_o
            if j.type != "fixed":
                R = R @ _axis_angle(self._axis[j.name], self._q_of(j, q))
            frames[j.child] = (R, p)
        return frames

    # -- gravity ---------------------------------------------------------

    def torque(self, q: Sequence[float]) -> np.ndarray:
        """Generalized gravity ``g(q)`` in N·m, ordered like ``joint_names``."""
        frames = self.link_frames(q)
        up = np.array([0.0, 0.0, self.gravity])  # minus the gravity vector

        # Mass and first moment of every link, in world coordinates.
        weight = {}
        for name, link in self.links.items():
            if link.mass == 0.0 or name not in frames:
                continue
            R, p = frames[name]
            weight[name] = (link.mass, p + R @ np.asarray(link.com, dtype=float))

        by_parent: Dict[str, List[UrdfJoint]] = {}
        for j in self.joints:
            by_parent.setdefault(j.parent, []).append(j)

        out = np.zeros(len(self.joint_names))
        for i, name in enumerate(self.joint_names):
            joint = next(j for j in self.joints if j.name == name)
            R, p = frames[joint.child]
            z = R @ self._axis[name]
            # Every link downstream of this joint.
            stack, subtree = [joint.child], []
            while stack:
                link = stack.pop()
                subtree.append(link)
                stack.extend(j.child for j in by_parent.get(link, []))
            tau = 0.0
            for link in subtree:
                if link in weight:
                    m, c = weight[link]
                    tau += m * float(up @ np.cross(z, c - p))
            out[i] = tau
        return out

    def lowest_point(self, q: Sequence[float]) -> Tuple[str, float]:
        """The link-frame origin with the smallest base-frame ``z``.

        A coarse clearance check for a planned motion: link *origins*, not
        link geometry, so leave a margin for the meshes around them.
        """
        frames = self.link_frames(q)
        name = min((n for n in frames if n != self._root), key=lambda n: frames[n][1][2])
        return name, float(frames[name][1][2])
