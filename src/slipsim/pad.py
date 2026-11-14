"""The tactile pad: a grid of sprung spheres ("taxels") that any hand can carry.

`attach_pad` only needs a parent body or frame whose local +x points at the object.
Hands call it once per finger; nothing here knows which hand it is on.
"""
from dataclasses import dataclass

import mujoco

# Collision bits. Two geoms touch when (contype_a & conaffinity_b) or (contype_b & conaffinity_a)
# is non-zero. With these values taxels touch only the object, and the object also touches
# the table; taxels never touch each other, the table or the hand itself.
TAXEL_CONTYPE, OBJECT_CONAFFINITY = 1, 1
OBJECT_CONTYPE, TABLE_CONAFFINITY = 2, 2


def explicit_body(parent, name, pos, mass, inertia, quat=None):
    """A body with hand-set mass and no weight (gravity is compensated)."""
    body = parent.add_body(name=name, pos=list(pos), childclass="main")
    if quat is not None:
        body.quat = list(quat)
    body.explicitinertial = True
    body.mass = mass
    body.inertia = [inertia] * 3
    body.gravcomp = 1.0   # hand and taxel weight would otherwise bias the taxel readings
    return body


@dataclass
class Taxel:
    name: str
    finger: int
    y: float   # position on the pad, across the finger [m]
    z: float   # position on the pad, along the finger [m]


def attach_pad(parent, prefix, finger, cfg, mu, base_x):
    """Grid of sprung spheres on a domed base; local +x of `parent` faces the object.

    Each taxel has three spring joints in the order (normal, y, z). The dome makes
    the normal force fall toward the edge, which is what lets the edge slip first.
    """
    pitch = cfg.pitch_mm * 1e-3
    half = (cfg.rows - 1) * pitch / 2
    dome = cfg.dome_radius_mm * 1e-3
    taxels = []
    for iy in range(cfg.rows):
        for iz in range(cfg.rows):
            y, z = -half + iy * pitch, -half + iz * pitch
            name = f"{prefix}_t{iy}_{iz}"
            x = base_x - (y * y + z * z) / (2 * dome)
            body = explicit_body(parent, name, (x, y, z), 2e-4, 1e-9)
            for suffix, axis, k in (("n", (1, 0, 0), cfg.k_normal),
                                    ("y", (0, 1, 0), cfg.k_shear),
                                    ("z", (0, 0, 1), cfg.k_shear)):
                joint = body.add_joint(name=f"{name}_{suffix}", type=mujoco.mjtJoint.mjJNT_SLIDE,
                                       axis=list(axis))
                joint.stiffness = [k, 0, 0]
                joint.damping = [cfg.damping, 0, 0]
                joint.armature = cfg.armature
            geom = body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_SPHERE,
                                 size=[cfg.taxel_radius_mm * 1e-3, 0, 0],
                                 contype=TAXEL_CONTYPE, conaffinity=0,
                                 rgba=[0.85, 0.85, 0.85, 1.0])
            geom.friction = [mu, 0.005, 0.0001]
            geom.condim = 3
            taxels.append(Taxel(name, finger, y, z))
    return taxels
