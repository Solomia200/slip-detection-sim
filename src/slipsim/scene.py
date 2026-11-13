"""Build the MuJoCo world for a scenario: physics options, table, object, hand with pads.

Hands live in hands.py, the tactile pad in pad.py. To add an object, register one more
builder here with @scene_object("name"); its body must be called "object".
"""
import mujoco
import numpy as np

from slipsim.hands import HANDS
from slipsim.pad import OBJECT_CONAFFINITY, OBJECT_CONTYPE, TABLE_CONAFFINITY

OBJECTS = {}


def scene_object(name):
    def decorator(fn):
        OBJECTS[name] = fn
        return fn
    return decorator


@scene_object("cylinder")
def _cylinder(spec, cfg):
    r, half_h = cfg.radius_mm * 1e-3, cfg.height_mm * 0.5e-3
    body = spec.worldbody.add_body(name="object", pos=[0, 0, half_h])
    body.add_freejoint(name="object_free")
    geom = body.add_geom(name="object_geom", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                         size=[r, half_h, 0], mass=cfg.mass_g * 1e-3,
                         contype=OBJECT_CONTYPE, conaffinity=OBJECT_CONAFFINITY,
                         rgba=[1.0, 0.55, 0.1, 1.0])
    geom.friction = [cfg.mu, 0.005, 0.0001]
    geom.condim = 3


class Scene:
    def __init__(self, scenario):
        self.scenario = scenario
        spec = mujoco.MjSpec()
        spec.option.timestep = 0.001
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC   # recorder relies on this
        spec.option.impratio = 10

        world = spec.worldbody
        world.add_light(pos=[0, 0, 1.0], dir=[0, 0, -1])
        table = world.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0.4, 0.4, 0.01],
                               contype=0, conaffinity=TABLE_CONAFFINITY, rgba=[0.45, 0.5, 0.55, 1])
        table.condim = 3

        if scenario.object.type not in OBJECTS:
            raise ValueError(f"невідомий предмет {scenario.object.type!r}; доступні: {sorted(OBJECTS)}")
        OBJECTS[scenario.object.type](spec, scenario.object)
        self.hand = HANDS[scenario.hand.type](spec, scenario)

        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.hand.bind(self.model)
        self.object_body = self.model.body("object").id

        # Index tables so the recorder can read every taxel at once.
        self.taxels = self.hand.taxels
        self.taxel_geoms = np.array([self.model.geom(t.name).id for t in self.taxels])
        self.taxel_bodies = self.model.geom_bodyid[self.taxel_geoms]
        self.geom_to_taxel = np.full(self.model.ngeom, -1)
        self.geom_to_taxel[self.taxel_geoms] = np.arange(len(self.taxels))
        self.taxel_qpos = np.array([[self.model.joint(f"{t.name}_{s}").qposadr[0] for s in "nyz"]
                                    for t in self.taxels])
