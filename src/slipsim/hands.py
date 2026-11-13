"""Robot hands. A hand adds its bodies to the world, puts pads on its fingers and obeys a
few commands from the phases (through episode.Context).

Two ways to add a hand:
  1. From an MJCF file (a teammate's model, MuJoCo Menagerie, ...): no Python needed.
     Use `hand: {type: xml, file: ..., pads: [...]}` in the scenario; see XmlHand.
  2. Built in code: subclass Hand and register it with @hand("name"); see RadialGripper.
"""
import mujoco
import numpy as np

from slipsim.config import resolve_path
from slipsim.pad import attach_pad, explicit_body

HANDS = {}


def hand(name):
    def decorator(cls):
        HANDS[name] = cls
        return cls
    return decorator


class Hand:
    """Everything the rest of slipsim asks of a hand.

    `__init__(spec, scenario)` adds the hand to `spec` and fills `self.taxels` (via
    attach_pad). After the model is compiled, `bind(model)` is called once; it must set
    `self.palm_body`. The step loop then only uses the methods below.
    """

    taxels: list
    palm_body: int   # body id of the hand's reference body ("palm"); set in bind()

    def bind(self, model):
        """Look up actuator, joint and body ids by name; called once after compiling."""
        raise NotImplementedError

    def set_grip(self, data, value):
        """Grip command for every finger (a number, or one value per finger)."""
        raise NotImplementedError

    def finger_positions(self, data):
        """Current position of each finger joint; used by the lock_fingers phase."""
        raise NotImplementedError

    def set_lift(self, data, height):
        """Move the hand to `height` metres above where it started."""
        raise NotImplementedError

    def palm_speed_z(self, data):
        """Vertical speed of the palm [m/s]; the object's speed is measured relative to it."""
        raise NotImplementedError


@hand("radial")
class RadialGripper(Hand):
    """N straight fingers spaced evenly around the object, each on a radial slider.

    Each slider has a pure force actuator, so the commanded force IS the normal force
    at rest. The whole gripper moves vertically on a stiff position servo.
    """

    _PALM_ABOVE_PAD = 0.07
    _STANDOFF = 0.008   # distance from finger frame to the pad surface [m]
    _MAX_TRAVEL = 0.02  # how far a finger may overrun past its nominal travel [m]

    def __init__(self, spec, scenario):
        cfg, pad = scenario.hand, scenario.pad
        obj = scenario.object
        self.n_fingers = cfg.n_fingers
        self.taxels = []
        pad_half = (pad.rows - 1) * pad.pitch_mm * 0.5e-3
        grasp_z = obj.height_mm * 0.5e-3
        self._palm_z0 = grasp_z + self._PALM_ABOVE_PAD

        palm = explicit_body(spec.worldbody, "palm", (0, 0, self._palm_z0), 0.3, 1e-3)
        palm.add_joint(name="lift", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1])
        palm.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, contype=0, conaffinity=0,
                      size=[obj.radius_mm * 1e-3 + 0.03, 0.006, 0], rgba=[0.15, 0.15, 0.18, 1])
        lift = spec.add_actuator(name="lift", target="lift", trntype=mujoco.mjtTrn.mjTRN_JOINT)
        lift.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        lift.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        lift.gainprm[0] = cfg.lift_kp
        lift.biasprm[1] = -cfg.lift_kp
        lift.biasprm[2] = -cfg.lift_kv

        radius = (obj.radius_mm * 1e-3 + self._STANDOFF + pad.taxel_radius_mm * 1e-3
                  + cfg.clearance_mm * 1e-3)
        for f in range(cfg.n_fingers):
            angle = 2 * np.pi * f / cfg.n_fingers
            quat = np.zeros(4)
            mujoco.mju_axisAngle2Quat(quat, np.array([0.0, 0.0, 1.0]), angle + np.pi)
            finger = explicit_body(palm, f"finger{f}",
                                   (radius * np.cos(angle), radius * np.sin(angle),
                                    -self._PALM_ABOVE_PAD), 0.05, 1e-5, quat=quat)
            joint = finger.add_joint(name=f"finger{f}_slide", type=mujoco.mjtJoint.mjJNT_SLIDE,
                                     axis=[1, 0, 0])
            joint.damping = [cfg.finger_damping, 0, 0]
            joint.limited = mujoco.mjtLimited.mjLIMITED_TRUE
            joint.range = [-self._MAX_TRAVEL, cfg.clearance_mm * 1e-3 + self._MAX_TRAVEL]
            spec.add_actuator(name=f"finger{f}", target=f"finger{f}_slide",
                              trntype=mujoco.mjtTrn.mjTRN_JOINT)
            finger.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, contype=0, conaffinity=0,
                            size=[0.003, pad_half + 0.002, pad_half + 0.002], pos=[0, 0, 0],
                            rgba=[0.15, 0.15, 0.18, 1])
            finger.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, contype=0, conaffinity=0,
                            size=[0.003, 0.005, self._PALM_ABOVE_PAD / 2],
                            pos=[-0.003, 0, self._PALM_ABOVE_PAD / 2],
                            rgba=[0.15, 0.15, 0.18, 1])
            self.taxels += attach_pad(finger, f"f{f}", f, pad, obj.mu, self._STANDOFF)

    def bind(self, model):
        self._lift_act = model.actuator("lift").id
        self._lift_qvel = model.joint("lift").dofadr[0]
        self._finger_acts = np.array([model.actuator(f"finger{f}").id
                                      for f in range(self.n_fingers)])
        self._finger_qpos = np.array([model.joint(f"finger{f}_slide").qposadr[0]
                                      for f in range(self.n_fingers)])
        self.palm_body = model.body("palm").id

    def set_grip(self, data, value):
        data.ctrl[self._finger_acts] = value

    def finger_positions(self, data):
        return data.qpos[self._finger_qpos].copy()

    def set_lift(self, data, height):
        data.ctrl[self._lift_act] = height

    def palm_speed_z(self, data):
        return data.qvel[self._lift_qvel]


@hand("xml")
class XmlHand(Hand):
    """Any hand described in an MJCF file; the scenario names its parts.

    The file should hold the hand only (no floor or object). Everything in it gets the
    prefix "hand/", but the scenario uses the names as written in the file. Geoms of the
    hand itself do not collide with anything; only the pads touch the object.

    `set_grip` writes the value into every grip actuator as it is. Whether that means
    newtons or metres depends on the actuator in the file (motor vs position).
    """

    PREFIX = "hand/"

    def __init__(self, spec, scenario):
        cfg = scenario.hand
        self._cfg = cfg
        path = resolve_path(cfg.file)
        child = mujoco.MjSpec.from_file(str(path))
        for geom in child.geoms:
            geom.contype, geom.conaffinity = 0, 0
        frame = spec.worldbody.add_frame(pos=_metres(cfg.pos_mm), quat=_quat(cfg.euler_deg))
        spec.attach(child, prefix=self.PREFIX, frame=frame)

        self.taxels = []
        for i, mount in enumerate(cfg.pads):
            body = spec.body(self.PREFIX + mount.body)
            if body is None:
                names = sorted(b.name.removeprefix(self.PREFIX) for b in spec.bodies
                               if b.name.startswith(self.PREFIX))
                raise ValueError(f"у {path.name} немає тіла {mount.body!r}; є: {names}")
            pad_frame = body.add_frame(pos=_metres(mount.pos_mm), quat=_quat(mount.euler_deg))
            self.taxels += attach_pad(pad_frame, f"f{i}", i, scenario.pad, scenario.object.mu, 0.0)

    def bind(self, model):
        def find(kind, name):
            try:
                return getattr(model, kind)(self.PREFIX + name).id
            except KeyError:
                raise ValueError(f"у моделі руки немає {kind} {name!r}") from None

        cfg = self._cfg
        self._grip_acts = np.array([find("actuator", n) for n in cfg.grip_actuators])
        joints = model.actuator_trnid[self._grip_acts, 0]
        self._grip_qpos = model.jnt_qposadr[joints]
        self._lift_act = None if cfg.lift_actuator is None else find("actuator", cfg.lift_actuator)
        self.palm_body = find("body", cfg.palm_body)
        self._model = model
        self._velocity = np.zeros(6)

    def set_grip(self, data, value):
        data.ctrl[self._grip_acts] = value

    def finger_positions(self, data):
        return data.qpos[self._grip_qpos].copy()

    def set_lift(self, data, height):
        if self._lift_act is None:
            raise ValueError("у цієї руки не вказано lift_actuator, фаза lift недоступна")
        data.ctrl[self._lift_act] = height

    def palm_speed_z(self, data):
        mujoco.mj_objectVelocity(self._model, data, mujoco.mjtObj.mjOBJ_BODY, self.palm_body,
                                 self._velocity, 0)
        return self._velocity[5]   # world-frame linear velocity, z


def _metres(mm):
    return [v * 1e-3 for v in mm]


def _quat(euler_deg):
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.radians(euler_deg), "xyz")
    return quat
