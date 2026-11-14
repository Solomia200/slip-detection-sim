"""Record one episode, keeping sensor data and ground truth strictly apart.

`taxels.npz` holds only what a skin could physically measure (taxel deflection).
`truth.npz` holds what only the engine knows (contact forces, slip, object motion).
Detectors read the first file alone, so ground truth cannot leak into them.
"""
import numpy as np

# A taxel counts as slipping when it sits on the friction cone AND moves relative
# to the object. Both conditions matter: soft contacts creep a little while stuck.
CONE_FRACTION = 0.9
SLIP_SPEED_MPS = 1e-3


def _point_velocity(data, model, bodies, points):
    """World velocity of `points` rigidly attached to `bodies`, vectorised."""
    cvel = data.cvel[bodies]
    offset = points - data.subtree_com[model.body_rootid[bodies]]
    return cvel[:, 3:6] + np.cross(cvel[:, :3], offset)


def contact_truth(scene):
    """Per-taxel normal force, shear force and slip speed from the current contacts."""
    model, data = scene.model, scene.data
    n = len(scene.taxels)
    fn, ft, slip = np.zeros(n), np.zeros(n), np.zeros(n)
    if data.ncon == 0:
        return fn, ft, slip

    geom_to_taxel = scene.geom_to_taxel
    contact = data.contact
    geom1, geom2 = contact.geom1, contact.geom2
    is_taxel_1 = geom_to_taxel[geom1] >= 0
    taxel_geom = np.where(is_taxel_1, geom1, geom2)
    idx = geom_to_taxel[taxel_geom]
    mask = (idx >= 0) & (contact.efc_address >= 0)
    if not mask.any():
        return fn, ft, slip

    idx = idx[mask]
    adr = contact.efc_address[mask]
    points = contact.pos[mask]
    frames = contact.frame[mask]
    # Elliptic cone: efc_force holds (normal, tangent1, tangent2) in contact frame.
    np.add.at(fn, idx, np.maximum(data.efc_force[adr], 0.0))
    np.add.at(ft, idx, np.hypot(data.efc_force[adr + 1], data.efc_force[adr + 2]))

    object_bodies = np.full(len(idx), scene.object_body)
    rel = (_point_velocity(data, model, object_bodies, points)
           - _point_velocity(data, model, scene.taxel_bodies[idx], points))
    normals = frames[:, :3]
    rel_t = rel - np.sum(rel * normals, axis=1, keepdims=True) * normals
    np.maximum.at(slip, idx, np.linalg.norm(rel_t, axis=1))
    return fn, ft, slip


class Recorder:
    def __init__(self, scene, max_frames):
        self.scene = scene
        n = len(scene.taxels)
        self.frame = 0
        self.mu = scene.scenario.object.mu
        self.t = np.zeros(max_frames)
        self.phase = np.zeros(max_frames, dtype=np.int16)
        self.deflection = np.zeros((max_frames, n, 3), dtype=np.float32)
        self.fn = np.zeros((max_frames, n), dtype=np.float32)
        self.ft = np.zeros((max_frames, n), dtype=np.float32)
        self.slip_speed = np.zeros((max_frames, n), dtype=np.float32)
        self.object_pos = np.zeros((max_frames, 3))
        self.palm_z = np.zeros(max_frames)
        self.object_rel_speed = np.zeros(max_frames)
        self.last_slipping = np.zeros(n, dtype=bool)
        self.last_fn = np.zeros(n)

    def record(self, phase_index):
        scene, data, i = self.scene, self.scene.data, self.frame
        self.t[i] = data.time
        self.phase[i] = phase_index
        self.deflection[i] = data.qpos[scene.taxel_qpos]
        fn, ft, slip = contact_truth(scene)
        self.fn[i], self.ft[i], self.slip_speed[i] = fn, ft, slip
        self.last_fn = fn
        self.last_slipping = is_slipping(fn, ft, slip, self.mu)
        self.object_pos[i] = data.xpos[scene.object_body]
        self.palm_z[i] = data.xpos[scene.hand.palm_body][2]
        velocity = data.cvel[scene.object_body, 3:6].copy()
        velocity[2] -= scene.hand.palm_speed_z(data)
        self.object_rel_speed[i] = np.linalg.norm(velocity)
        self.frame += 1

    def object_drop(self):
        """Current object height relative to the palm."""
        i = self.frame - 1
        return self.object_pos[i, 2] - self.palm_z[i]

    def save(self, out_dir, scenario):
        taxels = self.scene.taxels
        count = self.frame
        pad = scenario.pad
        np.savez_compressed(
            out_dir / "taxels.npz",
            t=self.t[:count],
            deflection_m=self.deflection[:count],    # (frames, taxels, [normal, y, z])
            finger=np.array([t.finger for t in taxels]),
            pad_y_m=np.array([t.y for t in taxels]),
            pad_z_m=np.array([t.z for t in taxels]),
            k_normal=pad.k_normal,
            k_shear=pad.k_shear,
        )
        np.savez_compressed(
            out_dir / "truth.npz",
            t=self.t[:count],
            phase=self.phase[:count],
            fn=self.fn[:count],
            ft=self.ft[:count],
            slip_speed=self.slip_speed[:count],
            object_pos=self.object_pos[:count],
            palm_z=self.palm_z[:count],
            object_rel_speed=self.object_rel_speed[:count],
            mu=self.mu,
        )


def is_slipping(fn, ft, slip_speed, mu):
    return (fn > 1e-6) & (ft >= CONE_FRACTION * mu * fn) & (slip_speed > SLIP_SPEED_MPS)
