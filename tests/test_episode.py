import mujoco
import numpy as np
import pytest

from slipsim.config import load_scenario
from slipsim.episode import run_episode, simulate
from slipsim.recorder import contact_truth, is_slipping
from slipsim.scene import Scene


@pytest.mark.parametrize("n", [3, 4, 6])
def test_gripper_builds_with_n_evenly_spaced_fingers(n):
    scene = Scene(load_scenario("grasp_lift", [f"hand.n_fingers={n}"]))
    mujoco.mj_forward(scene.model, scene.data)
    assert scene.model.nu == n + 1   # n finger motors + lift
    assert len(scene.taxels) == n * 49
    xy = np.array([scene.data.xpos[scene.model.body(f"finger{f}").id][:2] for f in range(n)])
    assert np.allclose(np.linalg.norm(xy, axis=1), np.linalg.norm(xy[0]))
    assert np.allclose(xy.sum(axis=0), 0, atol=1e-12)


@pytest.mark.parametrize("n", [3, 4, 6])
def test_grasp_lift_picks_cylinder_off_the_table(n):
    _, _, summary, _ = simulate(load_scenario("grasp_lift", [f"hand.n_fingers={n}"]))
    assert summary["held"]
    assert summary["object_rise_mm"] > 45


def test_weak_grip_cannot_lift():
    _, _, summary, _ = simulate(load_scenario("grasp_lift", ["phases.0.force_n=0.2"]))
    assert not summary["held"]
    assert summary["object_rise_mm"] < 10


def _scene_in_contact():
    scene = Scene(load_scenario("add_weight"))
    scene.hand.set_grip(scene.data, 6.0)
    for _ in range(800):
        mujoco.mj_step(scene.model, scene.data)
    return scene


def test_fast_contact_truth_matches_mujoco_functions():
    scene = _scene_in_contact()
    scene.data.xfrc_applied[scene.object_body, 2] = -4.0
    for _ in range(200):
        mujoco.mj_step(scene.model, scene.data)
    model, data = scene.model, scene.data
    fn, ft, slip = contact_truth(scene)
    assert (fn > 0).sum() > 20

    wrench, body_vel, object_vel = np.zeros(6), np.zeros(6), np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, scene.object_body, object_vel, 0)
    checked = 0
    for c in range(data.ncon):
        con = data.contact[c]
        taxel = max(scene.geom_to_taxel[con.geom1], scene.geom_to_taxel[con.geom2])
        if taxel < 0:
            continue
        mujoco.mj_contactForce(model, data, c, wrench)
        assert fn[taxel] == pytest.approx(max(wrench[0], 0), abs=1e-9)
        assert ft[taxel] == pytest.approx(np.hypot(wrench[1], wrench[2]), abs=1e-9)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                                 scene.taxel_bodies[taxel], body_vel, 0)
        # Compare against explicit rigid-body velocities taken at the contact point.
        p = con.pos
        v_obj = object_vel[3:] + np.cross(object_vel[:3], p - data.xpos[scene.object_body])
        v_tax = body_vel[3:] + np.cross(body_vel[:3], p - data.xpos[scene.taxel_bodies[taxel]])
        rel = v_obj - v_tax
        normal = con.frame[:3]
        expected = np.linalg.norm(rel - rel.dot(normal) * normal)
        assert slip[taxel] == pytest.approx(expected, abs=1e-9)
        checked += 1
    assert checked > 20


def test_slip_starts_at_pad_edge_and_precedes_object_motion():
    """The physics this project exists for: incipient slip, edge first."""
    scenario = load_scenario("add_weight")
    scene, rec, summary, _ = simulate(scenario)
    assert summary["gross_slip_s"] is not None
    assert summary["first_taxel_slip_s"] < summary["gross_slip_s"] - 0.2
    assert summary["w_at_first_taxel_slip"] > summary["w_at_gross_slip"] + 0.1

    count = rec.frame
    slipping = is_slipping(rec.fn[:count], rec.ft[:count], rec.slip_speed[:count], rec.mu)
    watched = rec.t[:count] >= summary["watch_start_s"]
    onset = np.full(slipping.shape[1], np.inf)
    for taxel in range(slipping.shape[1]):
        hits = np.flatnonzero(watched & slipping[:, taxel])
        if hits.size:
            onset[taxel] = rec.t[hits[0]]
    y = np.array([t.y for t in scene.taxels])
    z = np.array([t.z for t in scene.taxels])
    radius = np.hypot(y, z)
    contacted = rec.fn[:count].max(axis=0) > 1e-6
    finite = contacted & np.isfinite(onset)
    centre = finite & (radius < 0.003)
    edge = finite & (radius > 0.005)
    assert centre.any() and edge.any()
    assert np.median(onset[edge]) < np.median(onset[centre]) - 0.2


def test_side_pull_slips_instead_of_pushing_locked_fingers_away():
    scenario = load_scenario("pull_side", ["phases.5.force_n=[12, 0, 0]", "phases.5.duration_s=3"])
    scene = Scene(scenario)
    start = None
    from slipsim.episode import Context
    ctx = Context(scene)
    for index, phase in enumerate(scenario.phases):
        phase.begin(ctx)
        steps = int(round(phase.duration_s / scene.model.opt.timestep))
        for sub in range(steps):
            phase.apply(ctx, (sub + 1) / steps)
            ctx.before_step()
            mujoco.mj_step(scene.model, scene.data)
        if phase.do == "lock_fingers":
            start = scene.hand.finger_positions(scene.data)
    travel = np.abs(scene.hand.finger_positions(scene.data) - start)
    # 12 N on a 5 N/mm spring: fingers may flex by a few mm, not fly away.
    assert travel.max() < 4e-3


def test_finger_travel_is_limited_when_grip_is_lost():
    scene, _, _, _ = simulate(load_scenario("loosen_grip"))
    limit = scene.hand._MAX_TRAVEL + scene.scenario.hand.clearance_mm * 1e-3
    assert np.abs(scene.hand.finger_positions(scene.data)).max() < limit + 2e-3


def test_sensor_file_holds_no_ground_truth(tmp_path):
    out, _ = run_episode(load_scenario("grasp_lift", ["phases.3.duration_s=0.1"]), out_root=tmp_path)
    with np.load(out / "taxels.npz") as taxels:
        assert set(taxels.files) == {"t", "deflection_m", "finger", "pad_y_m", "pad_z_m",
                                     "k_normal", "k_shear"}
    with np.load(out / "truth.npz") as truth:
        assert {"fn", "ft", "slip_speed", "object_pos", "mu"} <= set(truth.files)
    assert (out / "plot.png").stat().st_size > 0
    assert (out / "config.yaml").exists() and (out / "manifest.json").exists()


def test_same_scenario_gives_identical_result():
    scenario = load_scenario("grasp_lift", ["phases.3.duration_s=0.1"])
    _, first, _, _ = simulate(scenario)
    _, second, _, _ = simulate(scenario)
    assert np.array_equal(first.deflection[:first.frame], second.deflection[:second.frame])
