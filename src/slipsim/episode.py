"""Run one scenario: step the physics through its phases and write a results folder."""
import json
import platform
import subprocess
import time
from datetime import datetime
from pathlib import Path

import mujoco
import numpy as np

from slipsim.analysis import analyse, plot_episode
from slipsim.config import REPO_ROOT, scenario_to_yaml
from slipsim.recorder import Recorder
from slipsim.scene import Scene


class Context:
    """What a phase may command. Phases never touch MuJoCo directly."""

    def __init__(self, scene):
        self._scene = scene
        self.grip_force = 0.0
        self.lift_target = 0.0
        self._lock_stiffness = None
        self._lock_positions = None

    def set_grip(self, force):
        self.grip_force = force
        self._scene.hand.set_grip(self._scene.data, force)

    def lock_fingers(self, stiffness):
        self._lock_stiffness = stiffness
        self._lock_positions = self._scene.hand.finger_positions(self._scene.data)

    def unlock_fingers(self):
        self._lock_stiffness = None

    def before_step(self):
        """Apply the finger spring, if locked; called before every physics step."""
        if self._lock_stiffness is None:
            return
        hand, data = self._scene.hand, self._scene.data
        spring = self._lock_stiffness * hand.grip_gear * (self._lock_positions - hand.finger_positions(data))
        hand.set_grip(data, self.grip_force + spring)

    def set_lift(self, height):
        self.lift_target = height
        self._scene.hand.set_lift(self._scene.data, height)

    def set_load(self, force, torque):
        applied = self._scene.data.xfrc_applied[self._scene.object_body]
        applied[:3] = force
        applied[3:] = torque


class _LiveView:
    """Optional 3D window; taxels turn red while slipping, blue with pressure."""

    REFRESH_STEPS = 16

    def __init__(self, scene):
        import mujoco.viewer
        self._scene = scene
        self._viewer = mujoco.viewer.launch_passive(scene.model, scene.data)
        self._viewer.cam.lookat[:] = [0, 0, 0.06]
        self._viewer.cam.distance = 0.4
        self._viewer.cam.elevation = -20
        self._wall_start = time.perf_counter()

    def update(self, step, rec):
        if step % self.REFRESH_STEPS:
            return True
        if not self._viewer.is_running():
            return False
        rgba = self._scene.model.geom_rgba[self._scene.taxel_geoms]
        pressure = np.clip(rec.last_fn / 0.3, 0, 1)
        rgba[:, 0] = np.where(rec.last_slipping, 1.0, 0.85 * (1 - pressure))
        rgba[:, 1] = np.where(rec.last_slipping, 0.1, 0.85 * (1 - pressure))
        rgba[:, 2] = np.where(rec.last_slipping, 0.1, 0.85)
        self._scene.model.geom_rgba[self._scene.taxel_geoms] = rgba
        self._viewer.sync()
        ahead = self._scene.data.time - (time.perf_counter() - self._wall_start)
        if ahead > 0:
            time.sleep(ahead)
        return True

    def close(self):
        self._viewer.close()


def _results_dir(name, out_root):
    root = Path(out_root) if out_root else Path.cwd() / "results"
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = root / f"{name}_{stamp}"
    suffix = 1
    while path.exists():
        suffix += 1
        path = root / f"{name}_{stamp}_{suffix}"
    path.mkdir(parents=True)
    return path


def _git_commit():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT,
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def simulate(scenario, view=False):
    """Run the physics only. Returns (scene, recorder, summary, seconds of wall time)."""
    scene = Scene(scenario)
    model, data = scene.model, scene.data

    dt = model.opt.timestep
    plan = [(index, phase, max(1, int(round(phase.duration_s / dt))))
            for index, phase in enumerate(scenario.phases)]
    total = sum(steps for _, _, steps in plan)
    rec = Recorder(scene, total // scenario.record_every + 1)
    ctx = Context(scene)
    live = _LiveView(scene) if view else None

    watch_start = None
    baseline_rel_z = None
    step = 0
    stopped = False
    wall_start = time.perf_counter()
    for index, phase, steps in plan:
        if phase.disturbs and watch_start is None:
            watch_start = float(data.time)
        phase.begin(ctx)
        for sub in range(steps):
            phase.apply(ctx, (sub + 1) / steps)
            ctx.before_step()
            mujoco.mj_step(model, data)
            if step % scenario.record_every == 0:
                rec.record(index)
            step += 1
            if live is not None and not live.update(step, rec):
                stopped = True
                break
            if (baseline_rel_z is not None and step % 10 == 0
                    and abs(rec.object_drop() - baseline_rel_z) > scenario.stop_if_dropped_mm * 1e-3):
                stopped = True
                break
        if phase.do == "close" and baseline_rel_z is None and rec.frame:
            baseline_rel_z = rec.object_drop()
        if stopped:
            break
    wall = time.perf_counter() - wall_start
    if live is not None:
        live.close()
    return scene, rec, analyse(rec, scenario, watch_start, baseline_rel_z), wall


def run_episode(scenario, view=False, out_root=None):
    scene, rec, summary, wall = simulate(scenario, view=view)
    out = _results_dir(scenario.name, out_root)
    (out / "config.yaml").write_text(scenario_to_yaml(scenario), encoding="utf-8")
    rec.save(out, scenario)
    plot_episode(rec, summary, scenario, out / "plot.png")
    manifest = {
        "scenario": scenario.name,
        "summary": summary,
        "sim_time_s": float(scene.data.time),
        "wall_time_s": wall,
        "speed_x_realtime": float(scene.data.time / wall) if wall > 0 else None,
        "degrees_of_freedom": int(scene.model.nv),
        "mujoco": mujoco.__version__,
        "numpy": np.__version__,
        "python": platform.python_version(),
        "git_commit": _git_commit(),
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False),
                                       encoding="utf-8")
    return out, manifest
