# slipsim

A MuJoCo simulation for the UCU × Infineon research project "Shear Force and Slip Detection
Method for Tactile Sensors": how fast, and how early, can tactile sensor data reveal that a
grasped object is starting to slip?

## Installation

```bash
python3.12 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
slipsim doctor        # should print PASS for 3, 4 and 6 fingers
```

## Usage

```bash
slipsim list                                        # available scenarios
slipsim run grasp_lift --view                       # run with the 3D viewer
slipsim run grasp_lift --set hand.n_fingers=6       # same, with 6 fingers
slipsim run parallel_lift --view                    # two-finger gripper loaded from XML
slipsim run xarm_lift --view                        # UFACTORY xArm Gripper loaded from XML
slipsim run add_weight --set hand.type=xarm_gripper # any scenario with UFACTORY xArm Gripper
slipsim run add_weight --set object.mu=0.3 --set pad.pitch_mm=1 --set pad.rows=13
slipsim show add_weight                             # all parameters of a scenario
```

In the viewer, taxels turn blue under pressure and red when they slip.

Every run creates a folder `results/<scenario>_<time>/`:

| File | Contents |
|---|---|
| `config.yaml` | all run parameters; this file can be run again |
| `manifest.json` | summary, library versions, commit |
| `plot.png` | height, forces, slip margin w, fraction of slipping taxels |
| `taxels.npz` | **sensor data:** deflection of every taxel, nothing else |
| `truth.npz` | **ground truth:** contact forces, slip, object motion |

Sensor data and ground truth are kept in separate files on purpose: a detector reads only
`taxels.npz`, so ground truth cannot leak into it by accident.

## Writing a scenario

Create `scenarios/my_test.yaml`. List only what differs from the defaults:

```yaml
description: "Heavy slippery cylinder, six fingers"
hand: {n_fingers: 6}
object: {mass_g: 500, mu: 0.3}
phases:
  - {do: close, force_n: 8}
  - {do: lift, height_mm: 50}
  - {do: ramp_load, force_n: [0, 0, -40], duration_s: 12}
```
Run it with `slipsim run my_test`. A misspelled field name is reported as an error.

| Phase | What it does |
|---|---|
| `close` | close smoothly; `force_n` is the force on **each** finger |
| `hold` | change nothing |
| `lift` | raise the hand by `height_mm` |
| `lock_fingers` | hold the fingers on springs; **required before any non-symmetric load** |
| `ramp_load` | smoothly apply an external force `force_n` and torque `torque_nm` |
| `ramp_grip` | smoothly change the grip force to `to_force_n` |

## Extending

| To add | Where | How |
|---|---|---|
| Scenario | `scenarios/` | one YAML file |
| Scenario phase | `phases.py` | one class with `@phase("name")` |
| Object | `scene.py` | one function with `@scene_object("name")` |
| Hand from an MJCF file | `hands/<name>/` | add the file and describe the hand in a scenario (below); no Python |
| Hand built in code | `hands.py` | a `Hand` subclass with `@hand("name")`; `attach_pad` adds the pads |

## Connecting a hand from XML

The file must contain only the hand: no floor and no object. Example:
`hands/parallel_gripper/parallel_gripper.xml` with `scenarios/parallel_lift.yaml`:

```yaml
hand:
  type: xml
  file: hands/parallel_gripper/parallel_gripper.xml
  palm_body: palm                  # object motion is measured relative to this body
  grip_actuators: [left, right]    # the close phase writes its value to these actuators
  lift_actuator: lift              # the lift phase writes the height here (optional)
  pads:                            # a pad faces along +x of its frame
    - {body: finger_left, euler_deg: [0, 0, -90]}
    - {body: finger_right, euler_deg: [0, 0, 90], pos_mm: [0, 0, 0]}
```

- Names are written as in the file. If a body does not exist, the error lists the ones that do.
- The hand's own geoms collide with nothing; only the pads touch the object.
- `close: {force_n: 6}` writes the number 6 to the actuators as is. For a `motor` actuator
  that is 6 N; for a `position` actuator it is 6 metres. A position-controlled hand needs its
  own phase.
- `<option>` settings in the hand file have no effect: `scene.py` sets the world physics.

## Structure

```
scenario.yaml → config.py → scene.py ─┬─ hands.py (hand) ── pad.py (pad / sensor)
                (validate)   (world)   └─ object, table
             → episode.py + phases.py → recorder.py → analysis.py
               (step loop)               (two files)   (summary, plot)
```
