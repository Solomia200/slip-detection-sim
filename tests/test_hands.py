import mujoco
import numpy as np
import pytest

from slipsim.cli import main
from slipsim.config import load_scenario
from slipsim.episode import simulate
from slipsim.recorder import contact_truth
from slipsim.scene import Scene


def test_cli_runs():
    assert main(["list"]) == 0
    assert main(["show", "parallel_lift"]) == 0


def test_xml_hand_lifts_cylinder():
    _, _, summary, _ = simulate(load_scenario("parallel_lift", ["phases.3.duration_s=0.3"]))
    assert summary["held"]
    assert summary["object_rise_mm"] > 45


def test_xml_hand_pads_face_the_object():
    scene = Scene(load_scenario("parallel_lift"))
    scene.hand.set_grip(scene.data, 6.0)
    for _ in range(1500):
        mujoco.mj_step(scene.model, scene.data)
    fn, _, _ = contact_truth(scene)
    finger = np.array([t.finger for t in scene.taxels])
    per_pad = [fn[finger == i].sum() for i in range(2)]
    # A force motor at rest: the pad carries the commanded 6 N.
    assert np.allclose(per_pad, 6.0, rtol=0.02)


def test_xml_hand_names_unknown_body():
    with pytest.raises(ValueError, match="finger_lft"):
        Scene(load_scenario("parallel_lift", ["hand.pads.0.body=finger_lft"]))


def test_xml_hand_requires_its_fields():
    with pytest.raises(Exception):
        load_scenario("parallel_lift", ["hand.palm_body=null"])
