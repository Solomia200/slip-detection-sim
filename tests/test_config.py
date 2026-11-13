import pytest
from pydantic import ValidationError

from slipsim.config import list_scenarios, load_scenario, scenario_to_yaml


def test_every_shipped_scenario_is_valid():
    names = list_scenarios()
    assert "grasp_lift" in names
    for name in names:
        assert load_scenario(name).phases


def test_set_changes_finger_count_and_phase_field():
    scenario = load_scenario("grasp_lift", ["hand.n_fingers=6", "phases.0.force_n=8"])
    assert scenario.hand.n_fingers == 6
    assert scenario.phases[0].force_n == 8


def test_typo_in_field_is_an_error_not_a_silent_default():
    with pytest.raises(ValidationError):
        load_scenario("grasp_lift", ["hand.n_finger=6"])


def test_unknown_phase_is_an_error():
    with pytest.raises(ValidationError):
        load_scenario("grasp_lift", ["phases.0.do=teleport"])


def test_bad_finger_count_is_an_error():
    with pytest.raises(ValidationError):
        load_scenario("grasp_lift", ["hand.n_fingers=1"])


def test_resolved_scenario_round_trips_through_yaml(tmp_path):
    path = tmp_path / "saved.yaml"
    original = load_scenario("add_weight")
    path.write_text(scenario_to_yaml(original), encoding="utf-8")
    assert load_scenario(str(path)).model_dump() == original.model_dump()
