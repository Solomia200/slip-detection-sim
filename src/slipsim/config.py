"""Scenario files: load YAML, apply `--set` overrides, validate everything once.

Every field has a default, so a scenario file only lists what it changes.
`slipsim show <name>` prints the full result. Unknown fields are errors, so a typo
cannot silently fall back to a default.
"""
from pathlib import Path
from typing import Annotated, Any, Literal, Union

import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator

from slipsim.phases import parse_phase

REPO_ROOT = Path(__file__).resolve().parents[2]


class Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


class RadialHandConfig(Strict):
    """Built-in gripper: N straight fingers around the object (hands.RadialGripper)."""
    type: Literal["radial"] = "radial"
    n_fingers: int = Field(3, ge=2, le=12)
    clearance_mm: float = Field(5.0, gt=0)      # gap between pad and object before closing
    finger_damping: float = Field(20.0, gt=0)   # N*s/m; sets the closing speed
    lift_kp: float = Field(20000.0, gt=0)       # N/m
    lift_kv: float = Field(500.0, gt=0)         # N*s/m


Vec3 = tuple[float, float, float]


class PadMount(Strict):
    """Where a pad sits on a body of an XML hand. The pad faces local +x of this frame."""
    body: str
    pos_mm: Vec3 = (0.0, 0.0, 0.0)              # offset from the body origin
    euler_deg: Vec3 = (0.0, 0.0, 0.0)           # rotation (x, y, z order) relative to the body


class XmlHandConfig(Strict):
    """A hand from an MJCF file (hands.XmlHand). Names are as written in that file."""
    type: Literal["xml"]
    file: str                                   # relative to the working dir or the repository
    pos_mm: Vec3 = (0.0, 0.0, 0.0)              # where the file's world origin is placed
    euler_deg: Vec3 = (0.0, 0.0, 0.0)
    palm_body: str                              # reference body: object motion is relative to it
    grip_actuators: list[str] = Field(min_length=1)
    lift_actuator: str | None = None            # a position actuator moving the hand up
    pads: list[PadMount] = Field(min_length=1)


HandConfig = Annotated[Union[RadialHandConfig, XmlHandConfig], Field(discriminator="type")]


class PadConfig(Strict):
    """Compliant "brush" pad: one sprung sphere per taxel on a domed base.

    The stiffness, damping and armature values below are NOT calibrated against a
    real elastomer yet. Armature gives the very light taxels enough effective mass
    for a stiff contact without adding weight.
    """
    pitch_mm: float = Field(2.0, gt=0)
    rows: int = Field(7, ge=1)
    dome_radius_mm: float = Field(12.0, gt=0)
    k_normal: float = Field(2000.0, gt=0)       # N/m per taxel
    k_shear: float = Field(2000.0, gt=0)        # N/m per taxel
    damping: float = Field(1.0, gt=0)           # N*s/m per taxel joint
    armature: float = Field(0.003, gt=0)        # kg per taxel joint
    taxel_radius_mm: float = Field(1.0, gt=0)


class ObjectConfig(Strict):
    type: str = "cylinder"
    radius_mm: float = Field(25.0, gt=0)
    height_mm: float = Field(100.0, gt=0)
    mass_g: float = Field(200.0, gt=0)
    mu: float = Field(0.5, gt=0)


class Scenario(Strict):
    name: str = "unnamed"
    description: str = ""
    hand: HandConfig = RadialHandConfig()
    pad: PadConfig = PadConfig()
    object: ObjectConfig = ObjectConfig()
    record_every: int = Field(1, ge=1)          # physics steps per recorded frame
    stop_if_dropped_mm: float = Field(30.0, gt=0)
    phases: list[Any] = Field(min_length=1)

    @field_validator("hand", mode="before")
    @classmethod
    def _default_hand_type(cls, value):
        # `--set hand.n_fingers=6` on a file without a hand section gives no type.
        if isinstance(value, dict) and "type" not in value:
            value = {**value, "type": "radial"}
        return value

    @field_validator("phases")
    @classmethod
    def _parse_phases(cls, value):
        return [parse_phase(item) for item in value]


def resolve_path(name):
    """A data file named in a scenario: absolute, or relative to the working dir or repo."""
    path = Path(name)
    for candidate in ([path] if path.is_absolute() else [Path.cwd() / path, REPO_ROOT / path]):
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"файл {name!r} не знайдено")


def hand_label(scenario):
    """Short human-readable name of the hand, for CLI output and plot titles."""
    hand = scenario.hand
    if hand.type == "radial":
        return f"радіальний гріпер, {hand.n_fingers} пальці(в)"
    return f"{Path(hand.file).stem}, подушечок: {len(hand.pads)}"


def scenario_dirs():
    return [Path.cwd() / "scenarios", REPO_ROOT / "scenarios"]


def list_scenarios():
    found = {}
    for directory in reversed(scenario_dirs()):
        for path in sorted(directory.glob("*.yaml")):
            found[path.stem] = path
    return found


def find_scenario(name):
    path = Path(name)
    if path.suffix in (".yaml", ".yml") and path.exists():
        return path
    known = list_scenarios()
    if name not in known:
        raise FileNotFoundError(f"сценарій {name!r} не знайдено; доступні: {sorted(known)}")
    return known[name]


def _set_path(tree, dotted, value):
    keys = dotted.split(".")
    node = tree
    for key in keys[:-1]:
        if isinstance(node, list):
            node = node[int(key)]
        else:
            node = node.setdefault(key, {})
    if isinstance(node, list):
        node[int(keys[-1])] = value
    else:
        node[keys[-1]] = value


def apply_overrides(raw, overrides):
    for item in overrides or []:
        if "=" not in item:
            raise ValueError(f"--set очікує шлях=значення, отримано {item!r}")
        dotted, text = item.split("=", 1)
        try:
            _set_path(raw, dotted.strip(), yaml.safe_load(text))
        except (IndexError, ValueError, AttributeError) as error:
            raise ValueError(f"--set {item!r}: такого шляху немає ({error})") from error
    return raw


def load_scenario(name, overrides=None):
    path = find_scenario(name)
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw.setdefault("name", path.stem)
    return Scenario.model_validate(apply_overrides(raw, overrides))


def scenario_to_yaml(scenario):
    return yaml.safe_dump(scenario.model_dump(mode="json"), allow_unicode=True, sort_keys=False)
