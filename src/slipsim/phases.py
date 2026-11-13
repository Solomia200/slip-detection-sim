"""Scenario phases: the steps a YAML scenario lists under `phases:`.

To add a phase, write one class here with `@phase("name")`; scenarios can use it
straight away. `apply` runs before every physics step with `frac` growing to 1.
"""
from typing import ClassVar

from pydantic import BaseModel, ConfigDict, Field, ValidationError

PHASES = {}


def phase(name):
    def decorator(cls):
        PHASES[name] = cls
        return cls
    return decorator


def parse_phase(raw):
    if isinstance(raw, Phase):
        return raw
    name = raw.get("do") if isinstance(raw, dict) else None
    if name not in PHASES:
        raise ValueError(f"невідома фаза {name!r}; доступні: {sorted(PHASES)}")
    return PHASES[name].model_validate(raw)


class Phase(BaseModel):
    model_config = ConfigDict(extra="forbid")

    do: str
    duration_s: float = Field(1.0, gt=0)

    # True for phases that push the grasp towards slipping; slip events are only
    # looked for from the first such phase onward.
    disturbs: ClassVar[bool] = False

    def begin(self, ctx):
        pass

    def apply(self, ctx, frac):
        pass


@phase("hold")
class Hold(Phase):
    """Keep every command as it is."""


@phase("close")
class Close(Phase):
    """Ramp the force of every finger from its current value up to `force_n`."""
    force_n: float = Field(gt=0)
    duration_s: float = Field(0.8, gt=0)

    def begin(self, ctx):
        ctx.unlock_fingers()
        self._start = ctx.grip_force

    def apply(self, ctx, frac):
        ctx.set_grip(self._start + (self.force_n - self._start) * frac)


@phase("lock_fingers")
class LockFingers(Phase):
    """Hold the fingers where they are with a spring, on top of the grip force.

    Purely force-driven fingers give way under a sideways load, so the object moves
    WITH them instead of slipping. Use this before any non-symmetric load.
    """
    stiffness_n_per_mm: float = Field(5.0, gt=0)
    duration_s: float = Field(0.01, gt=0)

    def begin(self, ctx):
        ctx.lock_fingers(self.stiffness_n_per_mm * 1e3)


@phase("lift")
class Lift(Phase):
    """Raise the gripper by `height_mm` along a smooth profile."""
    height_mm: float

    def begin(self, ctx):
        self._start = ctx.lift_target

    def apply(self, ctx, frac):
        smooth = frac * frac * (3.0 - 2.0 * frac)
        ctx.set_lift(self._start + self.height_mm * 1e-3 * smooth)


@phase("ramp_load")
class RampLoad(Phase):
    """Ramp an external force and torque on the object from zero, then keep it."""
    force_n: tuple[float, float, float] = (0.0, 0.0, 0.0)
    torque_nm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    duration_s: float = Field(10.0, gt=0)
    disturbs: ClassVar[bool] = True

    def apply(self, ctx, frac):
        ctx.set_load([f * frac for f in self.force_n], [t * frac for t in self.torque_nm])


@phase("ramp_grip")
class RampGrip(Phase):
    """Ramp the force of every finger down (or up) to `to_force_n`."""
    to_force_n: float = Field(ge=0)
    duration_s: float = Field(5.0, gt=0)
    disturbs: ClassVar[bool] = True

    def begin(self, ctx):
        ctx.unlock_fingers()
        self._start = ctx.grip_force

    def apply(self, ctx, frac):
        ctx.set_grip(self._start + (self.to_force_n - self._start) * frac)


__all__ = ["PHASES", "Phase", "ValidationError", "parse_phase", "phase"]
