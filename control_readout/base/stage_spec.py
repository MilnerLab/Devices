"""``StageSpec`` — what is true about a stage model, independent of what it is used for.

Lives in the Devices repo, next to :mod:`mock_params`, because these are properties of
the hardware: the model name, the units it counts in, and the soft limits configured on
its controller. Nothing here knows which arm of which experiment the stage happens to be
bolted into — that is the application's business, and it changes when the rig changes
while these numbers do not.

The split matters because the same facts were previously written out in four places that
could disagree, and did: the application's scan limits, a UI panel's arm table, the mock
controller's motion profiles, and each worker's axis constant. The mock had drifted into
a *different coordinate frame* (datasheet travel rather than the post-homing frame the
application commands in), which silently clamped every mocked scan. One definition per
stage, imported by all four, is what stops that recurring.
"""
from __future__ import annotations

from dataclasses import dataclass


class StageLimitError(ValueError):
    """A commanded position lies outside the stage's soft limits."""


@dataclass(frozen=True)
class StageSpec:
    """One stage model's identity, units and travel.

    ``limit_min``/``limit_max`` are the soft limits **as configured on the controller**,
    in the stage's native units — i.e. the frame the application commands in, which is
    not necessarily the datasheet travel. A stage homed to a mid-travel origin reports
    negative positions, and it is those numbers that belong here.
    """

    #: Manufacturer's model designation, e.g. ``"FMS300PP"``. Reaches run provenance.
    model: str
    #: Native units of every position and limit below: ``"mm"``, ``"deg"``, ``"steps"``.
    units: str
    limit_min: float
    limit_max: float

    @property
    def limits(self) -> tuple[float, float]:
        return (self.limit_min, self.limit_max)

    @property
    def travel(self) -> float:
        return self.limit_max - self.limit_min

    def in_limits(self, value: float) -> bool:
        return self.limit_min <= value <= self.limit_max

    def clamp(self, value: float) -> float:
        return min(max(value, self.limit_min), self.limit_max)

    def validate(self, value: float, *, label: str | None = None) -> None:
        """Raise :class:`StageLimitError` unless ``value`` is within the soft limits.

        ``label`` names the caller's role for the stage ("probe", "delay") so the message
        reads in the application's vocabulary rather than the model number's.
        """
        if self.in_limits(value):
            return
        who = label or self.model
        raise StageLimitError(
            f"{who} setpoint {value:.4f} {self.units} outside "
            f"[{self.limit_min}, {self.limit_max}]"
        )
