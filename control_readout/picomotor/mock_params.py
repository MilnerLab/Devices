"""Mock picomotor parameters. Subprocess-side only; never crosses IPC."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MockPicomotorParams:
    """How the fake 8742 pretends to step.

    ``step_time_s`` exists so a 5000-step move does not complete instantly. On the real
    controller that move takes minutes, and an instant mock hides every place the UI
    assumes a step is free.
    """

    step_time_s: float = 0.0002
    #: Ceiling on any single simulated move, so a mistyped target cannot hang a panel.
    max_sleep_s: float = 2.0

    def travel_time_s(self, steps: int) -> float:
        return min(abs(int(steps)) * self.step_time_s, self.max_sleep_s)
