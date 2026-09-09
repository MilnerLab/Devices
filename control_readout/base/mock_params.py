"""Motion profiles for the mock controllers. Subprocess-side only; never crosses IPC."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MockMotionProfile:
    """How a mock axis pretends to move.

    Per-axis rather than global because the rig's axes are not alike: a 300 mm
    linear stage and a waveplate rotator that indexes in encoder counts have nothing
    in common, and a mock that ignores that teaches the operator the wrong reflexes.
    """

    #: Travel limits in the axis' native units. Moves are clamped, because a mock that
    #: happily drives past a hard stop trains exactly the habit the real stage punishes.
    lower: float = -1e9
    upper: float = 1e9
    #: Native units per second. Used to work out how long a move should appear to take.
    velocity: float = 100.0
    #: Where ``home`` leaves the axis.
    home_position: float = 0.0
    #: Seconds a home search appears to take.
    home_time_s: float = 0.5
    #: Ceiling on any single simulated move, so a mistyped target cannot hang a panel
    #: for ten minutes of pretend travel.
    max_sleep_s: float = 2.0
    #: Multiplier on every simulated duration. 0.0 makes the mock instant, which is what
    #: a unit test wants; 1.0 is the realistic default the UI needs.
    time_scale: float = 1.0

    def clamp(self, value: float) -> float:
        return min(max(value, self.lower), self.upper)

    def travel_time_s(self, distance: float) -> float:
        if self.velocity <= 0.0:
            return 0.0
        return min(abs(distance) / self.velocity, self.max_sleep_s) * self.time_scale


#: A stage that responds instantly. For tests that assert behaviour, not timing.
INSTANT = MockMotionProfile(time_scale=0.0, home_time_s=0.0)
