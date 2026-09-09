"""Mock picomotor — tracks open-loop step counts per axis, no controller."""
from __future__ import annotations

import time

from control_readout.picomotor.config import PicomotorConfig
from control_readout.picomotor.mock_params import MockPicomotorParams


class MockPicomotor:
    """Stands in for a New Focus 8742. Mirrors :class:`Picomotor8742` exactly.

    The counter semantics are the interesting part, and they are the real device's:
    open-loop, per-axis, and only ever changed by a command. Nothing here reports a
    calibrated position, because the hardware has no encoder to give one.
    """

    def __init__(
        self,
        config: PicomotorConfig,
        params: MockPicomotorParams = MockPicomotorParams(),
    ) -> None:
        self._config = config
        self._params = params
        self._steps: dict[int, int] = {axis: 0 for axis in config.axes}

    def open(self) -> None: ...
    def close(self) -> None: ...

    def move_by(self, axis: int, steps: int) -> None:
        time.sleep(self._params.travel_time_s(steps))
        self._steps[axis] = self._steps.get(axis, 0) + int(steps)

    def move_to(self, axis: int, steps: int) -> None:
        self.move_by(axis, int(steps) - self._steps.get(axis, 0))

    def zero(self, axis: int) -> None:
        """Re-reference this axis' counter. Moves nothing, and touches no other axis."""
        self._steps[axis] = 0

    def position(self, axis: int) -> int:
        return self._steps.get(axis, 0)

    def is_moving(self, axis: int) -> bool:
        # Moves block here, so by the time anyone can ask, the axis has stopped.
        return False

    def wait_for_stop(self, axis: int, timeout_s: float = 30.0) -> bool:
        return True
