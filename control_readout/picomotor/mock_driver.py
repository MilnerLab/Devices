"""Mock picomotor — tracks open-loop step counts per axis, no controller."""
from __future__ import annotations

from control_readout.picomotor.config import PicomotorConfig


class MockPicomotor:
    def __init__(self, config: PicomotorConfig) -> None:
        self._config = config
        self._steps: dict[int, int] = {axis: 0 for axis in config.axes}

    def open(self) -> None: ...
    def close(self) -> None: ...

    def move_by(self, axis: int, steps: int) -> None:
        self._steps[axis] = self._steps.get(axis, 0) + steps

    def position(self, axis: int) -> int:
        return self._steps.get(axis, 0)
