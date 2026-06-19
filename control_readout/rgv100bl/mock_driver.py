"""Mock RGV100BL rotator — tracks angle in software, no XPS controller."""
from __future__ import annotations

from base_core.math.enums import AngleUnit
from base_core.math.models import Angle

from control_readout.rgv100bl.config import Rgv100Config


class MockRgvRotator:
    def __init__(self, config: Rgv100Config) -> None:
        self._config = config
        self._angle: Angle = Angle(0.0, AngleUnit.DEG)

    def open(self) -> None: ...
    def close(self) -> None: ...

    @property
    def current_angle(self) -> Angle:
        return self._angle

    def rotate(self, angle: Angle) -> None:
        self._angle = angle

    def home(self) -> None:
        self._angle = Angle(0.0, AngleUnit.DEG)
