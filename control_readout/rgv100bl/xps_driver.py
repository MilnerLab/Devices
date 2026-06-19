"""
Real RGV100BL driver via a Newport XPS controller (Ethernet, ``newportxps``).

Not exercised in CI (needs the controller + lib; the XPS requires an admin account).
Mirrors :class:`MockRgvRotator`. API names follow the ``newportxps`` package
(``initialize_group``/``home_group``/``move_stage``/``get_stage_position``) — confirm
on the bench (M1.C).
"""
from __future__ import annotations

from base_core.math.enums import AngleUnit
from base_core.math.models import Angle

from control_readout.rgv100bl.config import Rgv100Config


def _deg(angle: Angle) -> float:
    return float(angle) / AngleUnit.DEG.value


class XpsRgvRotator:
    def __init__(self, config: Rgv100Config) -> None:
        self._config = config
        self._xps = None
        self._angle: Angle = Angle(0.0, AngleUnit.DEG)

    def open(self) -> None:
        from newportxps import NewportXPS

        c = self._config
        self._xps = NewportXPS(c.host, username=c.username, password=c.password, port=c.port)
        self._xps.initialize_group(c.group)
        self._xps.home_group(c.group)

    def close(self) -> None:
        self._xps = None

    @property
    def current_angle(self) -> Angle:
        return self._angle

    def rotate(self, angle: Angle) -> None:
        self._require().move_stage(self._config.positioner, _deg(angle))
        self._angle = angle

    def home(self) -> None:
        self._require().home_group(self._config.group)
        self._angle = Angle(0.0, AngleUnit.DEG)

    def _require(self):
        if self._xps is None:
            raise RuntimeError("XpsRgvRotator: not open")
        return self._xps
