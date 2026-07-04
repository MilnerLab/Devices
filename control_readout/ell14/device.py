"""
ell14/device.py
===============

`ELL14Rotator` — the ELL14 rotary waveplate mount as a framework `Device`.

It is addressed by its hex bus address ('0'..'F') and delegates every command to
the `ElliptecController` that owns the serial line. Elliptec positions are in
encoder *counts*, so the framework's `move_to` / `move_by` already operate in
counts; on top of that this class adds ELL14 angle semantics (deg ↔ counts), a
configured travel range and software angle tracking.
"""

from __future__ import annotations

import logging
import math

from base_core.math.enums import AngleUnit
from base_core.math.models import Angle
from control_readout.base.device import Device
from control_readout.ell14.config import ELL14Config
from control_readout.ell14.controller import (
    ElliptecController,
    HomeDirection,
    StatusCode,
    _normalize_address,
)

log = logging.getLogger(__name__)

COUNTS_PER_REV = 262_144  # ELL14: 262144 pulses/rev (0x40000)


class ELL14Rotator(Device):
    """ELL14 rotary waveplate mount (units: deg) on an Elliptec serial bus."""

    units = "deg"

    def __init__(
        self,
        name: str,
        address: str,
        controller: ElliptecController,
        config: ELL14Config,
    ) -> None:
        super().__init__(name, address=_normalize_address(address), controller=controller)
        self._config = config
        self._current_angle: Angle = Angle(0, AngleUnit.DEG, wrap=False)

    @property
    def _elliptec(self) -> ElliptecController:
        return self.controller  # type: ignore[return-value]

    @property
    def current_angle(self) -> Angle:
        return self._current_angle

    # -- Elliptec bus commands -------------------------------------------- #
    # Elliptec positions are in encoder counts, so the framework's move_to /
    # move_by (delegating to the controller's move_absolute / move_relative)
    # already work in counts. Motion-stop is the framework's ``abort()``
    # (Device.stop() means "detach from the controller").
    @property
    def status(self) -> StatusCode:
        return self._elliptec.status(self.address)

    def get_status(self) -> StatusCode:
        return self._elliptec.get_status(self.address)

    def get_speed(self) -> int:
        return self._elliptec.get_speed(self.address)

    def set_speed(self, percent: int) -> None:
        self._elliptec.set_speed(self.address, percent)

    def get_position_counts(self) -> int:
        return self._elliptec.get_position_counts(self.address)

    def home(self, direction: HomeDirection = HomeDirection.CW) -> None:
        self._elliptec.home(self.address, direction)
        self._current_angle = Angle(0, AngleUnit.RAD, wrap=False)

    # -- ELL14 angle semantics -------------------------------------------- #
    def apply_config(self) -> None:
        self.set_speed(self._config.speed)
        log.debug("%s applied config; position counts=%d", self.name, self.get_position_counts())

    def rotate(self, angle: Angle) -> None:
        if float(angle) == 0.0:
            return
        new_angle = Angle(self._current_angle + angle, wrap=False)
        self._validate_new_delta_angle(new_angle)
        self._move_relative(angle)
        log.debug("%s current angle: %.3f deg", self.name, self._current_angle.Deg)

    def _move_relative(self, angle: Angle) -> None:
        self.move_by(self._angle_to_counts(angle))
        self._current_angle = Angle(self._current_angle + angle, wrap=False)
        log.debug("%s position counts=%d", self.name, self.get_position_counts())

    def _validate_new_delta_angle(self, new_angle: Angle) -> None:
        if self._config.angle_range.is_in_range(new_angle):
            return
        if new_angle > self._config.angle_range.max:
            correction = Angle(-self._config.out_of_range_rel_angle)
            log.debug("%s corrected above max of range", self.name)
        else:
            correction = self._config.out_of_range_rel_angle
            log.debug("%s corrected below min of range", self.name)
        self._move_relative(correction)

    def _angle_to_counts(self, angle: Angle) -> int:
        return int(round(angle.Rad / (2.0 * math.pi) * COUNTS_PER_REV))
