import math

from base_core.math.enums import AngleUnit
from base_core.math.models import Angle
from control_readout.ell14.base.elliptec_device import ElliptecDevice
from control_readout.ell14.base.enums import HomeDirection
from control_readout.ell14.config import ELL14Config

COUNTS_PER_REV = 262_144  # ELL14: 262144 pulses/rev (0x40000)

class ELL14Rotator(ElliptecDevice):
    def __init__(self, config: ELL14Config) -> None:
        super().__init__()
        self._config = config
        self._current_angle: Angle = Angle(0, AngleUnit.DEG, wrap=False)

    @property
    def current_angle(self) -> Angle:
        return self._current_angle

    def home(self, direction: HomeDirection = HomeDirection.CW) -> None:
        super().home(direction)
        self._current_angle = Angle(0, AngleUnit.RAD, wrap=False)

    def apply_config(self):
        self.set_speed(self._config.speed)
        print(self.get_position_counts())

    def rotate(self, angle: Angle) -> None:
        if float(angle) == 0.0:
            return
        new_angle = Angle(self._current_angle + angle, wrap=False)
        self._validate_new_delta_angle(new_angle)
        self._move_relative(angle)
        print("Current wp angle:", self._current_angle.Deg)
        print("--------------------------")

    def _move_relative(self, angle: Angle) -> None:
        self.move_relative(self._angle_to_counts(angle))
        print(self.get_position_counts())
        self._current_angle = Angle(self._current_angle + angle, wrap=False)

    def _validate_new_delta_angle(self, new_angle: Angle) -> None:
        if self._config.angle_range.is_in_range(new_angle):
            return
        if new_angle > self._config.angle_range.max:
            correction = Angle(-self._config.out_of_range_rel_angle)
            print("corrected max")
        else:
            correction = self._config.out_of_range_rel_angle
            print("corrected min")
        self._move_relative(correction)

    def _angle_to_counts(self, angle: Angle) -> int:
        return int(round(angle.Rad / (2.0 * math.pi) * COUNTS_PER_REV))

    def _counts_to_angle(counts: int) -> Angle:
        rad = (counts / COUNTS_PER_REV) * (2.0 * math.pi)
        return Angle(rad, AngleUnit.RAD)
