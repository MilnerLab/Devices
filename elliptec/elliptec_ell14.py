import math
import time

from typing import Optional
from base_core.math.enums import AngleUnit
from base_core.math.models import Angle, Range
from elliptec.base.elliptec_device import ElliptecDevice
from elliptec.config import ELL14Config

COUNTS_PER_REV = 262_144  # ELL14: 262144 pulses/rev (0x40000) :contentReference[oaicite:3]{index=3}

class Rotator(ElliptecDevice):
    def __init__(
        self,
        config: ELL14Config
    ) -> None:
        super().__init__()
        self._config = config
        self._current_angle: Angle = Angle(0, AngleUnit.DEG, wrap= False)

    def apply_config(self):
        self.set_speed(self._config.speed)
        print(self.get_position_counts())

    def rotate(self, angle: Angle) -> None:
        
        if float(angle) == 0.0:
            return

        new_angle = Angle(self._current_angle + angle, wrap= False)
        self._validate_new_delta_angle(new_angle)
        self._move_relative(angle)
        print("Current wp angle:", self._current_angle.Deg)
        print("--------------------------")


    # ------------------------------------------------------------------    
    # internal helpers
    # ------------------------------------------------------------------    


    def _move_relative(self, angle: Angle) -> None:
        self.move_relative(self._angle_to_counts(angle))
        print(self.get_position_counts())
        
        self._current_angle = Angle(self._current_angle + angle, wrap= False)

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