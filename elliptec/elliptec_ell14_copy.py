import math
import time

from typing import Optional
from base_lib.math.enums import AngleUnit
from base_lib.math.models import Angle, Range
from elliptec.elliptec_trace import ElliptecDevice

# === Konstanten ===
ANGLE_RANGE = Range(Angle(-90, AngleUnit.DEG), Angle(90, AngleUnit.DEG))
OUT_OF_RANGE_RELATIVE_ANGLE = Angle(90, AngleUnit.DEG)
HOME_ANGLE = Angle(0, AngleUnit.DEG)
COUNTS_PER_REV = 262_144  # ELL14: 262144 pulses/rev (0x40000) :contentReference[oaicite:3]{index=3}

class Rotator(ElliptecDevice):
    def __init__(
        self,
        port: str,
        *,
        address: Optional[str] = None,
        min_address: str = "0",
        max_address: str = "F",
    ) -> None:
        super().__init__(port, address=address, min_address=min_address, max_address=max_address)
        self.home()
        self._current_angle: Angle = Angle(0, AngleUnit.DEG)


    def rotate(self, angle: Angle) -> None:
        
        if float(angle) == 0.0:
            return

        new_angle = Angle(self._current_angle + angle)
        self._validate_new_delta_angle(new_angle)
        self._move_relative(angle)
        print("Current wp angle:", self._current_angle.Deg)
        print("--------------------------")


    # ------------------------------------------------------------------    
    # internal helpers
    # ------------------------------------------------------------------    


    def _move_relative(self, angle: Angle) -> None:
        self.move_relative(self._angle_to_counts(angle))
        self._current_angle = Angle(self._current_angle + angle)
        time.sleep(2.0)

    def _validate_new_delta_angle(self, new_angle: Angle) -> None:
        
        if ANGLE_RANGE.is_in_range(new_angle):
            return

        if new_angle > ANGLE_RANGE.max:
            correction = Angle(-OUT_OF_RANGE_RELATIVE_ANGLE)
            print("corrected max")
        else:
            correction = OUT_OF_RANGE_RELATIVE_ANGLE
            print("corrected min")

        self._move_relative(correction)
        
    def _angle_to_counts(self, angle: Angle) -> int:
        return int(round(angle.Rad / (2.0 * math.pi) * COUNTS_PER_REV))
        
    