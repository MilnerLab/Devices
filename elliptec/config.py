from dataclasses import dataclass

from base_core.math.models import Angle, Range
from elliptec.elliptec_ell14 import AngleUnit


@dataclass
class ELL14Config:
    speed: int = 70 #percent
    angle_range: Range[Angle] = Range(Angle(-90, AngleUnit.DEG), Angle(90, AngleUnit.DEG))
    out_of_range_rel_angle = Angle(90, AngleUnit.DEG)
    