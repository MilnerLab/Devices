from __future__ import annotations

from dataclasses import dataclass

from base_core.math.enums import AngleUnit
from base_core.math.models import Angle, Range


@dataclass
class Rgv100Config:
    """Configuration for the Newport RGV100BL rotation stage (via an XPS controller).

    Drives the HWP (D11). ``mock=True`` uses a synthetic rotator (no controller).
    """

    host: str = "10.1.137.137"        # XPS IP
    username: str = "PyControl"
    password: str = "labview2python"
    port: int = 5001
    group: str = "GROUP1"             # XPS positioner/group name
    positioner: str = "GROUP1.POSITIONER"
    angle_range: Range[Angle] = Range(Angle(-180, AngleUnit.DEG), Angle(180, AngleUnit.DEG))
    mock: bool = True
