from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PicomotorConfig:
    """Newport 8742 picomotor controller (Ethernet). Mirror tip/tilt, manual (no PID)."""

    host: str = "10.1.137.239"
    axes: tuple[int, ...] = (1, 2, 3, 4)
    mock: bool = True
