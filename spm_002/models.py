from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class SpectrumData:
    """
    One acquired spectrum from the spectrometer hardware.

    Config and spectrum are kept separate: the subprocess that owns a
    Spectrometer holds its SpectrometerConfig independently; this class
    contains only the measurement result.
    """
    counts: list[int]
    wavelengths: Optional[list[float]]
    timestamp_ns: int
