# acquisition/spm002/models.py
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Sequence

from .config import SpectrometerConfig


@dataclass
class SpectrumData:
    """
    Represents one acquired spectrum from the spectrometer.

    It keeps a snapshot of:
    - acquisition time
    - configuration that was active for this measurement
    - pixel indices
    - raw counts
    - optional wavelength axis (if LUT is available)
    """
    timestamp: datetime
    config: SpectrometerConfig

    pixels: List[int]
    counts: List[int]
    wavelengths: Optional[List[float]]  # None if LUT is not available

    @property
    def device_index(self) -> int:
        return self.config.device_index

    @property
    def exposure_ms(self) -> float:
        return self.config.exposure_ms

    @property
    def average(self) -> int:
        return self.config.average

    @property
    def dark_subtraction(self) -> int:
        return self.config.dark_subtraction

    @property
    def has_wavelengths(self) -> bool:
        return self.wavelengths is not None

    def __len__(self) -> int:
        return len(self.counts)

    @classmethod
    def from_raw(
        cls,
        counts: Sequence[int],
        wavelengths: Optional[Sequence[float]],
        config: SpectrometerConfig,
    ) -> "SpectrumData":
        pixels = list(range(len(counts)))

        if wavelengths is None:
            wl_list: Optional[List[float]] = None
        else:
            wl_list = list(wavelengths)

        return cls(
            timestamp=datetime.now(),
            config=config,
            pixels=pixels,
            counts=list(counts),
            wavelengths=wl_list,
        )
