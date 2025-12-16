# acquisition/spm002/config.py
from dataclasses import dataclass


@dataclass
class SpectrometerConfig:
    """
    Configuration for the spectrometer.

    This object is purely a data container. The Spectrometer class is
    responsible for applying these settings to the actual hardware.
    """
    device_index: int = 0
    exposure_ms: float = 50.0
    average: int = 1
    dark_subtraction: int = 0  # 0 = off, 1 = on
    mode: int = 0              # 0 = continuous mode
    scan_delay: int = 0        # used only in certain trigger modes
