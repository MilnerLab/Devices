from dataclasses import dataclass
from pathlib import Path

# Repository root (…/SPM-002)
REPO_ROOT = Path(__file__).resolve().parents[1]

# Path to the 32-bit Python used for acquisition
PYTHON32_PATH = str(
    REPO_ROOT / "acquisition" / ".venv32" / "Scripts" / "python.exe"
)

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
