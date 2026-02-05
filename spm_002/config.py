from dataclasses import asdict, dataclass
from pathlib import Path

# Repository root (…/SPM-002)
REPO_ROOT = Path(__file__).resolve().parents[1]

# Path to the 32-bit Python used for acquisition
PYTHON32_PATH = str(
    REPO_ROOT / ".venv32" / "Scripts" / "python.exe"
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
    average: int = 5
    dark_subtraction: int = 0  # 0 = off, 1 = on
    mode: int = 0              # 0 = continuous mode
    scan_delay: int = 0        # used only in certain trigger modes

    def to_json(self) -> dict:
        return asdict(self)

    @classmethod
    def from_json(cls, d: dict) -> "SpectrometerConfig":
        return cls(**{k: d[k] for k in cls.__dataclass_fields__ if k in d})
    
    def update_from_json(self, d: dict) -> None:
        for k in self.__dataclass_fields__:
            if k in d:
                setattr(self, k, d[k])