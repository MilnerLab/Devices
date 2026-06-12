from dataclasses import asdict, dataclass
from pathlib import Path

from base_core.framework.serialization.serde import Primitive, PrimitiveSerde


# Repository root (…/SPM-002)
REPO_ROOT = Path(__file__).resolve().parents[1]

# Path to the 32-bit Python used for acquisition
PYTHON32_PATH = str(
    REPO_ROOT / ".venv32" / "Scripts" / "python.exe"
)


@dataclass
class SpectrometerConfig(PrimitiveSerde):
    """
    Configuration for the spectrometer.

    This object is purely a data container. The Spectrometer class is
    responsible for applying these settings to the actual hardware.

    It implements PrimitiveSerde so it can travel inside a Message payload
    (e.g. SetConfig) across the JSONL pipe, the same way SharedRingBufferSpec
    rides inside ConfigureBuffer.
    """
    device_index: int = 0
    exposure_ms: float = 50.0
    average: int = 5
    dark_subtraction: int = 0  # 0 = off, 1 = on
    mode: int = 0              # 0 = continuous mode
    scan_delay: int = 0        # used only in certain trigger modes

    # ----- PrimitiveSerde (wire) -----

    def to_primitive(self) -> Primitive:
        return asdict(self)

    @classmethod
    def from_primitive(cls, v: Primitive) -> "SpectrometerConfig":
        return cls(**{k: v[k] for k in cls.__dataclass_fields__ if k in v})


