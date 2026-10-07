from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys 

from base_core.framework.serialization.serde import PrimitiveSerde
from base_core.quantities.enums import Prefix
from base_core.quantities.models import Time

# Repository root (…/SPM-002)
REPO_ROOT = Path(__file__).resolve().parents[1]

# Path to the .venv32 Python (32-bit on Windows for DLL; 64-bit on Linux for subprocess dev)
if sys.platform == "win32":
    PYTHON310_PATH = str(REPO_ROOT / ".venv310" / "Scripts" / "python.exe")
else:
    PYTHON310_PATH = str(REPO_ROOT / ".venv310" / "bin" / "python")


@dataclass
class CameraConfig(PrimitiveSerde):
    """
    Configuration shared by every camera vendor in this device family.

    Exposure, gain, and ROI cropping (offset_x/offset_y/width/height) are common
    enough across camera SDKs (PySpin, Thorlabs, ...) to live in one shape here,
    rather than each vendor declaring its own config dataclass. width/height also
    determine the shared-memory frame shape (see CameraMemorySpec) -- changing
    them after the buffer has been created is not supported.

    This object is purely a data container; the concrete Camera subclass is
    responsible for applying these settings to the actual hardware.
    """
    device_index: int = 0
    exposure_time: Time = Time(2000, Prefix.MICRO)
    gain: float = 25.0
    offset_x: int = 0
    offset_y: int = 0
    width: int = 1224
    height: int = 1024
    pixel_format: str = "Mono8"
    timeout_ms: int = 1000
