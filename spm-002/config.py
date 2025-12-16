# acquisition/config.py
from pathlib import Path

# Repository root (…/SPM-002)
REPO_ROOT = Path(__file__).resolve().parents[1]

# Path to the 32-bit Python used for acquisition
PYTHON32_PATH = str(
    REPO_ROOT / "acquisition" / ".venv32" / "Scripts" / "python.exe"
)