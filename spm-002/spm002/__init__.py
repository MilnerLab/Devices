# spm002/__init__.py
"""
High-level interface for the SPM-002 spectrometer using PhotonSpectr.dll.

This package only handles:
- connecting to the spectrometer
- configuring it
- acquiring spectra
"""

from .config import SpectrometerConfig
from .models import SpectrumData
from .spectrometer import Spectrometer
from .exceptions import SpectrometerError

__all__ = [
    "SpectrometerConfig",
    "SpectrumData",
    "Spectrometer",
    "SpectrometerError",
]
