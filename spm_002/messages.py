from __future__ import annotations

from dataclasses import dataclass

from messages import Message, Kind
from spm_002.config import SpectrometerConfig


@dataclass(frozen=True)
class SetSpectrometerConfig(Message):
    """main → subprocess: apply a new acquisition configuration."""
    NAME = "set_spectrometer_config"
    KIND = Kind.COMMAND
    config: SpectrometerConfig
