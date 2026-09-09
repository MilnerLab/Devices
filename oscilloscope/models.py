"""Types shared by every oscilloscope driver.

``ScopeTrace`` lived in ``mock_driver`` and was imported from there by the real
PyVISA driver, which meant the real instrument could not be driven without importing
the mock. The shared type belongs to neither driver, so it lives here.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ScopeTrace:
    samples: np.ndarray   # shape (channels, n_samples)
    timestamp_ns: int
