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
    #: When the acquisition *began* -- stamped before the trigger is armed, not after the
    #: transfer completes. The XCORR freshness gate admits a trace by asking whether its
    #: capture started after the stage finished moving, and only a start stamp answers
    #: that: a record captured before the move and read out after it carries an end
    #: stamp that looks perfectly fresh.
    timestamp_ns: int
    #: Sample interval in seconds, as the instrument reports it (``WFMOutpre:XINcr``).
    #: The time axis used to be derived from a configured sample rate nothing ever set,
    #: which made it a fiction at every horizontal setting but one.
    dt_s: float = 0.0
