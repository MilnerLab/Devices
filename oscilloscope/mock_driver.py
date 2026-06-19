"""
Mock oscilloscope driver — synthetic traces, no hardware (M1.D / D9).

Generates an envelope-bounded chirped sinusoid on CH1 (the same shape the XCORR
analysis expects, vs a time abscissa) and a position-like ramp on CH2 (the analog-
position-sync channel reserved in Q4). Standalone numpy so Devices has no app deps.
Swap for :mod:`oscilloscope.tbs_driver` (PyVISA/SCPI) when the TBS2012C is on the bench.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from oscilloscope.config import ScopeConfig


@dataclass(frozen=True)
class ScopeTrace:
    samples: np.ndarray   # shape (channels, n_samples)
    timestamp_ns: int


class MockScope:
    def __init__(self, config: ScopeConfig) -> None:
        self._config = config
        self._rng = np.random.default_rng(config.mock_seed)

    # match the real-driver lifecycle interface
    def open(self) -> None: ...
    def apply_config(self) -> None: ...
    def close(self) -> None: ...

    def acquire_trace(self) -> ScopeTrace:
        cfg = self._config
        n = cfg.n_samples
        x = np.linspace(0.0, 1.0, n)

        center = cfg.mock_center_frac
        width = max(cfg.mock_width_frac, 1e-3)
        env = np.exp(-0.5 * ((x - center) / width) ** 2)

        dx = x - center
        phase = cfg.mock_phase0 + 2.0 * np.pi * cfg.mock_chirp * dx * (1.0 + dx)  # chirped
        ch1 = env * 0.5 * (1.0 + np.cos(phase))
        if cfg.mock_noise > 0.0:
            ch1 = ch1 + self._rng.normal(0.0, cfg.mock_noise, size=n)

        ch2 = x  # synthetic probe-position ramp (analog-sync channel)

        rows = [ch1, ch2][: cfg.channels]
        samples = np.vstack(rows).astype(np.float64)
        return ScopeTrace(samples=samples, timestamp_ns=time.time_ns())
