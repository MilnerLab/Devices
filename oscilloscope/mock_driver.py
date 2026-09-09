"""
Mock oscilloscope driver — synthetic traces, no hardware (M1.D / D9).

Generates an envelope-bounded chirped sinusoid on CH1 (the same shape the XCORR
analysis expects, vs a time abscissa) and a position-like ramp on CH2 (the analog-
position-sync channel reserved in Q4). Standalone numpy so Devices has no app deps.
Mirrors :class:`oscilloscope.tbs_driver.TbsScope`.
"""
from __future__ import annotations

import time

import numpy as np

from oscilloscope.config import ScopeConfig
from oscilloscope.mock_params import MockScopeParams
from oscilloscope.models import ScopeTrace


class MockScope:
    def __init__(
        self,
        config: ScopeConfig,
        params: MockScopeParams = MockScopeParams(),
    ) -> None:
        self._config = config
        self._params = params
        self._rng = np.random.default_rng(params.seed)

    # match the real-driver lifecycle interface
    def open(self) -> None: ...
    def apply_config(self) -> None: ...
    def close(self) -> None: ...

    def acquire_trace(self) -> ScopeTrace:
        p = self._params
        n = self._config.n_samples
        started_ns = time.time_ns()
        x = np.linspace(0.0, 1.0, n)

        width = max(p.width_frac, 1e-3)
        env = np.exp(-0.5 * ((x - p.center_frac) / width) ** 2)

        dx = x - p.center_frac
        phase = p.phase0 + 2.0 * np.pi * p.chirp * dx * (1.0 + dx)  # chirped
        ch1 = env * 0.5 * (1.0 + np.cos(phase))
        if p.noise > 0.0:
            ch1 = ch1 + self._rng.normal(0.0, p.noise, size=n)

        ch2 = x  # synthetic probe-position ramp (analog-sync channel)

        rows = [ch1, ch2][: self._config.channels]
        samples = np.vstack(rows).astype(np.float64)
        # No instrument to ask, so the configured rate is the honest answer here.
        rate = self._config.sample_rate_hz
        return ScopeTrace(
            samples=samples,
            timestamp_ns=started_ns,
            dt_s=(1.0 / rate) if rate > 0 else 0.0,
        )
