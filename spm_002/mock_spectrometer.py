"""Mock SPM-002 — a synthetic fringe spectrum, no DLL and no hardware.

Mirrors :class:`spm_002.spectrometer.Spectrometer` so the worker cannot tell them
apart. The real driver is Windows-only and 32-bit (it loads ``PhotonSpectr.dll`` at
import), so without this there is no way to run the spectrometer path at all on a
development machine.

**What this mock does not do.** Its fringe phase drifts on its own clock and is not
affected by the mocked rotators, which live in a different subprocess entirely. So a
mocked stabilization loop exercises the whole chain — capture, template, correction,
rotate, acknowledge — but will never converge, because nothing it commands feeds back
into what it measures. Set ``drift_rad_s=0.0`` to hold the phase still and assert a
response to a deliberate step instead.
"""
from __future__ import annotations

import logging
import time
from typing import List, Optional

import numpy as np

from base_core.quantities.enums import Prefix
from spm_002.config import SpectrometerConfig
from spm_002.mock_params import MockSpectrometerParams
from spm_002.models import SpectrumData

log = logging.getLogger(__name__)


class MockSpectrometer:
    def __init__(
        self,
        config: Optional[SpectrometerConfig] = None,
        params: MockSpectrometerParams = MockSpectrometerParams(),
    ) -> None:
        self._config = config or SpectrometerConfig()
        self._params = params
        self._is_open = False
        self._rng = np.random.default_rng(params.seed)
        self._t0 = time.monotonic()
        self._wavelengths = (
            params.lambda_start_nm + params.lambda_step_nm * np.arange(params.pixels)
        )

    # -- lifecycle ---------------------------------------------------------

    @property
    def device_index(self) -> int:
        return self._config.device_index

    @property
    def is_open(self) -> bool:
        return self._is_open

    @property
    def num_pixels(self) -> int:
        return self._params.pixels

    @property
    def wavelengths(self) -> Optional[List[float]]:
        return [float(w) for w in self._wavelengths]

    def open(self) -> None:
        self._is_open = True
        self._t0 = time.monotonic()
        log.info("MockSpectrometer: open (no hardware, %d pixels)", self.num_pixels)

    def close(self) -> None:
        self._is_open = False

    def __enter__(self) -> "MockSpectrometer":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    # -- configuration -----------------------------------------------------

    def set_config(self, config: SpectrometerConfig) -> None:
        self._config = config

    def apply_config(self) -> None:
        return None

    def configure(self, config: Optional[SpectrometerConfig] = None) -> None:
        if config is not None:
            self.set_config(config)
        self.apply_config()

    # -- acquisition -------------------------------------------------------

    def acquire_spectrum(self) -> SpectrumData:
        p = self._params
        cfg = self._config
        lam = self._wavelengths

        envelope = p.amplitude * np.exp(
            -0.5 * ((lam - p.center_nm) / max(p.width_nm, 1e-6)) ** 2)

        # 2*pi*OPD/lambda, with both in the same units. The phase runs fastest at the
        # blue end, which is what puts more fringes on the short-wavelength side.
        phase = 2.0 * np.pi * (p.opd_um * 1000.0) / lam + self._phase_now()
        fringes = 1.0 + p.visibility * np.cos(phase)

        exposure_ms = self._exposure_ms()
        counts = p.dark + envelope * fringes * (exposure_ms / p.reference_exposure_ms)

        # Shot-noise-like: grows as the square root of the signal, and averaging N
        # frames cuts it by root N, so `average` behaves the way the real device does.
        average = max(int(cfg.average or 1), 1)
        counts = counts + self._rng.normal(0.0, np.sqrt(np.maximum(counts, 1.0) / average))

        if cfg.dark_subtraction:
            counts = counts - p.dark

        counts = np.clip(counts, 0, p.full_scale)
        return SpectrumData(
            counts=[int(c) for c in counts],
            wavelengths=self.wavelengths,
            timestamp_ns=time.time_ns(),
        )

    def _phase_now(self) -> float:
        return self._params.drift_rad_s * (time.monotonic() - self._t0)

    def _exposure_ms(self) -> float:
        # Read exactly the way the real driver does when it calls PHO_SetTime.
        return float(self._config.exposure_time.value(Prefix.MILLI))
