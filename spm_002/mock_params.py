"""Shape of the synthetic spectrum. Subprocess-side only; never crosses IPC."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MockSpectrometerParams:
    """A source envelope modulated by interference fringes.

    Fringes rather than a bare Gaussian, because everything downstream fits them: the
    stabilization tracker, the fringe fit and the phase template all read fringe phase.
    A smooth mock spectrum would exercise the plumbing and none of the analysis.
    """

    #: Must match the real SPM-002, because the shared-memory slot is sized for it:
    #: SpectrumMemorySpec bakes in shape (2, 3648). A mock with a different count
    #: cannot be written to the buffer at all. The worker overrides this from the
    #: attached buffer where it can, so the two cannot drift apart again.
    pixels: int = 3648
    #: Wavelength axis, nm. Roughly the SPM-002's range, in the cubic-LUT shape the
    #: real driver builds from PHO_GetLut.
    lambda_start_nm: float = 350.0
    lambda_step_nm: float = 0.1783   # 3648 px ≈ 350–1000 nm

    #: Source envelope: centre and width in nm.
    center_nm: float = 800.0
    width_nm: float = 40.0
    #: Peak counts at the reference exposure, before gain.
    amplitude: float = 20000.0

    #: Optical path difference in µm. Sets how many fringes fall under the envelope.
    opd_um: float = 30.0
    #: Fringe visibility, 0 to 1.
    visibility: float = 0.6

    #: Baseline counts present even in darkness.
    dark: float = 300.0
    #: Reference exposure, ms. Counts scale against this.
    reference_exposure_ms: float = 50.0

    #: Fringe phase drift, rad/s. Gives the stabilization loop something to chase.
    #: Set to 0.0 in a test that wants to assert a response to a *step* rather than
    #: watch the loop chase a ramp it can never catch.
    drift_rad_s: float = 0.15
    #: None draws a fresh stream each run; set it to make a run reproducible.
    seed: int | None = None

    #: Full-scale counts. The real path returns c_ushort, so the mock clips the same.
    full_scale: int = 65535
