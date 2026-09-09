"""Shape of the synthetic scope trace. Subprocess-side only; never crosses IPC.

These used to be six ``mock_*`` fields on ``ScopeConfig``, which meant a dataclass
describing a Tektronix TBS2012C also described a fake chirp. They describe the mock,
so they live with the mock.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MockScopeParams:
    """An envelope-bounded chirped sinusoid — the shape the XCORR analysis expects."""

    #: Envelope centre and width, as fractions of the record.
    center_frac: float = 0.5
    width_frac: float = 0.18
    #: Fringe count across the record.
    chirp: float = 9.0
    phase0: float = 0.3
    noise: float = 0.01
    #: None draws a fresh stream each run; set it to make a run reproducible.
    seed: int | None = None
