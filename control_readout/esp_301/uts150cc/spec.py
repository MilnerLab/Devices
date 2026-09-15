"""Identity, axis and travel of the UTS150CC linear stage.

Soft limits read live from the ESP301 (``SL?``/``SR?``) on 2026-07-19; see
XCORR_SPEC.md sec. 3.1. The stage homes to the centre of its 150 mm travel, so the legal
range is symmetric about zero rather than the 0..150 mm of the datasheet.
"""
from __future__ import annotations

from control_readout.base.stage_spec import StageSpec

#: 1-based ESP301 axis this stage is wired to. Adjust to match the hardware.
AXIS = 3

SPEC = StageSpec(
    model="UTS150CC",
    units="mm",
    limit_min=-75.0,
    limit_max=75.0,
)
