"""Identity, axis and travel of the FMS300PP linear stage.

Soft limits read live from the ESP301 (``SL?``/``SR?``) on 2026-07-19; see
XCORR_SPEC.md sec. 3.1. They are the post-homing frame the application commands in, not
the 0..300 mm datasheet travel — the stage homes to a mid-travel origin, so its legal
range starts negative.
"""
from __future__ import annotations

from control_readout.base.stage_spec import StageSpec

#: 1-based ESP301 axis this stage is wired to. Adjust to match the hardware.
AXIS = 1

SPEC = StageSpec(
    model="FMS300PP",
    units="mm",
    limit_min=-9.5,
    limit_max=290.5,
)
