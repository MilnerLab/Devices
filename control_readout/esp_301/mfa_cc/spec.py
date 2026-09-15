"""Identity, axis and travel of the MFA-CC linear stage.

Soft limits read live from the ESP301 (``SL?``/``SR?``) on 2026-07-19; see
XCORR_SPEC.md sec. 3.1. This is the one axis whose soft limits coincide with its
datasheet travel, because it homes to an end stop rather than to mid-travel.
"""
from __future__ import annotations

from control_readout.base.stage_spec import StageSpec

#: 1-based ESP301 axis this stage is wired to. Adjust to match the hardware.
AXIS = 2

SPEC = StageSpec(
    model="MFA-CC",
    units="mm",
    limit_min=0.0,
    limit_max=25.0,
)
