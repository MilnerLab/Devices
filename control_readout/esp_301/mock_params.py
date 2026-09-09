"""Motion profiles for a mocked ESP301, keyed by axis.

One dict rather than a constant per stage package, because the real ESP301 is one box
carrying all three axes on a single serial port. The mock mirrors that: one controller,
three axes, and the travel of each taken from its stage's datasheet so a mocked move
takes about as long as the real one and stops where the real one stops.
"""
from __future__ import annotations

from control_readout.base.mock_params import MockMotionProfile

#: Axis 1 — FMS300PP, 300 mm travel.
FMS300PP_AXIS = 1
#: Axis 2 — MFA-CC, 25 mm travel.
MFACC_AXIS = 2
#: Axis 3 — UTS150CC, 150 mm travel.
UTS150CC_AXIS = 3

MOCK_PROFILES = {
    FMS300PP_AXIS: MockMotionProfile(lower=0.0, upper=300.0, velocity=20.0),
    MFACC_AXIS: MockMotionProfile(lower=0.0, upper=25.0, velocity=2.5),
    UTS150CC_AXIS: MockMotionProfile(lower=0.0, upper=150.0, velocity=20.0),
}
