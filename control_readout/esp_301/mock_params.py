"""Motion profiles for a mocked ESP301, keyed by axis.

One dict rather than a constant per stage package, because the real ESP301 is one box
carrying all three axes on a single serial port. The mock mirrors that: one controller,
three axes, and the travel of each taken from its stage's :class:`StageSpec` so a mocked
move takes about as long as the real one and stops where the real one stops.

**Bounds come from the specs, never restated here.** They used to be written out again as
datasheet travel (``0..300``, ``0..150``) while the application commands in the stage's
post-homing frame (``-9.5..290.5``, ``-75..75``). ``MockController`` clamps to these
bounds, so every mocked scan of a negative grating position — including the acceptance run
documented in ``run_xcorr_headless.py`` — was silently pinned to 0 while the run file
recorded the position that had been *asked for*. A mock in a different coordinate frame
from the hardware is worse than no mock: it reports success for a move that never happened.
"""
from __future__ import annotations

from control_readout.base.mock_params import MockMotionProfile
from control_readout.esp_301.fms300pp import spec as fms300pp_spec
from control_readout.esp_301.mfa_cc import spec as mfa_cc_spec
from control_readout.esp_301.uts150cc import spec as uts150cc_spec

#: Axis 1 — FMS300PP, 300 mm travel.
FMS300PP_AXIS = fms300pp_spec.AXIS
#: Axis 2 — MFA-CC, 25 mm travel.
MFACC_AXIS = mfa_cc_spec.AXIS
#: Axis 3 — UTS150CC, 150 mm travel.
UTS150CC_AXIS = uts150cc_spec.AXIS


def _profile(spec, velocity: float) -> MockMotionProfile:
    """A profile bounded by the stage's real soft limits, at a plausible speed.

    Velocity is the only thing the spec does not carry: it is not a limit, it is how fast
    the operator has the axis configured to run, and it exists here purely so a mocked
    move takes about as long as the real one.
    """
    return MockMotionProfile(
        lower=spec.SPEC.limit_min,
        upper=spec.SPEC.limit_max,
        velocity=velocity,
    )


MOCK_PROFILES = {
    FMS300PP_AXIS: _profile(fms300pp_spec, velocity=20.0),
    MFACC_AXIS: _profile(mfa_cc_spec, velocity=2.5),
    UTS150CC_AXIS: _profile(uts150cc_spec, velocity=20.0),
}
