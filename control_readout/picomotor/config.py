from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PicomotorConfig:
    """Newport 8742 picomotor controller (Ethernet). Mirror tip/tilt, manual (no PID).

    Hardware only. Whether a mock stands in for the controller is a start-time
    decision carried by ``ConnectionMode``, not a property of the instrument.
    """

    host: str = "10.1.137.239"
    axes: tuple[int, ...] = (1, 2, 3, 4)


@dataclass(frozen=True)
class MirrorAxes:
    """Which two motors tip and tilt one mirror.

    The mapping is declared rather than inferred because getting it wrong costs an
    alignment session: the operator turns what they believe is pitch, the beam walks
    in yaw, and the error is only obvious once the alignment is already lost.
    """

    name: str
    #: Left/right on the arrow pad.
    yaw_axis: int
    #: Up/down on the arrow pad.
    pitch_axis: int
    #: Called out in the UI. See DEFAULT_MIRRORS for why exactly one axis carries this.
    critical: bool = False


#: The rig's two mirrors, covering all four motors exactly once.
#:
#: Motor 3 is flagged: it is the yaw axis behind the stage walk-off, so it is the one
#: that must not be confused for its neighbour.
DEFAULT_MIRRORS: tuple[MirrorAxes, ...] = (
    MirrorAxes(name="Mirror 1", yaw_axis=1, pitch_axis=2),
    MirrorAxes(name="Mirror 2", yaw_axis=3, pitch_axis=4, critical=True),
)
