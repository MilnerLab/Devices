"""IPC messages for the RGV100BL rotation worker (HWP)."""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, OKReply, Reply, Request
from base_core.math.models import Angle


@register
@dataclass(frozen=True)
class RotateRGVTo(Request[OKReply]):
    angle: Angle = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class HomeRGV(Request[OKReply]):
    pass


@register
@dataclass(frozen=True)
class RGVAngleUpdate(Message):
    """Spontaneous angle push (no request_id) — sent after rotate/home."""
    angle: Angle = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class RGVAngleReply(Reply):
    """Reply to GetCurrentRGVAngle (carries request_id)."""
    angle: Angle = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class GetCurrentRGVAngle(Request[RGVAngleReply]):
    pass


@register
@dataclass(frozen=True)
class SpinRGV(Request[OKReply]):
    """Start (or re-rate) continuous rotation. Sign of the velocity sets the direction.

    Sent again while already spinning, this changes the speed without stopping.
    """
    velocity_deg_s: float = 0.0


@register
@dataclass(frozen=True)
class StopSpinRGV(Request[OKReply]):
    """Ramp the spin down to a stop and report the angle it settled at."""
    pass


@register
@dataclass(frozen=True)
class RGVSpinStateUpdate(Message):
    """Spontaneous push whenever the spin starts, changes rate or stops.

    The angle read-back is deliberately NOT part of this: while spinning there is no
    stable position to report. ``RGVAngleUpdate`` follows once the stage has stopped.
    """
    spinning: bool = False
    velocity_deg_s: float = 0.0
