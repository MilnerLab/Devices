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
    """Start, or re-rate, free-running rotation. The sign sets the direction.

    Unlike every other command here this one has no destination: the plate turns
    until something stops it. The OK only says the controller accepted the rate.
    """
    velocity_deg_s: float = 0.0


@register
@dataclass(frozen=True)
class StopSpinRGV(Request[OKReply]):
    """Ramp a free-running plate to a stop. The settled angle follows as an update."""
    pass


@register
@dataclass(frozen=True)
class RGVSpinStateUpdate(Message):
    """Spontaneous spin-state push (no request_id).

    The worker is the authority on whether the plate is turning, because it also ends
    a spin on its own — a pause, a stop, or a position command arriving from anywhere.
    Without this push the handle would go on believing a stopped plate is still
    free-running, and keep discarding the angle read-backs it needs.
    """
    spinning: bool = False
    velocity_deg_s: float = 0.0

