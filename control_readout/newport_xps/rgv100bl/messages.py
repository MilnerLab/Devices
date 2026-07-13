"""IPC messages for the RGV100BL rotation worker (HWP)."""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, OKReply, Reply, Request
from base_core.math.models import Angle


@register
@dataclass(frozen=True)
class RotateRGVTo(Request[OKReply]):
    """Absolute move: rotate the HWP TO this angle."""
    angle: Angle = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class RotateRGVBy(Request[OKReply]):
    """Relative move: rotate the HWP BY this delta from its current angle.

    Used by the phase-stabilization loop, whose corrections are relative nudges
    (see PhaseCorrector); accumulating them via move_by is what lets the loop
    converge instead of driving to an absolute angle proportional to the error.
    """
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

