from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, Request, OKReply, Reply
from base_core.math.models import Angle


@register
@dataclass(frozen=True)
class RotateELL14(Request[OKReply]):
    angle: Angle = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class HomeELL14Rotator(Request[OKReply]):
    pass


@register
@dataclass(frozen=True)
class CurrentELL14Position(Message):
    """Spontaneous angle push (no request_id) — sent after rotate/home."""
    angle: Angle = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class ELL14PositionReply(Reply):
    """Reply to GetCurrentELL14Position (carries request_id)."""
    angle: Angle = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class GetCurrentELL14Position(Request[ELL14PositionReply]):
    pass
