from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, Request, OKReply
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
    angle: Angle = None  # type: ignore[assignment]

@register
@dataclass(frozen=True)
class RequestCurrentPosition(Request[CurrentELL14Position]):
    pass
