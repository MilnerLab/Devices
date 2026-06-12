from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, Request, OKReply
from base_core.math.models import Angle


@register
@dataclass(frozen=True)
class Rotate(Request[OKReply]):
    angle: Angle = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class HomeRotator(Request[OKReply]):
    pass


@register
@dataclass(frozen=True)
class RotatorMoved(Message):
    angle: Angle = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class RotatorHomed(Message):
    pass
