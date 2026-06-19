"""IPC messages for the RGV100BL rotation worker (HWP)."""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, OKReply, Request
from base_core.math.models import Angle


@register
@dataclass(frozen=True)
class RotateHwpTo(Request[OKReply]):
    angle: Angle = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class HomeHwp(Request[OKReply]):
    pass


@register
@dataclass(frozen=True)
class HwpAngleUpdate(Message):
    angle: Angle = None  # type: ignore[assignment]
