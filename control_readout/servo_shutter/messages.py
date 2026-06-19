"""IPC messages for the servo-shutter worker."""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, OKReply, Request


@register
@dataclass(frozen=True)
class BlockArm(Request[OKReply]):
    arm: int = 0


@register
@dataclass(frozen=True)
class UnblockArm(Request[OKReply]):
    arm: int = 0


@register
@dataclass(frozen=True)
class ArmStateChanged(Message):
    arm: int = 0
    blocked: bool = False
