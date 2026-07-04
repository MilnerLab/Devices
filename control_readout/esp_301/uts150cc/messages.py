"""IPC messages for the UTS150CC linear stage worker (positions in mm)."""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, OKReply, Reply, Request


@register
@dataclass(frozen=True)
class MoveUTS150CCTo(Request[OKReply]):
    position: float = 0.0  # mm


@register
@dataclass(frozen=True)
class HomeUTS150CC(Request[OKReply]):
    pass


@register
@dataclass(frozen=True)
class UTS150CCPosUpdate(Message):
    """Spontaneous position push (no request_id) — sent after moves/home."""
    position: float = 0.0  # mm


@register
@dataclass(frozen=True)
class UTS150CCPosReply(Reply):
    """Reply to GetCurrentPosUTS150CC (carries request_id)."""
    position: float = 0.0  # mm


@register
@dataclass(frozen=True)
class GetCurrentPosUTS150CC(Request[UTS150CCPosReply]):
    pass
