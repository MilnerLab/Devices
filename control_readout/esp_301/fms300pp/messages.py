"""IPC messages for the FMS300PP linear stage worker (positions in mm)."""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, OKReply, Reply, Request


@register
@dataclass(frozen=True)
class MoveFMS300PPTo(Request[OKReply]):
    position: float = 0.0  # mm


@register
@dataclass(frozen=True)
class HomeFMS300PP(Request[OKReply]):
    pass


@register
@dataclass(frozen=True)
class FMS300PPPosUpdate(Message):
    """Spontaneous position push (no request_id) — sent after moves/home."""
    position: float = 0.0  # mm


@register
@dataclass(frozen=True)
class FMS300PPPosReply(Reply):
    """Reply to GetCurrentPosFMS300PP (carries request_id)."""
    position: float = 0.0  # mm


@register
@dataclass(frozen=True)
class GetCurrentPosFMS300PP(Request[FMS300PPPosReply]):
    pass
