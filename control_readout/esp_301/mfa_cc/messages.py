"""IPC messages for the MFA-CC linear stage worker (positions in mm)."""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, OKReply, Reply, Request


@register
@dataclass(frozen=True)
class MoveMFACCTo(Request[OKReply]):
    position: float = 0.0  # mm


@register
@dataclass(frozen=True)
class HomeMFACC(Request[OKReply]):
    pass


@register
@dataclass(frozen=True)
class MFACCPosUpdate(Message):
    """Spontaneous position push (no request_id) — sent after moves/home."""
    position: float = 0.0  # mm


@register
@dataclass(frozen=True)
class MFACCPosReply(Reply):
    """Reply to GetCurrentPosMFACC (carries request_id)."""
    position: float = 0.0  # mm


@register
@dataclass(frozen=True)
class GetCurrentPosMFACC(Request[MFACCPosReply]):
    pass
