"""IPC messages for the picomotor worker (manual mirror tip/tilt)."""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, OKReply, Request


@register
@dataclass(frozen=True)
class StepBy(Request[OKReply]):
    axis: int = 0
    steps: int = 0


@register
@dataclass(frozen=True)
class StepsMoved(Message):
    """Open-loop step count after a move (steppers have no absolute encoder)."""

    axis: int = 0
    total_steps: int = 0
