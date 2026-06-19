"""IPC messages for the ESP301 worker (mirrors the elliptec messages pattern).

Commands are ``Request[OKReply]`` (main → worker); telemetry/events are ``Message``
(worker → main). All carry an ``axis`` field (the controller is addressed per-axis).
"""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, OKReply, Request


@register
@dataclass(frozen=True)
class MoveTo(Request[OKReply]):
    axis: int = 0
    position: float = 0.0


@register
@dataclass(frozen=True)
class MoveRelative(Request[OKReply]):
    axis: int = 0
    delta: float = 0.0


@register
@dataclass(frozen=True)
class HomeAxis(Request[OKReply]):
    axis: int = 0


@register
@dataclass(frozen=True)
class StopAxis(Request[OKReply]):
    axis: int = 0


@register
@dataclass(frozen=True)
class SetVelocity(Request[OKReply]):
    axis: int = 0
    velocity: float = 0.0


@register
@dataclass(frozen=True)
class PositionUpdate(Message):
    """Periodic true-position telemetry (poll loop, ~20 Hz)."""

    axis: int = 0
    position: float = 0.0


@register
@dataclass(frozen=True)
class MoveComplete(Message):
    """Emitted when an axis transitions from moving to motion-done."""

    axis: int = 0
    position: float = 0.0
