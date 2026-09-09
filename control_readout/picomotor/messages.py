"""IPC messages for the picomotor worker (manual mirror tip/tilt)."""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, OKReply, Reply, Request


@register
@dataclass(frozen=True)
class StepBy(Request[OKReply]):
    """Relative move — the primary control on a manual tip/tilt mirror."""

    axis: int = 0
    steps: int = 0


@register
@dataclass(frozen=True)
class StepTo(Request[OKReply]):
    """Absolute move on the open-loop counter.

    ``steps`` is a target on the controller's own count, not a calibrated position:
    the 8742 has no encoder, so this is a convenience on top of the same counter
    ``StepBy`` advances, never a coordinate.
    """

    axis: int = 0
    steps: int = 0


@register
@dataclass(frozen=True)
class ZeroAxis(Request[OKReply]):
    """Re-reference one axis counter to zero. Moves nothing."""

    axis: int = 0


@register
@dataclass(frozen=True)
class StepsMoved(Message):
    """Open-loop step count after a move (steppers have no absolute encoder).

    Spontaneous — no ``request_id``. Pushed after every command that changes a
    counter, because the accepting reply says only that the command was taken, not
    where the axis ended up.
    """

    axis: int = 0
    total_steps: int = 0


@register
@dataclass(frozen=True)
class StepsReply(Reply):
    """Counters read back without moving, ``{axis: total_steps}``.

    The codec is JSON-backed and JSON object keys are strings, so the axes arrive on
    the far side as ``str``. The handle coerces them; nothing here can.
    """

    steps: dict = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class QuerySteps(Request[StepsReply]):
    """Read the counters without moving anything. Empty ``axes`` means all of them."""

    axes: tuple = ()
