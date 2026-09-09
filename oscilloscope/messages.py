"""IPC messages for the oscilloscope worker."""
from __future__ import annotations

from dataclasses import dataclass, field

from base_core.ipc.codec import register
from base_core.ipc.message import OKReply, Reply, Request

from oscilloscope.config import ScopeConfig


@register
@dataclass(frozen=True)
class SetScopeConfig(Request[OKReply]):
    """main → worker: apply a new acquisition configuration."""

    config: ScopeConfig = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class AcquirePointReply(Reply):
    """Per-trace scalars for one measurement point (D3 step 1).

    Reduced in the subprocess on purpose. One point is ``n_traces`` records of
    ``n_samples``, and sending those across the pipe to average them in the main
    process would put megabytes through a JSON codec for a handful of floats. The
    across-trace average is the caller's to take.
    """

    #: Positive-mean of each trace.
    values: list = field(default_factory=list)
    #: How many samples went into each of those means.
    counts: list = field(default_factory=list)


@register
@dataclass(frozen=True)
class AcquirePoint(Request[AcquirePointReply]):
    """main → worker: acquire ``n_traces`` freshness-gated traces and reduce them.

    ``discard`` drops that many traces first. The scope's buffer can still hold a
    record captured before the stage finished moving, and averaging that in silently
    biases the point toward the previous position.
    """

    n_traces: int = 1
    channel: int = 1
    discard: int = 0
    probe_mm: float = 0.0


@register
@dataclass(frozen=True)
class AcquireTraceReply(Reply):
    """One raw trace, plus the reduction that would have been taken from it."""

    samples: list = field(default_factory=list)
    dt_s: float = 0.0
    v_mean_pos: float = 0.0
    n_positive: int = 0


@register
@dataclass(frozen=True)
class AcquireTrace(Request[AcquireTraceReply]):
    """main → worker: fetch one raw trace for live display.

    The one path allowed to carry bulk samples across IPC, and only because it is
    driven by a human looking at an alignment view while a run is parked at a step
    gate. It is not on any acquisition path.
    """

    channel: int = 1
