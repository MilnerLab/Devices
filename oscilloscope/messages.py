"""IPC messages for the oscilloscope worker."""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import OKReply, Request

from oscilloscope.config import ScopeConfig


@register
@dataclass(frozen=True)
class SetScopeConfig(Request[OKReply]):
    """main → worker: apply a new acquisition configuration."""

    config: ScopeConfig = None  # type: ignore[assignment]
