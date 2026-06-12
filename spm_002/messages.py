from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import OKReply, Request
from spm_002.config import SpectrometerConfig


@register
@dataclass(frozen=True)
class SetSpectrometerConfig(Request[OKReply]):
    """Main process → subprocess: apply a new SpectrometerConfig."""
    config: SpectrometerConfig = None  # type: ignore[assignment]
