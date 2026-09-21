from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import OKReply, Request
from camera.base.config import CameraConfig


@register
@dataclass(frozen=True)
class SetCameraConfig(Request[OKReply]):
    """Main process -> subprocess: apply a new CameraConfig. Shared by every
    camera vendor, since the config shape (exposure/gain/ROI) is shared."""
    config: CameraConfig = None  # type: ignore[assignment]
