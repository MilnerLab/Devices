from __future__ import annotations

from abc import ABC, abstractmethod

from camera.base.config import CameraConfig
from camera.base.models import FrameData


class Camera(ABC):
    """
    Vendor-agnostic camera interface.

    Responsibilities of a concrete subclass (e.g. VmiCamera):
    - open/close the device
    - apply a CameraConfig (exposure, gain, ROI, pixel format) to the device
      and start free-running acquisition
    - acquire single frames and return FrameData objects

    A concrete subclass does NOT:
    - handle multiple devices
    - do any GUI or plotting
    - know about shared memory or IPC -- CameraWorker is the only caller
    """

    def __init__(self, config: CameraConfig) -> None:
        self.config: CameraConfig = config

    @property
    @abstractmethod
    def is_open(self) -> bool:
        ...

    @abstractmethod
    def open(self) -> None:
        """Open the hardware connection."""
        ...

    @abstractmethod
    def apply_config(self) -> None:
        """Push self.config to the device and start free-running acquisition."""
        ...

    def set_config(self, config: CameraConfig) -> None:
        """Replace the current configuration object. Does not touch the device."""
        self.config = config

    def configure(self, config: CameraConfig | None = None) -> None:
        """Convenience method: optionally set a new config, then apply it."""
        if config is not None:
            self.set_config(config)
        self.apply_config()

    @abstractmethod
    def acquire_frame(self) -> FrameData:
        """Block for one frame and return it."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Close the device. Safe to call multiple times."""
        ...
