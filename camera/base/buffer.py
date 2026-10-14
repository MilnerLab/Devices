from __future__ import annotations

import numpy as np

from base_core.framework.shm.buffer import SharedMemoryBuffer
from base_core.framework.shm.spec import MemorySpec
from camera.base.config import CameraConfig


#: Frame dtype for each pixel format a CameraConfig may name. Mono10/Mono12 arrive unpacked
#: in 16-bit words, so they share Mono16's dtype. Anything not listed here is rejected by
#: CameraMemorySpec.from_config rather than guessed at.
PIXEL_FORMAT_DTYPES: dict[str, str] = {
    "Mono8": "uint8",
    "Mono10": "uint16",
    "Mono12": "uint16",
    "Mono16": "uint16",
}


class CameraMemorySpec(MemorySpec):
    """
    MemorySpec for camera shared memory. Vendor-agnostic: shape/dtype come from
    the caller's CameraConfig (width/height/pixel_format), not baked in per vendor.

    Register in a camera module:
        spec = CameraMemorySpec("camera_vmi_frame", width=config.width, height=config.height)
    """

    def __init__(
        self,
        name: str,
        slot_count: int = 2,
        width: int = 1224,
        height: int = 1024,
        dtype: str = "uint8",
    ) -> None:
        super().__init__(
            name=name,
            slot_count=slot_count,
            shape=(height, width),
            dtype=dtype,
        )

    @classmethod
    def from_config(cls, name: str, config: CameraConfig, slot_count: int = 2) -> CameraMemorySpec:
        """The spec whose frames fit what ``config`` makes the camera produce."""
        try:
            dtype = PIXEL_FORMAT_DTYPES[config.pixel_format]
        except KeyError:
            raise ValueError(
                f"Unsupported pixel format {config.pixel_format!r}; "
                f"expected one of {sorted(PIXEL_FORMAT_DTYPES)}"
            ) from None
        return cls(name, slot_count, width=config.width, height=config.height, dtype=dtype)

    def matches(self, config: CameraConfig) -> bool:
        """True when frames produced under ``config`` fit this spec as-is."""
        return (
            self.shape == (config.height, config.width)
            and self.dtype == PIXEL_FORMAT_DTYPES.get(config.pixel_format)
        )

    @property
    def width(self) -> int:
        return self.shape[1]

    @property
    def height(self) -> int:
        return self.shape[0]


class CameraBuffer(SharedMemoryBuffer):
    """
    Shared memory buffer for a single camera's frames.

    Main process (CameraService, via CameraWorkerHandle):
        spec = CameraMemorySpec(name, width=..., height=...)
        buf  = CameraBuffer.create(spec)

    Subprocess (CameraWorker):
        self.register_buffer_class(CameraBuffer)     # in setup()
        buf = self.get_buffer(CameraBuffer)
        slot = self.get_granted_slot(CameraBuffer)
        buf.write_frame(slot, frame)
        self.notify_written(CameraBuffer, slot, item_id, timestamp_ns)

    Consumer:
        buf = CameraBuffer.attach(spec)
        frame = buf.frame(slot)
        bus.publish(FrameAck(slot=slot, item_id=item_id, consumer_id="..."))
    """

    def write_frame(self, slot: int, frame: np.ndarray) -> None:
        """Write a single frame into the given slot."""
        self.write_slot(slot, frame)

    def frame(self, slot: int) -> np.ndarray:
        """Return a copy of the frame for the given slot."""
        return self.read_slot_copy(slot)
