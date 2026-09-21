from __future__ import annotations

import numpy as np

from base_core.framework.shm.buffer import SharedMemoryBuffer
from base_core.framework.shm.spec import MemorySpec


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
