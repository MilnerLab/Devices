from __future__ import annotations

import numpy as np

from base_core.framework.shm.buffer import SharedMemoryBuffer
from base_core.framework.shm.spec import MemorySpec


class ScopeMemorySpec(MemorySpec):
    """MemorySpec for oscilloscope traces. Shape = (channels, n_samples)."""

    def __init__(
        self,
        name: str,
        slot_count: int = 8,
        channels: int = 2,
        n_samples: int = 2000,
    ) -> None:
        super().__init__(
            name=name,
            slot_count=slot_count,
            shape=(channels, n_samples),
            dtype="float64",
        )

    @property
    def channels(self) -> int:
        return self.shape[0]

    @property
    def n_samples(self) -> int:
        return self.shape[1]


class ScopeBuffer(SharedMemoryBuffer):
    """Shared-memory buffer for scope traces; row c = channel c's samples."""

    def write_trace(self, slot: int, samples: np.ndarray) -> None:
        self.write_slot(slot, np.asarray(samples, dtype=np.float64))

    def read_trace(self, slot: int) -> np.ndarray:
        return self.read_slot_copy(slot)
