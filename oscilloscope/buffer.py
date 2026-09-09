from __future__ import annotations

import numpy as np

from base_core.framework.shm.buffer import SharedMemoryBuffer
from base_core.framework.shm.spec import MemorySpec


class ScopeMemorySpec(MemorySpec):
    """MemorySpec for oscilloscope traces. Shape = (channels, n_samples).

    The shape is a *ceiling*, not the record length in force. A MemorySpec is frozen and
    the segment is allocated once, before the subprocess attaches, so sizing it to the
    current ``ScopeConfig`` would make record length unchangeable for the life of the
    app. Sizing it to the largest record the instrument offers costs 640 kB and lets the
    operator edit the length between runs; each frame occupies the top-left corner of a
    slot and the reader slices it back out using the config the handle applied.
    """

    def __init__(
        self,
        name: str,
        slot_count: int = 2,
        channels: int = 2,
        n_samples: int = 20_000,
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
        """Write a (channels, n_samples) trace into the corner of ``slot``.

        Writes through a sub-view rather than ``write_slot`` so a record shorter than the
        spec's ceiling is legal. Whatever sits beyond the written region is the previous
        frame's tail; the reader never looks there because it slices to the length the
        applied config carries.
        """
        data = np.asarray(samples, dtype=np.float64)
        channels, n_samples = data.shape
        view = self.read_slot_view(slot)
        if channels > view.shape[0] or n_samples > view.shape[1]:
            raise ValueError(
                f"trace ({channels}, {n_samples}) does not fit a slot of "
                f"{view.shape} -- raise the ScopeMemorySpec ceiling")
        np.copyto(view[:channels, :n_samples], data)

    def trace(self, slot: int, channels: int, n_samples: int) -> np.ndarray:
        """A copy of the written region of ``slot``.

        Copied, not viewed: the slot is handed back to the writer the moment the reader
        acks, and a view would quietly start showing the next frame mid-calculation.
        """
        return np.array(self.read_slot_view(slot)[:channels, :n_samples])
