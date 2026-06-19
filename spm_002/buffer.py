from __future__ import annotations

import numpy as np

from base_core.framework.shm.buffer import SharedMemoryBuffer
from base_core.framework.shm.spec import MemorySpec


class SpectrumMemorySpec(MemorySpec):
    """
    MemorySpec for spectrometer shared memory.

    Bakes in shape=(2, pixel_count) and dtype="float64" so callers only need
    to supply a name (and optionally slot_count / pixel_count).

    Register in SpectrometerModule:
        c.register_instance(SpectrumMemorySpec, SpectrumMemorySpec("spectrum_spm002"))

    Resolve in any consumer module:
        spec = c.get(SpectrumMemorySpec)
    """

    def __init__(
        self,
        name: str,
        slot_count: int = 2,
        pixel_count: int = 3648,
    ) -> None:
        super().__init__(
            name=name,
            slot_count=slot_count,
            shape=(2, pixel_count),
            dtype="float64",
        )

    @property
    def pixel_count(self) -> int:
        return self.shape[1]


class SpectrumBuffer(SharedMemoryBuffer):
    """
    Shared memory buffer for spectrometer data.

    Shape: (2, pixel_count) — row 0 = wavelengths, row 1 = intensities.

    Main process (SpectrometerService):
        spec = SpectrumMemorySpec("spectrum_spm002")
        buf  = SpectrumBuffer.create(spec)

    Subprocess (SpectrometerProcess):
        self.register_buffer_class(SpectrumBuffer)     # in setup()
        buf = self.get_buffer(SpectrumBuffer)           # after AttachBuffer arrives
        slot = self.get_granted_slot(SpectrumBuffer)
        buf.write_spectrum(slot, wavelengths, intensities)
        self.notify_written(SpectrumBuffer, slot, item_id, timestamp_ns)

    Consumer (e.g. PhaseControlVM):
        buf = SpectrumBuffer.attach(spec)
        wl  = buf.wavelengths(slot)
        ins = buf.intensities(slot)
        bus.publish(SpectrumAck(slot=slot, item_id=item_id, consumer_id="phase_control"))
    """

    def write_spectrum(
        self,
        slot: int,
        wavelengths: np.ndarray,
        intensities: np.ndarray,
    ) -> None:
        """Write wavelengths (row 0) and intensities (row 1) into the given slot."""
        self.write_slot(slot, np.vstack([wavelengths, intensities]))

    def wavelengths(self, slot: int) -> np.ndarray:
        """Return a copy of the wavelengths row for the given slot."""
        return self.read_slot_copy(slot)[0]

    def intensities(self, slot: int) -> np.ndarray:
        """Return a copy of the intensities row for the given slot."""
        return self.read_slot_copy(slot)[1]
