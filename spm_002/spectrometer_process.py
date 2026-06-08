from __future__ import annotations

import threading

import numpy as np

from spm_002.config import SpectrometerConfig
from spm_002.messages import SetSpectrometerConfig
from spm_002.models import SpectrumData
from spm_002.shared_spectrum_buffer import SharedSpectrumBuffer
from spm_002.spectrometer import Spectrometer

from base_core.framework.subprocess.shared_memory.base_protocol import base_registry
from base_core.framework.subprocess.shared_memory.models import (
    ItemDescriptor,
    SharedRingBufferSpec,
)
from base_core.framework.subprocess.granted_slot_writer_process_base import (
    GrantedSlotWriterProcessBase,
)


class SpectrometerSubprocess(
    GrantedSlotWriterProcessBase[SharedSpectrumBuffer, SpectrumData]
):
    def __init__(self) -> None:
        registry = base_registry().extend(SetSpectrometerConfig)
        super().__init__(registry, source="spectrometer", buffer_id="spectrometer")

        self._cfg: SpectrometerConfig | None = None
        self._cfg_lock = threading.Lock()
        self._cfg_ready = threading.Event()

        self._spectrometer: Spectrometer | None = None
        self._first_write = True

        self.on(SetSpectrometerConfig, self._handle_set_config)

    # ------------------------------------------------------------------
    # Config handling
    # ------------------------------------------------------------------

    def _handle_set_config(
        self, msg: SetSpectrometerConfig, envelope: dict
    ) -> None:
        with self._cfg_lock:
            self._cfg = msg.config
            if self._spectrometer is not None:
                self._spectrometer.configure(self._cfg)
        self._cfg_ready.set()
        self.reply_ok(envelope)

    # ------------------------------------------------------------------
    # GrantedSlotWriterProcessBase hooks
    # ------------------------------------------------------------------

    def attach_buffer(self, spec: SharedRingBufferSpec) -> SharedSpectrumBuffer:
        return SharedSpectrumBuffer.attach(spec)

    def acquire_measurement(
        self, stop_event: threading.Event
    ) -> SpectrumData | None:
        if not self._cfg_ready.is_set():
            return None

        with self._cfg_lock:
            if self._spectrometer is None:
                self._spectrometer = Spectrometer(config=self._cfg)
                self._spectrometer.__enter__()
            return self._spectrometer.acquire_spectrum()

    def write_measurement_to_slot(
        self, *, measurement: SpectrumData, item: ItemDescriptor
    ) -> None:
        wavelengths: np.ndarray | None = None
        if self._first_write and measurement.wavelengths is not None:
            wavelengths = np.asarray(
                measurement.wavelengths, dtype=self.buffer.spec.np_dtype
            )
            self._first_write = False

        intensities = np.asarray(measurement.counts, dtype=self.buffer.spec.np_dtype)

        self.buffer.write_spectrum(
            slot=item.slot,
            intensities=intensities,
            item_id=item.item_id,
            timestamp_ns=item.timestamp_ns,
            wavelengths=wavelengths,
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def main(self, stop_event: threading.Event) -> None:
        try:
            super().main(stop_event)
        finally:
            if self._spectrometer is not None:
                self._spectrometer.__exit__(None, None, None)
                self._spectrometer = None


if __name__ == "__main__":
    SpectrometerSubprocess().run()
