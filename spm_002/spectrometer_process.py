from __future__ import annotations

import threading
from typing import Any

import numpy as np

from spm_002.config import SpectrometerConfig
from spm_002.models import SpectrumData
from spm_002.shared_spectrum_buffer import SharedSpectrumBuffer, SharedSpectrumBufferProtocol
from spm_002.spectrometer import Spectrometer

from base_core.framework.subprocess.shared_memory.models import (
    FrameDescriptor,
    SharedRingBufferSpec,
)
from base_core.framework.subprocess.writer_process_base import GrantedSlotWriterProcessBase


class SpectrometerSubprocess(
    GrantedSlotWriterProcessBase[SharedSpectrumBuffer, SpectrumData]
):
    PROTOCOL = SharedSpectrumBufferProtocol

    def __init__(self) -> None:
        super().__init__(source="spectrometer")

        self._cfg = SpectrometerConfig()
        self._cfg_lock = threading.Lock()
        self._cfg_ready = threading.Event()
        self._cfg_dirty = threading.Event()

        self._spectrometer: Spectrometer | None = None
        self._first_write = True

    # ------------------------------------------------------------------
    # config handling
    # ------------------------------------------------------------------
    def _cfg_snapshot(self) -> SpectrometerConfig:
        with self._cfg_lock:
            return SpectrometerConfig.from_json(self._cfg.to_json())

    def on_command(
        self,
        name: str,
        payload: dict[str, Any],
        message: dict[str, Any],
    ) -> None:
        # device-specific commands first
        if name == "shutdown":
            self.reply_ok(message)
            self.stop()
            return

        if name == "set_config":
            with self._cfg_lock:
                self._cfg.update_from_json(payload)
            self._cfg_ready.set()
            self._cfg_dirty.set()
            self.reply_ok(message)
            return

        # shared-buffer commands are handled by the base class
        super().on_command(name, payload, message)

    # ------------------------------------------------------------------
    # shared buffer
    # ------------------------------------------------------------------
    def attach_buffer(self, spec: SharedRingBufferSpec) -> SharedSpectrumBuffer:
        return SharedSpectrumBuffer.attach(spec)

    # ------------------------------------------------------------------
    # acquisition
    # ------------------------------------------------------------------
    def on_buffer_configured(self) -> None:
        # optional hook; nothing special needed here
        pass

    def acquire_measurement(
        self,
        stop_event: threading.Event,
    ) -> SpectrumData | None:
        """
        Called by the base class when a writable slot is available.
        We wait for config, lazily open/configure the device, then acquire one spectrum.
        """
        if not self._cfg_ready.is_set():
            return None

        if self._spectrometer is None:
            self._spectrometer = Spectrometer(config=self._cfg_snapshot())
            self._spectrometer.__enter__()

        if self._cfg_dirty.is_set():
            self._cfg_dirty.clear()
            self._spectrometer.configure(self._cfg_snapshot())

        return self._spectrometer.acquire_spectrum()

    def write_measurement_to_slot(
        self,
        *,
        measurement: SpectrumData,
        frame: FrameDescriptor,
    ) -> None:
        wavelengths = None
        if self._first_write:
            wavelengths = np.asarray(measurement.wavelengths, dtype=self.buffer.spec.np_dtype)

        intensities = np.asarray(measurement.counts, dtype=self.buffer.spec.np_dtype)

        self.buffer.write_spectrum(
            slot=frame.slot,
            wavelengths=wavelengths,
            intensities=intensities,
            frame_id=frame.frame_id,
            timestamp_ns=frame.timestamp_ns,
        )

        self._first_write = False

    # ------------------------------------------------------------------
    # lifecycle
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