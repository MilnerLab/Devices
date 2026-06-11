from __future__ import annotations

import threading

import numpy as np

from spm_002.config import SpectrometerConfig
from spm_002.messages import SetSpectrometerConfig
from spm_002.models import SpectrumData
from spm_002.shared_spectrum_buffer import SharedSpectrumBuffer
from spm_002.spectrometer import Spectrometer

from base_core.framework.subprocess.shared_memory.shared_memory_base_messages import base_registry
from base_core.framework.subprocess.shared_memory.models import (
    ItemDescriptor,
    SharedRingBufferSpec,
)
from base_core.framework.subprocess.worker import ProducerWorker
from messages import Message

class SpectrometerWorker(ProducerWorker[SharedSpectrumBuffer, SpectrumData]):
    name = "spectrometer"
    buffer_id = "spectrometer"
    messages = [SetSpectrometerConfig]

    def __init__(self) -> None:
        super().__init__()
        self._cfg: SpectrometerConfig | None = None
        self._cfg_lock = threading.Lock()
        self._cfg_ready = threading.Event()
        self._spectrometer: Spectrometer | None = None
        self._first_write = True

    def start(self):
        if self._spectrometer is None:
                self._spectrometer = Spectrometer(config=self._cfg)
                self._spectrometer.__enter__()
        return super().start()
    # ------------------------------------------------------------------
    # Command routing (called from stdin thread by SubprocessApp)
    # ------------------------------------------------------------------

    def handle(self, msg: Message, request_id: str | None) -> None:
        if isinstance(msg, SetSpectrometerConfig):
            self._handle_set_config(msg, request_id)
        else:
            super().handle(msg, request_id)

    def _handle_set_config(
        self, msg: SetSpectrometerConfig, request_id: str | None
    ) -> None:
        with self._cfg_lock:
            previous = self._cfg
            self._cfg = msg.config
            if self._spectrometer is not None:
                try:
                    self._spectrometer.configure(self._cfg)
                except Exception:
                    self._cfg = previous
                    raise
        self._cfg_ready.set()
        self.reply_ok(request_id)

    # ------------------------------------------------------------------
    # ProducerWorker hooks
    # ------------------------------------------------------------------

    def attach_buffer(self, spec: SharedRingBufferSpec) -> SharedSpectrumBuffer:
        return SharedSpectrumBuffer.attach(spec)

    def acquire(self) -> SpectrumData | None:
        if not self._cfg_ready.wait(timeout=0.1):
            return None
        with self._cfg_lock:
            return self._spectrometer.acquire_spectrum()

    def write_to_slot(self, *, data: SpectrumData, item: ItemDescriptor) -> None:
        wavelengths: np.ndarray | None = None
        if self._first_write and data.wavelengths is not None:
            wavelengths = np.asarray(data.wavelengths, dtype=self.buffer.spec.np_dtype)
            self._first_write = False

        intensities = np.asarray(data.counts, dtype=self.buffer.spec.np_dtype)

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

    def _reset(self) -> None:
        super()._reset()
        self._cfg_ready.clear()
        self._first_write = True
        if self._spectrometer is not None:
            try:
                self._spectrometer.__exit__(None, None, None)
            except Exception:
                pass
            self._spectrometer = None

