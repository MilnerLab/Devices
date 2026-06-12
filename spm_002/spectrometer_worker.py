from __future__ import annotations

import time
import threading

import numpy as np

from spm_002.config import SpectrometerConfig
from spm_002.messages import Message, SetSpectrometerConfig
from spm_002.models import SpectrumData
from spm_002.shared_spectrum_buffer import SharedSpectrumBuffer
from spm_002.spectrometer import Spectrometer

from base_core.framework.subprocess.shared_memory.shared_memory_base_messages import (
    ItemWritten,
    SlotGranted,
)
from base_core.framework.subprocess.worker import Worker


class SpectrometerWorker(Worker):
    """
    Worker that continuously acquires spectra and writes them to a shared ring buffer.

    Because acquisition is hardware-driven (the spectrometer blocks until data
    is ready), this worker overrides run() with an acquisition loop that
    interleaves queue draining (for control messages) with hardware polling.

    Protocol
    --------
    Main process sends SlotGranted to grant a writable slot. Worker acquires
    a spectrum, writes it to that slot, emits ItemWritten, and replies OK.
    The SlotGranted reply_ok triggers the main process to grant the next slot.
    """

    name = "spectrometer"
    bus_messages = (SlotGranted, SetSpectrometerConfig)
    write_buffer_cls = SharedSpectrumBuffer
    buffer_id = "spectrometer"

    def __init__(self) -> None:
        super().__init__()
        self._cfg: SpectrometerConfig | None = None
        self._cfg_lock = threading.Lock()
        self._spectrometer: Spectrometer | None = None
        self._first_write = True
        # Pending grants stored separately from the queue so the acquisition
        # loop can check them without consuming the next queue entry.
        self._pending_grants: list[SlotGranted] = []
        self._pending_grants_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        with self._cfg_lock:
            cfg = self._cfg
        if self._spectrometer is None:
            self._spectrometer = Spectrometer(config=cfg)
            self._spectrometer.__enter__()

    def close(self) -> None:
        if self._spectrometer is not None:
            try:
                self._spectrometer.__exit__(None, None, None)
            except Exception:
                pass
            self._spectrometer = None

    def _reset(self) -> None:
        super()._reset()
        self._first_write = True
        with self._pending_grants_lock:
            self._pending_grants.clear()
        if self._spectrometer is not None:
            try:
                self._spectrometer.__exit__(None, None, None)
            except Exception:
                pass
            self._spectrometer = None

    # ------------------------------------------------------------------
    # Acquisition loop (overrides base run())
    # ------------------------------------------------------------------

    def run(self) -> None:
        while not self._should_stop:
            # Drain all queued control messages first (non-blocking).
            self._drain_queue()

            # Acquire if we have a pending grant.
            with self._pending_grants_lock:
                if not self._pending_grants:
                    # No grant yet — wait briefly for queue activity.
                    self._queue_event.wait(timeout=0.05)
                    continue
                grant = self._pending_grants[0]

            data = self._acquire()
            if data is None:
                time.sleep(0.001)
                continue

            with self._pending_grants_lock:
                self._pending_grants.pop(0)

            self._write_spectrum(grant, data)

    def _drain_queue(self) -> None:
        while self._queue:
            msg = self._queue.popleft()
            if not self._queue:
                self._queue_event.clear()
            try:
                self.handle(msg)
            except Exception as exc:
                self.reply_error(msg.request_id, str(exc))

    def _acquire(self) -> SpectrumData | None:
        if self._spectrometer is None:
            return None
        with self._cfg_lock:
            return self._spectrometer.acquire_spectrum()

    def _write_spectrum(self, grant: SlotGranted, data: SpectrumData) -> None:
        buf: SharedSpectrumBuffer = self._process_buffers[self.buffer_id]
        wavelengths: np.ndarray | None = None
        if self._first_write and data.wavelengths is not None:
            wavelengths = np.asarray(data.wavelengths, dtype=buf.spec.np_dtype)
            self._first_write = False

        intensities = np.asarray(data.counts, dtype=buf.spec.np_dtype)
        ts = time.time_ns()

        buf.write_spectrum(
            slot=grant.slot,
            intensities=intensities,
            item_id=grant.item_id,
            timestamp_ns=ts,
            wavelengths=wavelengths,
        )
        self.emit(ItemWritten(
            slot=grant.slot,
            item_id=grant.item_id,
            timestamp_ns=ts,
            buffer_id=self.buffer_id,
        ))
        self.reply_ok(grant.request_id)

    # ------------------------------------------------------------------
    # Message handling
    # ------------------------------------------------------------------

    def handle(self, msg: Message) -> None:
        if isinstance(msg, SlotGranted) and msg.buffer_id == self.buffer_id:
            with self._pending_grants_lock:
                self._pending_grants.append(msg)
        elif isinstance(msg, SetSpectrometerConfig):
            with self._cfg_lock:
                previous = self._cfg
                self._cfg = msg.config
                if self._spectrometer is not None:
                    try:
                        self._spectrometer.configure(self._cfg)
                    except Exception:
                        self._cfg = previous
                        raise
            self.reply_ok(msg.request_id)
