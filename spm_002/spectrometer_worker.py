from __future__ import annotations

import logging
import threading
import time

import numpy as np

from base_core.framework.events.event_bus import EventBus
from base_core.framework.shm.writer_worker import WriterWorker
from base_core.ipc.subprocess_connector import SubprocessPipelineConnector
from base_core.ipc.threaded_worker import worker_thread
from spm_002.buffer import SpectrumBuffer
from spm_002.config import SpectrometerConfig
from spm_002.messages import SetSpectrometerConfig
from spm_002.spectrometer import Spectrometer

log = logging.getLogger(__name__)

WORKER_ID = "spectrometer"


class SpectrometerWorker(WriterWorker[SpectrumBuffer]):
    """
    Runs the acquisition loop inside the spectrometer subprocess.

    On StartWorker: opens the hardware, applies config, starts the acquisition stream.
    On StopWorker:  drains the stream, closes the hardware.
    On ResetWorker: stop + start.
    On SetSpectrometerConfig: applies new settings (live while running or buffered for next start).
    """

    def __init__(
        self,
        bus: EventBus,
        connector: SubprocessPipelineConnector,
        config: SpectrometerConfig,
        get_buffer,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector, get_buffer)
        self._config = config
        self._spectrometer: Spectrometer | None = None
        self._item_id = 0

    def _setup(self) -> None:
        super()._setup()  # registers SlotGrant subscription
        self._unsubs.append(
            self._bus.subscribe(SetSpectrometerConfig, self._on_set_config)
        )

    def _start(self) -> None:
        if self._spectrometer is not None:
            log.warning("SpectrometerWorker: _start() called while already running")
            return
        self._spectrometer = Spectrometer(self._config)
        self._spectrometer.open()
        self._spectrometer.apply_config()
        self._start_producing(self._acquire_producer, on_item=self._on_acquired)
        log.debug("SpectrometerWorker: started acquisition")

    def _stop(self) -> None:
        if self._prod_handle is not None:
            fut = self._prod_handle.future
            self._stop_producing()
            try:
                fut.result(timeout=5.0)
            except Exception:
                log.warning("SpectrometerWorker: acquisition did not stop in 5 s")
        if self._spectrometer is not None:
            try:
                self._spectrometer.close()
            except Exception:
                log.exception("SpectrometerWorker: error closing device")
            self._spectrometer = None
        log.debug("SpectrometerWorker: stopped")

    def _reset(self) -> None:
        self._stop()
        self._start()

    @worker_thread
    def _on_set_config(self, msg: SetSpectrometerConfig) -> None:
        self._config = msg.config
        if self._spectrometer is not None and self._spectrometer.is_open:
            try:
                self._spectrometer.configure(self._config)
            except Exception as exc:
                log.exception("SpectrometerWorker: configure failed")
                self._reply_error(msg, str(exc))
                return
        self._reply_ok(msg)

    def _acquire_producer(self, stop: threading.Event):
        """Generator: yields (slot, wavelengths, intensities, timestamp_ns) until stopped."""
        while not stop.is_set():
            spectrometer = self._spectrometer
            if spectrometer is None:
                break
            slot = self._get_slot()
            if slot is None:
                time.sleep(0.001)
                continue
            try:
                data = spectrometer.acquire_spectrum()
                wavelengths = (
                    np.array(data.wavelengths, dtype=np.float64)
                    if data.wavelengths is not None
                    else np.arange(len(data.counts), dtype=np.float64)
                )
                intensities = np.array(data.counts, dtype=np.float64)
                yield (slot, wavelengths, intensities, data.timestamp_ns)
            except Exception:
                log.exception("SpectrometerWorker: acquisition error — stopping loop")
                return

    def _on_acquired(self, item: tuple) -> None:
        slot, wavelengths, intensities, timestamp_ns = item
        self._get_buffer().write_spectrum(slot, wavelengths, intensities)
        self._item_id += 1
        self._notify_written(slot, self._item_id, timestamp_ns)
