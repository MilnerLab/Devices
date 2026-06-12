from __future__ import annotations

import logging
import threading
import time
from typing import Callable

import numpy as np

from base_core.framework.events.event_bus import EventBus
from base_core.ipc.subprocess_connector import SubprocessPipelineConnector
from base_core.ipc.worker import BaseWorker
from spm_002.buffer import SpectrumBuffer
from spm_002.config import SpectrometerConfig
from spm_002.messages import SetSpectrometerConfig
from spm_002.spectrometer import Spectrometer

log = logging.getLogger(__name__)

WORKER_ID = "spectrometer"


class SpectrometerWorker(BaseWorker):
    """
    Runs the acquisition loop inside the spectrometer subprocess.

    On StartWorker: opens the hardware, applies config, starts the acquisition thread.
    On StopWorker:  stops the acquisition thread, closes the hardware.
    On ResetWorker: stop + start.
    On SetSpectrometerConfig: applies new settings (live while running or buffered for next start).

    Injected callables allow the worker to interact with the WriterSubprocessMain
    slot-grant machinery without holding a direct reference to the subprocess main.
    """

    def __init__(
        self,
        bus: EventBus,
        connector: SubprocessPipelineConnector,
        config: SpectrometerConfig,
        get_slot: Callable[[], int | None],
        get_buffer: Callable[[], SpectrumBuffer],
        notify_written: Callable[[int, int, int], None],
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = config
        self._get_slot = get_slot
        self._get_buffer = get_buffer
        self._notify_written = notify_written
        self._spectrometer: Spectrometer | None = None
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._item_id = 0

    def _setup(self) -> None:
        self._unsubs.append(
            self._bus.subscribe(SetSpectrometerConfig, self._on_set_config)
        )

    def _start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            log.warning("SpectrometerWorker: _start() called while already running")
            return
        self._spectrometer = Spectrometer(self._config)
        self._spectrometer.open()
        self._spectrometer.apply_config()
        self._running.set()
        self._thread = threading.Thread(
            target=self._acquire_loop,
            name="spectrometer-acquire",
            daemon=True,
        )
        self._thread.start()
        log.debug("SpectrometerWorker: started acquisition thread")

    def _stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                log.warning("SpectrometerWorker: acquisition thread did not stop in time")
            self._thread = None
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

    def _acquire_loop(self) -> None:
        """Continuously acquire spectra and write them to shared memory."""
        while self._running.is_set():
            spectrometer = self._spectrometer
            if spectrometer is None:
                break

            slot = self._get_slot()
            if slot is None:
                time.sleep(0.001)
                continue

            try:
                data = spectrometer.acquire_spectrum()

                if data.wavelengths is not None:
                    wavelengths = np.array(data.wavelengths, dtype=np.float64)
                else:
                    wavelengths = np.arange(len(data.counts), dtype=np.float64)

                intensities = np.array(data.counts, dtype=np.float64)

                self._get_buffer().write_spectrum(slot, wavelengths, intensities)

                self._item_id += 1
                self._notify_written(slot, self._item_id, data.timestamp_ns)

            except Exception:
                log.exception("SpectrometerWorker: acquisition error — stopping loop")
                self._running.clear()
