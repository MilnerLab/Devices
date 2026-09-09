from __future__ import annotations

import logging
import threading
import time
from dataclasses import replace

import numpy as np

from base_core.framework.events.event_bus import EventBus
from base_core.framework.shm.writer_worker import WriterWorker
from base_core.ipc.device_worker import DeviceWorkerMixin
from base_core.ipc.subprocess_connector import SubprocessPipelineConnector
from base_core.ipc.threaded_worker import worker_thread
from spm_002.buffer import SpectrumBuffer
from spm_002.config import SpectrometerConfig
from spm_002.messages import SetSpectrometerConfig

log = logging.getLogger(__name__)

WORKER_ID = "spectrometer"


class SpectrometerWorker(DeviceWorkerMixin, WriterWorker[SpectrumBuffer]):
    """
    Runs the acquisition loop inside the spectrometer subprocess.

    On StartWorker: opens the hardware, applies config, starts the acquisition stream.
    On PauseWorker: drains the stream, closes nothing (device stays open).
    On ResumeWorker: restarts the acquisition stream (device stays open).
    On StopWorker: closes the hardware.
    On SetSpectrometerConfig: applies new settings (live while running or buffered for next start).
    """

    def __init__(
        self,
        bus: EventBus,
        connector: SubprocessPipelineConnector,
        get_buffer,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector, get_buffer)
        self._config = None
        self._spectrometer = None
        self._item_id = 0

    def _setup(self) -> None:
        super()._setup()  # registers SlotGrant subscription
        self._unsubs.append(
            self._bus.subscribe(SetSpectrometerConfig, self._on_set_config)
        )

    def _start(self) -> None:
        if self._spectrometer is None:
            self._spectrometer = self._open_device()
        self._start_producing(self._acquire_producer, on_item=self._on_acquired)
        log.debug("SpectrometerWorker: started acquisition")

    def _connect(self):
        # Imported here, not at module scope: spm_002.dll loads PhotonSpectr.dll at
        # import time, so a top-level import makes the whole module unimportable on a
        # machine without the DLL — and an unimportable module cannot fall back.
        from spm_002.spectrometer import Spectrometer

        device = Spectrometer(self._config)
        device.open()
        device.apply_config()
        return device

    def _connect_mock(self):
        from spm_002.mock_params import MockSpectrometerParams
        from spm_002.mock_spectrometer import MockSpectrometer

        # Size the mock from the buffer it has to write into, rather than trusting the
        # two defaults to agree. They did not: the slot is shaped (2, 3648) for the real
        # SPM-002, and a mock of any other length fails on the first write with a numpy
        # broadcast error, several frames away from the mismatch that caused it.
        params = MockSpectrometerParams()
        pixels = self._buffer_pixel_count()
        if pixels is not None and pixels != params.pixels:
            span_nm = (params.pixels - 1) * params.lambda_step_nm
            params = replace(params, pixels=pixels,
                             lambda_step_nm=span_nm / max(pixels - 1, 1))
            log.info("MockSpectrometer: sized to the buffer's %d pixels", pixels)

        device = MockSpectrometer(self._config, params)
        device.open()
        device.apply_config()
        return device

    def _buffer_pixel_count(self) -> int | None:
        """Pixels per slot, or None if the buffer has not been attached yet."""
        try:
            return int(self._get_buffer().spec.shape[1])
        except Exception:
            return None

    def _pause(self) -> None:
        handle = self._stop_producing()
        if handle is not None:
            handle.wait(timeout=5.0)
            if not handle.done_event.is_set():
                log.warning("SpectrometerWorker: acquisition did not stop in 5 s")

    def _resume(self) -> None:
        self._start_producing(self._acquire_producer, on_item=self._on_acquired)
        log.debug("SpectrometerWorker: resumed acquisition")

    def _stop(self) -> None:
        if self._spectrometer is not None:
            try:
                self._spectrometer.close()
            except Exception:
                log.exception("SpectrometerWorker: error closing device")
            self._spectrometer = None
            
        if self._config is not None:
            self._config = None

    @worker_thread
    def _on_set_config(self, msg: SetSpectrometerConfig) -> None:
        self._config = msg.config
        if self._spectrometer is not None and self._spectrometer.is_open:
            was_producing = self._prod_handle is not None
            if was_producing:
                handle = self._stop_producing()
                if handle is not None:
                    handle.wait(timeout=5.0)
            try:
                self._spectrometer.configure(self._config)
            except Exception as exc:
                log.exception("SpectrometerWorker: configure failed")
                self._reply_error(msg, str(exc))
                if was_producing:
                    self._start_producing(self._acquire_producer, on_item=self._on_acquired)
                return
            if was_producing:
                self._start_producing(self._acquire_producer, on_item=self._on_acquired)
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
