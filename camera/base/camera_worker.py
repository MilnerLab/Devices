from __future__ import annotations

import logging
import threading
import time
from typing import Callable

from base_core.framework.events.event_bus import EventBus
from base_core.framework.shm.writer_worker import WriterWorker
from base_core.ipc.subprocess_connector import SubprocessPipelineConnector
from base_core.ipc.threaded_worker import worker_thread
from camera.base.buffer import CameraBuffer
from camera.base.camera import Camera
from camera.base.config import CameraConfig
from camera.base.messages import SetCameraConfig

log = logging.getLogger(__name__)


class CameraWorker(WriterWorker[CameraBuffer]):
    """
    Runs the acquisition loop inside a camera subprocess.

    Generic across camera vendors: the concrete Camera subclass is injected via
    camera_factory, so no per-vendor subclass of this worker is needed -- only a
    Camera implementation (e.g. VmiCamera) and a subprocess entry point that
    wires it in (see camera/vmi/camera_process.py).

    On StartWorker: opens the hardware, applies config, starts the acquisition stream.
    On PauseWorker: drains the stream, closes nothing (device stays open).
    On ResumeWorker: restarts the acquisition stream (device stays open).
    On StopWorker: closes the hardware.
    On SetCameraConfig: applies exposure/gain live while the device is open (acquisition
    keeps running); the whole config is stored and applied in full on the next start.
    """

    def __init__(
        self,
        worker_id: str,
        bus: EventBus,
        connector: SubprocessPipelineConnector,
        get_buffer,
        camera_factory: Callable[[CameraConfig], Camera],
    ) -> None:
        super().__init__(worker_id, bus, connector, get_buffer)
        self._camera_factory = camera_factory
        self._config: CameraConfig | None = None
        self._camera: Camera | None = None
        self._item_id = 0

    def _setup(self) -> None:
        super()._setup()  # registers SlotGrant subscription
        self._unsubs.append(
            self._bus.subscribe(SetCameraConfig, self._on_set_config)
        )

    def _start(self) -> None:
        if self._camera is None:
            self._camera = self._camera_factory(self._config)
            self._camera.open()
            self._camera.apply_config()
        self._start_producing(self._acquire_producer, on_item=self._on_acquired)
        log.debug("CameraWorker: started acquisition")

    def _pause(self) -> None:
        handle = self._stop_producing()
        if handle is not None:
            handle.wait(timeout=5.0)
            if not handle.done_event.is_set():
                log.warning("CameraWorker: acquisition did not stop in 5 s")

    def _resume(self) -> None:
        self._start_producing(self._acquire_producer, on_item=self._on_acquired)
        log.debug("CameraWorker: resumed acquisition")

    def _stop(self) -> None:
        if self._camera is not None:
            try:
                self._camera.close()
            except Exception:
                log.exception("CameraWorker: error closing device")
            self._camera = None

        if self._config is not None:
            self._config = None

    @worker_thread
    def _on_set_config(self, msg: SetCameraConfig) -> None:
        self._config = msg.config
        if self._camera is not None and self._camera.is_open:
            # Live, without stopping the acquisition loop: update_live only writes nodes
            # that are safe mid-stream. A full configure() would re-run BeginAcquisition on
            # a stream that is already running.
            try:
                self._camera.update_live(self._config)
            except Exception as exc:
                log.exception("CameraWorker: live config update failed")
                self._reply_error(msg, str(exc))
                return
        self._reply_ok(msg)

    def _acquire_producer(self, stop: threading.Event):
        """Generator: yields (slot, frame, timestamp_ns) until stopped."""
        while not stop.is_set():
            camera = self._camera
            if camera is None:
                break
            slot = self._get_slot()
            if slot is None:
                time.sleep(0.001)
                continue
            try:
                data = camera.acquire_frame()
                yield (slot, data.frame, data.timestamp_ns)
            except Exception:
                log.exception("CameraWorker: acquisition error — stopping loop")
                return

    def _on_acquired(self, item: tuple) -> None:
        slot, frame, timestamp_ns = item
        self._get_buffer().write_frame(slot, frame)
        self._item_id += 1
        self._notify_written(slot, self._item_id, timestamp_ns)
