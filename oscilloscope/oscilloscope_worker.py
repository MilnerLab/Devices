"""
Oscilloscope producer worker — streams traces into shared memory.

Mirrors SpectrometerWorker: a production stream pulls a trace from the driver,
writes it to a granted slot, and notifies the slot was written.
The driver is the real TBS2012C when the instrument answers, and the synthetic-trace
mock when it does not — see :class:`DeviceWorkerMixin`.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Callable

from base_core.framework.events.event_bus import EventBus
from base_core.framework.shm.writer_worker import WriterWorker
from base_core.ipc.device_worker import DeviceWorkerMixin
from base_core.ipc.subprocess_connector import SubprocessPipelineConnector
from base_core.ipc.threaded_worker import worker_thread

from oscilloscope.buffer import ScopeBuffer
from oscilloscope.config import ScopeConfig
from oscilloscope.messages import ScopeTimebase, SetScopeConfig

log = logging.getLogger(__name__)

WORKER_ID = "oscilloscope"


class OscilloscopeWorker(DeviceWorkerMixin, WriterWorker[ScopeBuffer]):
    def __init__(
        self,
        bus: EventBus,
        connector: SubprocessPipelineConnector,
        config: ScopeConfig,
        get_buffer: Callable[[], ScopeBuffer],
    ) -> None:
        super().__init__(WORKER_ID, bus, connector, get_buffer)
        self._config = config
        self._scope = None
        self._item_id = 0
        #: Last sample interval reported to the main process. Only changes are sent.
        self._dt_s = 0.0

    def _setup(self) -> None:
        super()._setup()  # registers SlotGrant subscription
        self._unsubs.append(self._bus.subscribe(SetScopeConfig, self._on_set_config))

    def _start(self) -> None:
        if self._scope is not None:
            log.warning("OscilloscopeWorker: _start() while already running")
            return
        self._scope = self._open_device()
        self._start_producing(self._acquire_producer, on_item=self._on_acquired)

    def _connect(self):
        # open() and apply_config() belong inside the real path, not after it: a driver
        # that constructs happily and fails on open() is the ordinary PyVISA failure,
        # and it has to demote like any other.
        from oscilloscope.tbs_driver import TbsScope

        scope = TbsScope(self._config)
        scope.open()
        scope.apply_config()
        return scope

    def _connect_mock(self):
        from oscilloscope.mock_driver import MockScope

        scope = MockScope(self._config)
        scope.open()
        scope.apply_config()
        return scope

    def _pause(self) -> None:
        handle = self._stop_producing()
        if handle is not None:
            handle.wait(timeout=5.0)
            if not handle.done_event.is_set():
                log.warning("OscilloscopeWorker: acquisition did not stop in 5 s")

    def _resume(self) -> None:
        self._start_producing(self._acquire_producer, on_item=self._on_acquired)

    def _stop(self) -> None:
        self._pause()
        # Forget the reported time base so the next start re-reports it: the main process
        # may have been rebound to a fresh worker in between and know nothing.
        self._dt_s = 0.0
        if self._scope is not None:
            try:
                self._scope.close()
            except Exception:
                log.exception("OscilloscopeWorker: error closing device")
            self._scope = None

    @worker_thread
    def _on_set_config(self, msg: SetScopeConfig) -> None:
        # spec.shape, not the ScopeMemorySpec accessors: AttachBuffer carries a plain
        # MemorySpec over the pipe, so what the subprocess attached is the base class.
        max_channels, max_samples = self._get_buffer().spec.shape
        if msg.config.channels > max_channels or msg.config.n_samples > max_samples:
            self._reply_error(
                msg,
                f"record ({msg.config.channels}, {msg.config.n_samples}) does not fit a "
                f"buffer slot of ({max_channels}, {max_samples})")
            return
        self._config = msg.config
        self._reply_ok(msg)

    def _acquire_producer(self, stop: threading.Event):
        """Generator: yields (slot, trace) until stopped."""
        while not stop.is_set():
            scope = self._scope
            if scope is None:
                break
            slot = self._get_slot()
            if slot is None:
                time.sleep(0.001)
                continue
            try:
                yield (slot, scope.acquire_trace())
            except Exception:
                log.exception("OscilloscopeWorker: acquisition error — stopping loop")
                return

    def _on_acquired(self, item: tuple) -> None:
        slot, trace = item
        self._get_buffer().write_trace(slot, trace.samples)
        self._item_id += 1
        self._notify_written(slot, self._item_id, trace.timestamp_ns)
        if trace.dt_s != self._dt_s:
            # One message per turn of the horizontal knob. The frame itself carries no
            # metadata, so this is the only way the time axis learns its scale.
            self._dt_s = trace.dt_s
            self._notify(ScopeTimebase(dt_s=trace.dt_s))
