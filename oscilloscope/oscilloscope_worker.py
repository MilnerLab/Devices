"""
Oscilloscope producer worker — streams traces into shared memory.

Mirrors SpectrometerWorker: a production stream pulls a trace from the driver,
writes it to a granted slot, and notifies the slot was written.
The driver is the mock by default, the real TBS2012C when config.mock=False.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Callable

from base_core.framework.events.event_bus import EventBus
from base_core.framework.shm.writer_worker import WriterWorker
from base_core.ipc.subprocess_connector import SubprocessPipelineConnector
from base_core.ipc.threaded_worker import worker_thread

from oscilloscope.buffer import ScopeBuffer
from oscilloscope.config import ScopeConfig
from oscilloscope.messages import SetScopeConfig

log = logging.getLogger(__name__)

WORKER_ID = "oscilloscope"


def _make_driver(config: ScopeConfig):
    if config.mock:
        from oscilloscope.mock_driver import MockScope
        return MockScope(config)
    from oscilloscope.tbs_driver import TbsScope
    return TbsScope(config)


class OscilloscopeWorker(WriterWorker[ScopeBuffer]):
    def __init__(
        self,
        bus: EventBus,
        connector: SubprocessPipelineConnector,
        config: ScopeConfig,
        get_buffer: Callable[[], ScopeBuffer],
    ) -> None:
        super().__init__(WORKER_ID, bus, connector, ScopeBuffer, get_buffer)
        self._config = config
        self._scope = None
        self._item_id = 0

    def _setup(self) -> None:
        super()._setup()  # registers SlotGrant subscription
        self._unsubs.append(self._bus.subscribe(SetScopeConfig, self._on_set_config))

    def _start(self) -> None:
        if self._scope is not None:
            log.warning("OscilloscopeWorker: _start() while already running")
            return
        self._scope = _make_driver(self._config)
        self._scope.open()
        self._scope.apply_config()
        self._start_producing(self._acquire_producer, on_item=self._on_acquired)

    def _stop(self) -> None:
        if self._prod_handle is not None:
            fut = self._prod_handle.future
            self._stop_producing()
            try:
                fut.result(timeout=5.0)
            except Exception:
                log.warning("OscilloscopeWorker: acquisition did not stop in 5 s")
        if self._scope is not None:
            try:
                self._scope.close()
            except Exception:
                log.exception("OscilloscopeWorker: error closing device")
            self._scope = None

    def _reset(self) -> None:
        self._stop()
        self._start()

    @worker_thread
    def _on_set_config(self, msg: SetScopeConfig) -> None:
        self._config = msg.config
        self._reply_ok(msg)

    def _acquire_producer(self, stop: threading.Event):
        """Generator: yields (slot, samples, timestamp_ns) until stopped."""
        while not stop.is_set():
            scope = self._scope
            if scope is None:
                break
            slot = self._get_slot()
            if slot is None:
                time.sleep(0.001)
                continue
            try:
                trace = scope.acquire_trace()
                yield (slot, trace.samples, trace.timestamp_ns)
            except Exception:
                log.exception("OscilloscopeWorker: acquisition error — stopping loop")
                return

    def _on_acquired(self, item: tuple) -> None:
        slot, samples, timestamp_ns = item
        self._get_buffer().write_trace(slot, samples)
        self._item_id += 1
        self._notify_written(slot, self._item_id, timestamp_ns)
