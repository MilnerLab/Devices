"""
Oscilloscope producer worker — streams traces into shared memory.

Mirrors the spectrometer ``SpectrometerWorker``: an acquisition thread pulls a trace
from the driver, writes it to a granted slot, and notifies the slot was written. The
driver is the mock by default, the real TBS2012C when ``config.mock=False``.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Callable

from base_core.framework.events.event_bus import EventBus
from base_core.ipc.subprocess_connector import SubprocessPipelineConnector
from base_core.ipc.worker import BaseWorker

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


class OscilloscopeWorker(BaseWorker):
    def __init__(
        self,
        bus: EventBus,
        connector: SubprocessPipelineConnector,
        config: ScopeConfig,
        get_slot: Callable[[], int | None],
        get_buffer: Callable[[], ScopeBuffer],
        notify_written: Callable[[int, int, int], None],
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = config
        self._get_slot = get_slot
        self._get_buffer = get_buffer
        self._notify_written = notify_written
        self._scope = None
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._item_id = 0

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(SetScopeConfig, self._on_set_config))

    def _start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            log.warning("OscilloscopeWorker: _start() while already running")
            return
        self._scope = _make_driver(self._config)
        self._scope.open()
        self._scope.apply_config()
        self._running.set()
        self._thread = threading.Thread(
            target=self._acquire_loop, name="oscilloscope-acquire", daemon=True
        )
        self._thread.start()

    def _stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        if self._scope is not None:
            try:
                self._scope.close()
            except Exception:
                log.exception("OscilloscopeWorker: error closing device")
            self._scope = None

    def _reset(self) -> None:
        self._stop()
        self._start()

    def _on_set_config(self, msg: SetScopeConfig) -> None:
        self._config = msg.config
        self._reply_ok(msg)

    def _acquire_loop(self) -> None:
        while self._running.is_set():
            scope = self._scope
            if scope is None:
                break
            slot = self._get_slot()
            if slot is None:
                time.sleep(0.001)
                continue
            try:
                trace = scope.acquire_trace()
                self._get_buffer().write_trace(slot, trace.samples)
                self._item_id += 1
                self._notify_written(slot, self._item_id, trace.timestamp_ns)
            except Exception:
                log.exception("OscilloscopeWorker: acquisition error — stopping loop")
                self._running.clear()
