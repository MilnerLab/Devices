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

import numpy as np

from oscilloscope.buffer import ScopeBuffer
from oscilloscope.config import ScopeConfig
from oscilloscope.messages import (
    AcquirePoint,
    AcquirePointReply,
    AcquireTrace,
    AcquireTraceReply,
    SetScopeConfig,
)

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

    def _setup(self) -> None:
        super()._setup()  # registers SlotGrant subscription
        self._unsubs.append(self._bus.subscribe(SetScopeConfig, self._on_set_config))
        self._unsubs.append(self._bus.subscribe(AcquirePoint, self._on_acquire_point))
        self._unsubs.append(self._bus.subscribe(AcquireTrace, self._on_acquire_trace))

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
        if self._scope is not None:
            try:
                self._scope.close()
            except Exception:
                log.exception("OscilloscopeWorker: error closing device")
            self._scope = None

    @worker_thread
    def _on_set_config(self, msg: SetScopeConfig) -> None:
        self._config = msg.config
        self._reply_ok(msg)

    @worker_thread
    def _on_acquire_point(self, msg: AcquirePoint) -> None:
        scope = self._scope
        if scope is None:
            self._reply_error(msg, "Oscilloscope not started")
            return
        try:
            for _ in range(max(int(msg.discard), 0)):
                # The scope's buffer can still hold a record captured before the stage
                # finished moving. Averaging that in biases the point toward where the
                # probe used to be, which reads as a real feature in the interferogram.
                scope.acquire_trace()
            values: list[float] = []
            counts: list[int] = []
            for _ in range(max(int(msg.n_traces), 1)):
                row = self._channel_row(scope.acquire_trace(), msg.channel)
                positive = row[row > 0.0]
                values.append(float(positive.mean()) if positive.size else 0.0)
                counts.append(int(positive.size))
        except Exception as exc:
            log.exception("OscilloscopeWorker: point acquisition failed")
            self._reply_error(msg, str(exc))
            return
        self._reply(AcquirePointReply(values=values, counts=counts, request_id=msg.id))

    @worker_thread
    def _on_acquire_trace(self, msg: AcquireTrace) -> None:
        scope = self._scope
        if scope is None:
            self._reply_error(msg, "Oscilloscope not started")
            return
        try:
            row = self._channel_row(scope.acquire_trace(), msg.channel)
            positive = row[row > 0.0]
        except Exception as exc:
            log.exception("OscilloscopeWorker: trace acquisition failed")
            self._reply_error(msg, str(exc))
            return
        rate = self._config.sample_rate_hz
        self._reply(AcquireTraceReply(
            samples=[float(v) for v in row],
            dt_s=(1.0 / rate) if rate > 0 else 0.0,
            v_mean_pos=float(positive.mean()) if positive.size else 0.0,
            n_positive=int(positive.size),
            request_id=msg.id,
        ))

    def _channel_row(self, trace, channel: int) -> "np.ndarray":
        """Samples for a 1-based channel number, as the SCPI interface numbers them."""
        index = max(int(channel), 1) - 1
        samples = np.asarray(trace.samples)
        if index >= samples.shape[0]:
            raise IndexError(
                f"channel {channel} is not in this trace: the scope is configured for "
                f"{samples.shape[0]} channel(s)")
        return samples[index]

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
