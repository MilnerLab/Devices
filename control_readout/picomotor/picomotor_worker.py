"""Picomotor worker — manual mirror tip/tilt, no PID."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from base_core.ipc.device_worker import DeviceWorkerMixin
from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread

from control_readout.picomotor.config import PicomotorConfig
from control_readout.picomotor.messages import (
    QuerySteps,
    StepBy,
    StepsMoved,
    StepsReply,
    StepTo,
    ZeroAxis,
)

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "picomotor"


class PicomotorWorker(DeviceWorkerMixin, ThreadedWorker):
    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        config: PicomotorConfig,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = config
        self._driver = None
        self._is_paused = False

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(StepBy, self._on_step_by))
        self._unsubs.append(self._bus.subscribe(StepTo, self._on_step_to))
        self._unsubs.append(self._bus.subscribe(ZeroAxis, self._on_zero_axis))
        self._unsubs.append(self._bus.subscribe(QuerySteps, self._on_query_steps))

    def _start(self) -> None:
        if self._driver is None:
            self._driver = self._open_device()
        self._is_paused = False

    def _connect(self):
        from control_readout.picomotor.picomotor_driver import Picomotor8742

        driver = Picomotor8742(self._config)
        driver.open()
        return driver

    def _connect_mock(self):
        from control_readout.picomotor.mock_driver import MockPicomotor

        driver = MockPicomotor(self._config)
        driver.open()
        return driver

    def _pause(self) -> None:
        self._is_paused = True

    def _resume(self) -> None:
        self._is_paused = False

    def _stop(self) -> None:
        if self._driver is not None:
            self._driver.close()
            self._driver = None
        self._is_paused = False

    # -- commands ----------------------------------------------------------

    @worker_thread
    def _on_step_by(self, msg: StepBy) -> None:
        self._commanded(msg, lambda d: d.move_by(msg.axis, msg.steps), msg.axis)

    @worker_thread
    def _on_step_to(self, msg: StepTo) -> None:
        self._commanded(msg, lambda d: d.move_to(msg.axis, msg.steps), msg.axis)

    @worker_thread
    def _on_zero_axis(self, msg: ZeroAxis) -> None:
        self._commanded(msg, lambda d: d.zero(msg.axis), msg.axis)

    @worker_thread
    def _on_query_steps(self, msg: QuerySteps) -> None:
        driver = self._require(msg)
        if driver is None:
            return
        axes = tuple(msg.axes) or tuple(self._config.axes)
        try:
            steps = {int(axis): driver.position(int(axis)) for axis in axes}
        except Exception as exc:
            log.exception("PicomotorWorker: reading the counters failed")
            self._reply_error(msg, str(exc))
            return
        self._reply(StepsReply(steps=steps, request_id=msg.id))

    # -- shared control flow ------------------------------------------------

    def _commanded(self, msg, action, axis: int) -> None:
        """Run a command, then report the counter the controller reads back.

        The read-back is not optional and is never computed here. On an open-loop stage
        the controller's count is the only truth there is, and a locally accumulated
        total drifts silently the moment a step is dropped or a move is refused —
        exactly when the operator most needs to be told.
        """
        driver = self._require(msg)
        if driver is None:
            return
        try:
            action(driver)
            self._notify(StepsMoved(axis=axis, total_steps=driver.position(axis)))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("PicomotorWorker: %s failed", type(msg).__name__)
            self._reply_error(msg, str(exc))

    def _require(self, msg):
        if self._driver is None or self._is_paused:
            self._reply_error(msg, "Picomotor not started or paused")
            return None
        return self._driver
