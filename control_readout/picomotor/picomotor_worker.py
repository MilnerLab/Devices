"""Picomotor worker — manual mirror tip/tilt (StepBy), no PID."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread

from control_readout.picomotor.config import PicomotorConfig
from control_readout.picomotor.messages import StepBy, StepsMoved

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "picomotor"


def _make_driver(config: PicomotorConfig):
    if config.mock:
        from control_readout.picomotor.mock_driver import MockPicomotor
        return MockPicomotor(config)
    from control_readout.picomotor.picomotor_driver import Picomotor8742
    return Picomotor8742(config)


class PicomotorWorker(ThreadedWorker):
    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        config: PicomotorConfig,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = config
        self._driver = None

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(StepBy, self._on_step_by))

    def _start(self) -> None:
        self._driver = _make_driver(self._config)
        self._driver.open()

    def _stop(self) -> None:
        if self._driver is not None:
            self._driver.close()
            self._driver = None

    def _reset(self) -> None:
        self._stop()
        self._start()

    @worker_thread
    def _on_step_by(self, msg: StepBy) -> None:
        if self._driver is None:
            self._reply_error(msg, "Picomotor not started")
            return
        try:
            self._driver.move_by(msg.axis, msg.steps)
            self._notify(StepsMoved(axis=msg.axis, total_steps=self._driver.position(msg.axis)))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("PicomotorWorker: step failed")
            self._reply_error(msg, str(exc))
