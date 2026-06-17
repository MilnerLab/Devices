from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread
from control_readout.elliptec.config import ELL14Config
from control_readout.elliptec.elliptec_ell14 import Rotator
from control_readout.elliptec.messages import (
    HomeRotator,
    Rotate,
    RotatorHomed,
    RotatorMoved,
)

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "rotator"


class RotatorWorker(ThreadedWorker):
    def __init__(
        self,
        bus: EventBus,
        connector: SubprocessPipelineConnector,
        config: ELL14Config,
        port: str,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = config
        self._port = port
        self._rotator: Rotator | None = None

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(Rotate, self._on_rotate))
        self._unsubs.append(self._bus.subscribe(HomeRotator, self._on_home))

    def _start(self) -> None:
        self._rotator = Rotator(self._config)
        self._rotator.open(self._port)
        self._rotator.apply_config()

    def _stop(self) -> None:
        if self._rotator is not None:
            self._rotator.close()
            self._rotator = None

    def _reset(self) -> None:
        self._stop()
        self._start()

    @worker_thread
    def _on_rotate(self, msg: Rotate) -> None:
        if self._rotator is None:
            self._reply_error(msg, "Rotator not started")
            return
        try:
            self._rotator.rotate(msg.angle)
            self._notify(RotatorMoved(angle=self._rotator.current_angle))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("RotatorWorker: rotate failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_home(self, msg: HomeRotator) -> None:
        if self._rotator is None:
            self._reply_error(msg, "Rotator not started")
            return
        try:
            self._rotator.home()
            self._notify(RotatorHomed())
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("RotatorWorker: home failed")
            self._reply_error(msg, str(exc))
