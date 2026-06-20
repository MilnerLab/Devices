from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread
from control_readout.ell14.config import ELL14Config
from control_readout.ell14.ell14_driver import ELL14Rotator
from control_readout.ell14.messages import CurrentELL14Position, HomeELL14Rotator, RotateELL14

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "rotator"


class ELL14RotatorWorker(ThreadedWorker):
    def __init__(
        self,
        bus: EventBus,
        connector: SubprocessPipelineConnector,
        port: str,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = ELL14Config()
        self._port = port
        self._rotator: ELL14Rotator | None = None
        self._is_paused = False

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(RotateELL14, self._on_rotate))
        self._unsubs.append(self._bus.subscribe(HomeELL14Rotator, self._on_home))

    def _start(self) -> None:
        if self._rotator is None:
            self._rotator = ELL14Rotator(self._config)
            self._rotator.open(self._port)
            self._rotator.apply_config()
        self._is_paused = False

    def _pause(self) -> None:
        self._is_paused = True

    def _reset(self) -> None:
        if self._rotator is not None:
            self._rotator.close()
            self._rotator = None
            self._is_paused = False

    @worker_thread
    def _on_rotate(self, msg: RotateELL14) -> None:
        if self._rotator is None or self._is_paused:
            self._reply_error(msg, "Rotator not started or paused!")
            return
        try:
            self._rotator.rotate(msg.angle)
            self._notify(CurrentELL14Position(angle=self._rotator.current_angle))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("RotatorWorker: rotate failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_home(self, msg: HomeELL14Rotator) -> None:
        if self._rotator is None or self._is_paused:
            self._reply_error(msg, "Rotator not started or paused!")
            return
        try:
            self._rotator.home()
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("RotatorWorker: home failed")
            self._reply_error(msg, str(exc))
