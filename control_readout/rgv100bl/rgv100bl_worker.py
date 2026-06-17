"""RGV100BL rotation worker (HWP) — command-style, notifies angle after each move."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread

from control_readout.rgv100bl.config import Rgv100Config
from control_readout.rgv100bl.messages import HomeHwp, HwpAngleUpdate, RotateHwpTo

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "rgv100bl"


def _make_driver(config: Rgv100Config):
    if config.mock:
        from control_readout.rgv100bl.mock_driver import MockRgvRotator
        return MockRgvRotator(config)
    from control_readout.rgv100bl.xps_driver import XpsRgvRotator
    return XpsRgvRotator(config)


class Rgv100blWorker(ThreadedWorker):
    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        config: Rgv100Config,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = config
        self._rotator = None

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(RotateHwpTo, self._on_rotate))
        self._unsubs.append(self._bus.subscribe(HomeHwp, self._on_home))

    def _start(self) -> None:
        self._rotator = _make_driver(self._config)
        self._rotator.open()

    def _stop(self) -> None:
        if self._rotator is not None:
            self._rotator.close()
            self._rotator = None

    def _reset(self) -> None:
        self._stop()
        self._start()

    @worker_thread
    def _on_rotate(self, msg: RotateHwpTo) -> None:
        if self._rotator is None:
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            self._rotator.rotate(msg.angle)
            self._notify(HwpAngleUpdate(angle=self._rotator.current_angle))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("Rgv100blWorker: rotate failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_home(self, msg: HomeHwp) -> None:
        if self._rotator is None:
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            self._rotator.home()
            self._notify(HwpAngleUpdate(angle=self._rotator.current_angle))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("Rgv100blWorker: home failed")
            self._reply_error(msg, str(exc))
