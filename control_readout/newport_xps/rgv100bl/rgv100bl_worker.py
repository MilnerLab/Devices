"""RGV100BL rotation worker (HWP) — command-style, notifies angle after each move."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread

from control_readout.newport_xps.controller import XPSController
from control_readout.newport_xps.rgv100bl.messages import GetCurrentRGVAngle, HomeRGV, RGVAngleUpdate, RotateRGVTo
from control_readout.newport_xps.rgv100bl.rgv_device import RGV
from control_readout.rgv100bl.config import Rgv100Config
from control_readout.rgv100bl.messages import HomeHwp, HwpAngleUpdate, RotateHwpTo

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "rgv100bl"




class Rgv100blWorker(ThreadedWorker):
    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        controller: XPSController,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._controller = controller
        self._rotator: Optional[RGV] = None

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(RotateRGVTo, self._on_rotate))
        self._unsubs.append(self._bus.subscribe(HomeRGV, self._on_home))
        self._unsubs.append(self._bus.subscribe(GetCurrentRGVAngle, self._on_get_angle))

    def _start(self) -> None:
        self._rotator = RGV("rot", group="Rot", controller=self._controller)
        self._rotator.start()

    def _pause(self) -> None:
        if self._rotator is not None:
            self._rotator.abort()

    def _reset(self) -> None:
        if self._rotator is not None:
            self._rotator.stop()
            self._rotator = None

    @worker_thread
    def _on_rotate(self, msg: RotateHwpTo) -> None:
        if self._rotator is None:
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            self._rotator.rotate(msg.angle)
            self._notify(RGVAngleUpdate(angle=self._rotator.angle()))
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
            self._notify(RGVAngleUpdate(angle=self._rotator.angle()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("Rgv100blWorker: home failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_get_angle(self, msg: GetCurrentRGVAngle) -> None:
        if self._rotator is None:
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            self._reply(RGVAngleUpdate(angle=self._rotator.angle(), request_id=msg.id))
        except Exception as exc:
            log.exception("Rgv100blWorker: get angle failed")
            self._reply_error(msg, str(exc))