"""RGV100BL rotation worker (HWP) — command-style, notifies angle after each move."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread

from base_core.math.models import Angle
from control_readout.newport_xps.controller import XPSController
from control_readout.newport_xps.rgv100bl.messages import (
    GetCurrentRGVAngle,
    HomeRGV,
    RGVAngleReply,
    RGVAngleUpdate,
    RotateRGVBy,
    RotateRGVTo,
)
from control_readout.newport_xps.rgv100bl.rgv100bl_device import RGV

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
        self._unsubs.append(self._bus.subscribe(RotateRGVBy, self._on_rotate_by))
        self._unsubs.append(self._bus.subscribe(HomeRGV, self._on_home))
        self._unsubs.append(self._bus.subscribe(GetCurrentRGVAngle, self._on_get_angle))

    def _start(self) -> None:
        if self._rotator is None:
            log.info("Rgv100blWorker: creating RGV, start/initialize/home ...")
            self._rotator = RGV("rot", group="GROUP1", controller=self._controller,positioner="POSITIONER")
            self._rotator.start()
            self._rotator.initialize()
            self._rotator.home()  # required after initialize before any move (else XPS error -22)
            log.info("Rgv100blWorker: ready, homed to %.4f deg", self._rotator.angle().Deg)

    def _pause(self) -> None:
        if self._rotator is not None:
            self._rotator.abort()

    def _resume(self) -> None:
        if self._rotator is None:
            self._start()

    def _stop(self) -> None:
        if self._rotator is not None:
            self._rotator.stop()
            self._rotator = None

    @worker_thread
    def _on_rotate(self, msg: RotateRGVTo) -> None:
        if self._rotator is None:
            log.warning("Rgv100blWorker: RotateRGVTo received but rotator not started — ignoring")
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            before = self._rotator.angle().Deg
            # NOTE: RGV.rotate() -> move_to() is an ABSOLUTE move to msg.angle, not a
            # relative nudge. If callers pass a relative correction, this is a bug.
            log.info(
                "Rgv100blWorker: rotate to (absolute) %.4f deg [current %.4f deg]",
                msg.angle.Deg, before,
            )
            self._rotator.rotate(msg.angle)
            after = self._rotator.angle().Deg
            log.info("Rgv100blWorker: rotate complete: %.4f -> %.4f deg", before, after)
            self._notify(RGVAngleUpdate(angle=self._rotator.angle()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("Rgv100blWorker: rotate failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_rotate_by(self, msg: RotateRGVBy) -> None:
        if self._rotator is None:
            log.warning("Rgv100blWorker: RotateRGVBy received but rotator not started — ignoring")
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            before = self._rotator.angle().Deg
            # Relative move: nudge the plate BY msg.angle from where it is now.
            log.info(
                "Rgv100blWorker: rotate by (relative) %.4f deg [current %.4f deg]",
                msg.angle.Deg, before,
            )
            self._rotator.move_by(msg.angle.Deg)
            after = self._rotator.angle().Deg
            log.info("Rgv100blWorker: rotate complete: %.4f -> %.4f deg", before, after)
            self._notify(RGVAngleUpdate(angle=self._rotator.angle()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("Rgv100blWorker: rotate_by failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_home(self, msg: HomeRGV) -> None:
        if self._rotator is None:
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            log.info("Rgv100blWorker: homing ...")
            self._rotator.home()
            log.info("Rgv100blWorker: home complete at %.4f deg", self._rotator.angle().Deg)
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
            self._reply(RGVAngleReply(angle=self._rotator.angle(), request_id=msg.id))
        except Exception as exc:
            log.exception("Rgv100blWorker: get angle failed")
            self._reply_error(msg, str(exc))