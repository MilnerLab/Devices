"""RGV100BL rotation worker (HWP) — command-style, notifies angle after each move."""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from base_core.math.models import Angle
from control_readout.base.motorized_worker import MotorizedWorker
from control_readout.newport_xps.controller import XPSController
from control_readout.newport_xps.rgv100bl.messages import (
    GetCurrentRGVAngle,
    HomeRGV,
    RGVAngleReply,
    RGVAngleUpdate,
    RotateRGVTo,
)
from control_readout.newport_xps.rgv100bl.rgv100bl_device import RGV

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

WORKER_ID = "rgv100bl"


class Rgv100blWorker(MotorizedWorker):
    MOVE_MSG = RotateRGVTo
    HOME_MSG = HomeRGV
    GET_POS_MSG = GetCurrentRGVAngle

    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        controller: XPSController,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._controller = controller
        self._rotator: Optional[RGV] = None

    def _start(self) -> None:
        if self._rotator is None:
            self._rotator = RGV("rot", group="GROUP1", controller=self._controller,positioner="POSITIONER")
            self._rotator.start()
            self._rotator.initialize()
            self._rotator.home()  # required after initialize before any move (else XPS error -22)

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

    def _ready(self) -> bool:
        return self._rotator is not None

    def _not_ready_msg(self) -> str:
        return "RGV100BL not started"

    def _move_value(self, msg: RotateRGVTo) -> Angle:
        return msg.angle

    def _do_move(self, value: Angle) -> None:
        self._rotator.rotate(value)

    def _do_home(self) -> None:
        self._rotator.home()

    def _read_value(self) -> Angle:
        return self._rotator.angle()

    def _pos_update_msg(self, value: Angle) -> RGVAngleUpdate:
        return RGVAngleUpdate(angle=value)

    def _pos_reply_msg(self, value: Angle, request_id: str) -> RGVAngleReply:
        return RGVAngleReply(angle=value, request_id=request_id)