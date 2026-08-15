from __future__ import annotations

from typing import TYPE_CHECKING

from base_core.math.models import Angle
from control_readout.ell14.controller import ElliptecController
from control_readout.ell14.config import ELL14Config
from control_readout.ell14.device import ELL14Rotator
from control_readout.ell14.messages import (
    CurrentELL14Position,
    ELL14PositionReply,
    GetCurrentELL14Position,
    HomeELL14Rotator,
    RotateELL14,
)
from control_readout.base.motorized_worker import MotorizedWorker

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

WORKER_ID = "rotator"


class ELL14RotatorWorker(MotorizedWorker):
    MOVE_MSG = RotateELL14
    HOME_MSG = HomeELL14Rotator
    GET_POS_MSG = GetCurrentELL14Position

    def __init__(
        self,
        bus: EventBus,
        connector: SubprocessPipelineConnector,
        port: str,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = ELL14Config()
        self._port = port
        self._controller: ElliptecController | None = None
        self._rotator: ELL14Rotator | None = None
        self._is_paused = False

    def _start(self) -> None:
        if self._controller is None:
            self._controller = ElliptecController(self._port)
            self._controller.connect()
        if self._rotator is None:
            address = self._controller.resolve_address()
            self._rotator = ELL14Rotator("rotator", address, self._controller, self._config)
            self._rotator.start()
            self._rotator.apply_config()
        self._is_paused = False

    def _pause(self) -> None:
        self._is_paused = True

    def _resume(self) -> None:
        self._is_paused = False

    def _stop(self) -> None:
        if self._rotator is not None:
            self._rotator.stop()
            self._rotator = None
        if self._controller is not None:
            self._controller.disconnect()
            self._controller = None
        self._is_paused = False

    def _ready(self) -> bool:
        return self._rotator is not None and not self._is_paused

    def _not_ready_msg(self) -> str:
        return "Rotator not started or paused!"

    def _move_value(self, msg: RotateELL14) -> Angle:
        return msg.angle

    def _do_move(self, value: Angle) -> None:
        self._rotator.rotate(value)

    def _do_home(self) -> None:
        self._rotator.home()

    def _read_value(self) -> Angle:
        return self._rotator.current_angle

    def _pos_update_msg(self, value: Angle) -> CurrentELL14Position:
        return CurrentELL14Position(angle=value)

    def _pos_reply_msg(self, value: Angle, request_id: str) -> ELL14PositionReply:
        return ELL14PositionReply(angle=value, request_id=request_id)
