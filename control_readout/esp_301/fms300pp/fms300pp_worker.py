"""FMS300PP linear-stage worker — command-style, notifies position after each move."""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from control_readout.esp_301.controller import ESP301Controller
from control_readout.esp_301.fms300pp.fms300pp_device import FMS300PP
from control_readout.esp_301.fms300pp.messages import (
    FMS300PPPosReply,
    FMS300PPPosUpdate,
    GetCurrentPosFMS300PP,
    HomeFMS300PP,
    MoveFMS300PPTo,
)
from control_readout.base.motorized_worker import MotorizedWorker

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

WORKER_ID = "fms300pp"
#: 1-based ESP301 axis this stage is wired to. Adjust to match the hardware.
AXIS = 1


class Fms300ppWorker(MotorizedWorker):
    MOVE_MSG = MoveFMS300PPTo
    HOME_MSG = HomeFMS300PP
    GET_POS_MSG = GetCurrentPosFMS300PP

    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        controller: ESP301Controller,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._controller = controller
        self._stage: Optional[FMS300PP] = None

    def _start(self) -> None:
        if self._stage is None:
            self._stage = FMS300PP("fms300pp", axis=AXIS, controller=self._controller)
            self._stage.start()

    def _pause(self) -> None:
        if self._stage is not None:
            self._stage.abort()

    def _resume(self) -> None:
        if self._stage is None:
            self._start()

    def _stop(self) -> None:
        if self._stage is not None:
            self._stage.stop()
            self._stage = None

    def _ready(self) -> bool:
        return self._stage is not None

    def _not_ready_msg(self) -> str:
        return "FMS300PP not started"

    def _move_value(self, msg: MoveFMS300PPTo) -> float:
        return msg.position

    def _do_move(self, value: float) -> None:
        self._stage.move_to(value)

    def _do_home(self) -> None:
        self._stage.home()

    def _read_value(self) -> float:
        return self._stage.position()

    def _pos_update_msg(self, value: float) -> FMS300PPPosUpdate:
        return FMS300PPPosUpdate(position=value)

    def _pos_reply_msg(self, value: float, request_id: str) -> FMS300PPPosReply:
        return FMS300PPPosReply(position=value, request_id=request_id)
