"""MFA-CC linear-stage worker — command-style, notifies position after each move."""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from control_readout.esp_301.controller import ESP301Controller
from control_readout.esp_301.mfa_cc.mfa_cc_device import MFACC
from control_readout.esp_301.mfa_cc.messages import (
    GetCurrentPosMFACC,
    HomeMFACC,
    MFACCPosReply,
    MFACCPosUpdate,
    MoveMFACCTo,
)
from control_readout.base.motorized_worker import MotorizedWorker

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

WORKER_ID = "mfacc"
#: 1-based ESP301 axis this stage is wired to. Adjust to match the hardware.
AXIS = 2


class MfaccWorker(MotorizedWorker):
    MOVE_MSG = MoveMFACCTo
    HOME_MSG = HomeMFACC
    GET_POS_MSG = GetCurrentPosMFACC

    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        controller: ESP301Controller,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._controller = controller
        self._stage: Optional[MFACC] = None

    def _start(self) -> None:
        if self._stage is None:
            self._stage = MFACC("mfacc", axis=AXIS, controller=self._controller)
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
        return "MFA-CC not started"

    def _move_value(self, msg: MoveMFACCTo) -> float:
        return msg.position

    def _do_move(self, value: float) -> None:
        self._stage.move_to(value)

    def _do_home(self) -> None:
        self._stage.home()

    def _read_value(self) -> float:
        return self._stage.position()

    def _pos_update_msg(self, value: float) -> MFACCPosUpdate:
        return MFACCPosUpdate(position=value)

    def _pos_reply_msg(self, value: float, request_id: str) -> MFACCPosReply:
        return MFACCPosReply(position=value, request_id=request_id)
