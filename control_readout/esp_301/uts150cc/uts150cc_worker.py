"""UTS150CC linear-stage worker — command-style, notifies position after each move."""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from base_core.ipc.connection_mode import ConnectionMode
from base_core.ipc.device_worker import DeviceWorkerMixin

from control_readout.base.controller_provider import ControllerProvider
from control_readout.esp_301.uts150cc.uts150cc_device import UTS150CC
from control_readout.esp_301.uts150cc.spec import AXIS
from control_readout.esp_301.uts150cc.messages import (
    GetCurrentPosUTS150CC,
    HomeUTS150CC,
    MoveUTS150CCTo,
    UTS150CCPosReply,
    UTS150CCPosUpdate,
)
from control_readout.base.motorized_worker import MotorizedWorker

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

WORKER_ID = "uts150cc"


class Uts150ccWorker(DeviceWorkerMixin, MotorizedWorker):
    MOVE_MSG = MoveUTS150CCTo
    HOME_MSG = HomeUTS150CC
    GET_POS_MSG = GetCurrentPosUTS150CC

    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        provider: ControllerProvider,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._provider = provider
        self._stage: Optional[UTS150CC] = None

    def _start(self) -> None:
        if self._stage is None:
            self._stage = self._open_device()

    def _connect(self) -> UTS150CC:
        return self._attach(self._provider.acquire(ConnectionMode.DEVICE))

    def _connect_mock(self) -> UTS150CC:
        return self._attach(self._provider.acquire(ConnectionMode.MOCK))

    def _attach(self, controller) -> UTS150CC:
        stage = UTS150CC("uts150cc", axis=AXIS, controller=controller)
        stage.start()
        return stage

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
        return "UTS150CC not started"

    def _move_value(self, msg: MoveUTS150CCTo) -> float:
        return msg.position

    def _do_move(self, value: float) -> None:
        self._stage.move_to(value)

    def _do_home(self) -> None:
        self._stage.home()

    def _read_value(self) -> float:
        return self._stage.position()

    def _pos_update_msg(self, value: float) -> UTS150CCPosUpdate:
        return UTS150CCPosUpdate(position=value)

    def _pos_reply_msg(self, value: float, request_id: str) -> UTS150CCPosReply:
        return UTS150CCPosReply(position=value, request_id=request_id)
