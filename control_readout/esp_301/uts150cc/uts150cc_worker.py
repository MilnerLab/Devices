"""UTS150CC linear-stage worker — command-style, notifies position after each move."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread

from control_readout.esp_301.controller import ESP301Controller
from control_readout.esp_301.uts150cc.uts150cc_device import UTS150CC
from control_readout.esp_301.uts150cc.messages import (
    GetCurrentPosUTS150CC,
    HomeUTS150CC,
    MoveUTS150CCTo,
    UTS150CCPosReply,
    UTS150CCPosUpdate,
)

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "uts150cc"
#: 1-based ESP301 axis this stage is wired to. Adjust to match the hardware.
AXIS = 3


class Uts150ccWorker(ThreadedWorker):
    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        controller: ESP301Controller,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._controller = controller
        self._stage: Optional[UTS150CC] = None

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(MoveUTS150CCTo, self._on_move))
        self._unsubs.append(self._bus.subscribe(HomeUTS150CC, self._on_home))
        self._unsubs.append(self._bus.subscribe(GetCurrentPosUTS150CC, self._on_get_pos))

    def _start(self) -> None:
        self._stage = UTS150CC("uts150cc", axis=AXIS, controller=self._controller)
        self._stage.start()

    def _pause(self) -> None:
        if self._stage is not None:
            self._stage.abort()

    def _reset(self) -> None:
        if self._stage is not None:
            self._stage.stop()
            self._stage = None

    @worker_thread
    def _on_move(self, msg: MoveUTS150CCTo) -> None:
        if self._stage is None:
            self._reply_error(msg, "UTS150CC not started")
            return
        try:
            self._stage.move_to(msg.position)
            self._notify(UTS150CCPosUpdate(position=self._stage.position()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("Uts150ccWorker: move failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_home(self, msg: HomeUTS150CC) -> None:
        if self._stage is None:
            self._reply_error(msg, "UTS150CC not started")
            return
        try:
            self._stage.home()
            self._notify(UTS150CCPosUpdate(position=self._stage.position()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("Uts150ccWorker: home failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_get_pos(self, msg: GetCurrentPosUTS150CC) -> None:
        if self._stage is None:
            self._reply_error(msg, "UTS150CC not started")
            return
        try:
            self._reply(UTS150CCPosReply(position=self._stage.position(), request_id=msg.id))
        except Exception as exc:
            log.exception("Uts150ccWorker: get position failed")
            self._reply_error(msg, str(exc))
