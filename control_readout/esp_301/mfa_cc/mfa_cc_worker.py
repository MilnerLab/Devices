"""MFA-CC linear-stage worker — command-style, notifies position after each move."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread

from control_readout.esp_301.controller import ESP301Controller
from control_readout.esp_301.mfa_cc.mfa_cc_device import MFACC
from control_readout.esp_301.mfa_cc.messages import (
    GetCurrentPosMFACC,
    HomeMFACC,
    MFACCPosReply,
    MFACCPosUpdate,
    MoveMFACCTo,
)

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "mfacc"
#: 1-based ESP301 axis this stage is wired to. Adjust to match the hardware.
AXIS = 2


class MfaccWorker(ThreadedWorker):
    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        controller: ESP301Controller,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._controller = controller
        self._stage: Optional[MFACC] = None

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(MoveMFACCTo, self._on_move))
        self._unsubs.append(self._bus.subscribe(HomeMFACC, self._on_home))
        self._unsubs.append(self._bus.subscribe(GetCurrentPosMFACC, self._on_get_pos))

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

    @worker_thread
    def _on_move(self, msg: MoveMFACCTo) -> None:
        if self._stage is None:
            self._reply_error(msg, "MFA-CC not started")
            return
        try:
            self._stage.move_to(msg.position)
            self._notify(MFACCPosUpdate(position=self._stage.position()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("MfaccWorker: move failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_home(self, msg: HomeMFACC) -> None:
        if self._stage is None:
            self._reply_error(msg, "MFA-CC not started")
            return
        try:
            self._stage.home()
            self._notify(MFACCPosUpdate(position=self._stage.position()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("MfaccWorker: home failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_get_pos(self, msg: GetCurrentPosMFACC) -> None:
        if self._stage is None:
            self._reply_error(msg, "MFA-CC not started")
            return
        try:
            self._reply(MFACCPosReply(position=self._stage.position(), request_id=msg.id))
        except Exception as exc:
            log.exception("MfaccWorker: get position failed")
            self._reply_error(msg, str(exc))
