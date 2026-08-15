"""Base for command-style motorized-stage workers: move/home, notifying
position after each action, plus an on-demand position pull."""
from __future__ import annotations

import logging
from typing import ClassVar

from base_core.ipc.message import Message, Reply, Request
from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread

log = logging.getLogger(__name__)


class MotorizedWorker(ThreadedWorker):
    """Base for a single-axis motorized stage/rotator worker.

    Subclasses declare MOVE_MSG/HOME_MSG/GET_POS_MSG (their IPC request
    types) and implement the hook methods below against their own device
    object. _start/_pause/_resume/_stop stay per-subclass.
    """

    MOVE_MSG: ClassVar[type[Request]]
    HOME_MSG: ClassVar[type[Request]]
    GET_POS_MSG: ClassVar[type[Request]]

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(self.MOVE_MSG, self._on_move))
        self._unsubs.append(self._bus.subscribe(self.HOME_MSG, self._on_home))
        self._unsubs.append(self._bus.subscribe(self.GET_POS_MSG, self._on_get_pos))

    # --- hooks: subclasses implement against their own device object -----

    def _ready(self) -> bool:
        raise NotImplementedError

    def _not_ready_msg(self) -> str:
        raise NotImplementedError

    def _move_value(self, msg: Request):
        """Extract the target value (e.g. msg.position or msg.angle)."""
        raise NotImplementedError

    def _do_move(self, value) -> None:
        raise NotImplementedError

    def _do_home(self) -> None:
        raise NotImplementedError

    def _read_value(self):
        raise NotImplementedError

    def _pos_update_msg(self, value) -> Message:
        raise NotImplementedError

    def _pos_reply_msg(self, value, request_id: str) -> Reply:
        raise NotImplementedError

    # --- shared control flow ---------------------------------------------

    @worker_thread
    def _on_move(self, msg: Request) -> None:
        if not self._ready():
            self._reply_error(msg, self._not_ready_msg())
            return
        try:
            self._do_move(self._move_value(msg))
            self._notify(self._pos_update_msg(self._read_value()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("%s: move failed", type(self).__name__)
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_home(self, msg: Request) -> None:
        if not self._ready():
            self._reply_error(msg, self._not_ready_msg())
            return
        try:
            self._do_home()
            self._notify(self._pos_update_msg(self._read_value()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("%s: home failed", type(self).__name__)
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_get_pos(self, msg: Request) -> None:
        if not self._ready():
            self._reply_error(msg, self._not_ready_msg())
            return
        try:
            self._reply(self._pos_reply_msg(self._read_value(), msg.id))
        except Exception as exc:
            log.exception("%s: get position failed", type(self).__name__)
            self._reply_error(msg, str(exc))
