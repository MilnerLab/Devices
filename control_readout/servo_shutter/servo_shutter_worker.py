"""Servo-shutter worker — block/unblock a centrifuge arm (manual stub for now)."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread

from control_readout.servo_shutter.config import ServoShutterConfig
from control_readout.servo_shutter.messages import ArmStateChanged, BlockArm, UnblockArm

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "servo_shutter"


def _make_driver(config: ServoShutterConfig):
    # Only the manual stub exists today; real Arduino/ESP32 actuation is a TODO (D16).
    from control_readout.servo_shutter.stub_driver import ManualShutterStub
    return ManualShutterStub(config)


class ServoShutterWorker(ThreadedWorker):
    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        config: ServoShutterConfig,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = config
        self._driver = None

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(BlockArm, self._on_block))
        self._unsubs.append(self._bus.subscribe(UnblockArm, self._on_unblock))

    def _start(self) -> None:
        self._driver = _make_driver(self._config)
        self._driver.open()

    def _stop(self) -> None:
        if self._driver is not None:
            self._driver.close()
            self._driver = None

    def _reset(self) -> None:
        self._stop()
        self._start()

    @worker_thread
    def _on_block(self, msg: BlockArm) -> None:
        self._set(msg, msg.arm, blocked=True)

    @worker_thread
    def _on_unblock(self, msg: UnblockArm) -> None:
        self._set(msg, msg.arm, blocked=False)

    def _set(self, msg, arm: int, *, blocked: bool) -> None:
        if self._driver is None:
            self._reply_error(msg, "Servo shutter not started")
            return
        try:
            (self._driver.block if blocked else self._driver.unblock)(arm)
            self._notify(ArmStateChanged(arm=arm, blocked=blocked))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("ServoShutterWorker: set failed")
            self._reply_error(msg, str(exc))
