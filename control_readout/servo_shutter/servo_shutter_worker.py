"""Servo-shutter worker — block/unblock a centrifuge arm (manual actuation for now)."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from base_core.ipc.device_worker import DeviceWorkerMixin
from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread

from control_readout.servo_shutter.config import ServoShutterConfig
from control_readout.servo_shutter.messages import ArmStateChanged, BlockArm, UnblockArm

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "servo_shutter"


class ServoShutterWorker(DeviceWorkerMixin, ThreadedWorker):
    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        config: ServoShutterConfig,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = config
        self._driver = None
        self._is_paused = False

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(BlockArm, self._on_block))
        self._unsubs.append(self._bus.subscribe(UnblockArm, self._on_unblock))

    def _start(self) -> None:
        if self._driver is None:
            self._driver = self._open_device()
        self._is_paused = False

    def _connect(self):
        # The manual driver IS the real one: a person blocks the arm, prompted by it.
        # Nothing here can fail, so this device never demotes on its own — asking for
        # the mock is the only way to get one, which is the honest state of D16.
        from control_readout.servo_shutter.manual_driver import ManualShutter

        driver = ManualShutter(self._config)
        driver.open()
        return driver

    def _connect_mock(self):
        from control_readout.servo_shutter.mock_driver import MockShutter

        driver = MockShutter(self._config)
        driver.open()
        return driver

    def _pause(self) -> None:
        self._is_paused = True

    def _resume(self) -> None:
        self._is_paused = False

    def _stop(self) -> None:
        if self._driver is not None:
            self._driver.close()
            self._driver = None
        self._is_paused = False

    @worker_thread
    def _on_block(self, msg: BlockArm) -> None:
        self._set(msg, msg.arm, blocked=True)

    @worker_thread
    def _on_unblock(self, msg: UnblockArm) -> None:
        self._set(msg, msg.arm, blocked=False)

    def _set(self, msg, arm: int, *, blocked: bool) -> None:
        if self._driver is None or self._is_paused:
            self._reply_error(msg, "Servo shutter not started or paused")
            return
        try:
            (self._driver.block if blocked else self._driver.unblock)(arm)
            self._notify(ArmStateChanged(arm=arm, blocked=blocked))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("ServoShutterWorker: set failed")
            self._reply_error(msg, str(exc))
