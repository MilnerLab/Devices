"""RGV100BL rotation worker (HWP) — command-style, notifies angle after each move."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

from base_core.ipc.connection_mode import ConnectionMode
from base_core.ipc.device_worker import DeviceWorkerMixin
from base_core.ipc.threaded_worker import worker_thread
from base_core.math.models import Angle

from control_readout.base.controller_provider import ControllerProvider
from control_readout.base.motorized_worker import MotorizedWorker
from control_readout.newport_xps.rgv100bl.messages import (
    GetCurrentRGVAngle,
    HomeRGV,
    RGVAngleReply,
    RGVAngleUpdate,
    RGVSpinStateUpdate,
    RotateRGVTo,
    SpinRGV,
    StopSpinRGV,
)
from control_readout.newport_xps.rgv100bl.rgv100bl_device import RGV

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "rgv100bl"

GROUP = "GROUP1"
POSITIONER = "POSITIONER"


class Rgv100blWorker(DeviceWorkerMixin, MotorizedWorker):
    MOVE_MSG = RotateRGVTo
    HOME_MSG = HomeRGV
    GET_POS_MSG = GetCurrentRGVAngle

    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        provider: ControllerProvider,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._provider = provider
        self._rotator: Optional[RGV] = None

    def _setup(self) -> None:
        super()._setup()
        # Without these two the handle's spin request reached the subprocess and nobody
        # answered it, so the handle sat at BUSY for the life of the app.
        self._unsubs.append(self._bus.subscribe(SpinRGV, self._on_spin))
        self._unsubs.append(self._bus.subscribe(StopSpinRGV, self._on_stop_spin))

    def _start(self) -> None:
        if self._rotator is None:
            self._rotator = self._open_device()

    def _connect(self) -> RGV:
        return self._attach(self._provider.acquire(ConnectionMode.DEVICE))

    def _connect_mock(self) -> RGV:
        return self._attach(self._provider.acquire(ConnectionMode.MOCK))

    def _attach(self, controller) -> RGV:
        rotator = RGV("rot", group=GROUP, controller=controller, positioner=POSITIONER)
        rotator.start()
        rotator.initialize()
        rotator.home()  # required after initialize before any move (else XPS error -22)
        return rotator

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

    # -- continuous rotation ------------------------------------------------ #

    @worker_thread
    def _on_spin(self, msg: SpinRGV) -> None:
        if not self._ready():
            self._reply_error(msg, self._not_ready_msg())
            return
        try:
            self._rotator.spin(msg.velocity_deg_s)
        except Exception as exc:
            # A SingleAxis group refuses this. The handle rolls back its optimistic
            # "spinning" announcement on the error, so the panel does not claim a
            # plate is turning when it is not.
            log.exception("Rgv100blWorker: spin refused")
            self._reply_error(msg, str(exc))
            return
        self._notify(RGVSpinStateUpdate(spinning=True, velocity_deg_s=msg.velocity_deg_s))
        self._reply_ok(msg)

    @worker_thread
    def _on_stop_spin(self, msg: StopSpinRGV) -> None:
        if not self._ready():
            self._reply_error(msg, self._not_ready_msg())
            return
        try:
            self._rotator.stop_spin()
            self._notify(RGVSpinStateUpdate(spinning=False, velocity_deg_s=0.0))
            # The plate has settled, so its angle is meaningful again. Push it, or the
            # handle keeps discarding read-backs it believes are sampled off a spin.
            self._notify(self._pos_update_msg(self._read_value()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("Rgv100blWorker: stop spin failed")
            self._reply_error(msg, str(exc))
