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
        # True between an accepted SpinRGV and the stop that ends it. The worker owns this
        # rather than polling the controller: every path that commands a position has to
        # stop the spin first, and that check must not depend on the network being up.
        self._spinning = False

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
            # Ending the spin first matters: abort() on a spinning group is a hard stop,
            # and pausing the worker must not shock-load the optic on the plate.
            self._end_spin()
            self._rotator.abort()

    def _resume(self) -> None:
        if self._rotator is None:
            self._start()

    def _stop(self) -> None:
        if self._rotator is not None:
            self._end_spin()
            self._rotator.stop()
            self._rotator = None

    def _ready(self) -> bool:
        return self._rotator is not None

    def _not_ready_msg(self) -> str:
        return "RGV100BL not started"

    def _move_value(self, msg: RotateRGVTo) -> Angle:
        return msg.angle

    def _do_move(self, value: Angle) -> None:
        # A position command against a spinning group is refused by the XPS. Stopping
        # here rather than erroring means a move always wins over a spin, which is the
        # required precedence: an explicit command overrides the free-running mode.
        self._end_spin()
        self._rotator.rotate(value)

    def _do_home(self) -> None:
        self._end_spin()
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
            # The likeliest cause is a group that is not a SpindleAxis; say so, because the
            # controller's own error text for it is not obviously about that. The handle
            # rolls back its optimistic "spinning" announcement on the error, so the panel
            # does not claim a plate is turning when it is not.
            log.exception("Rgv100blWorker: spin refused")
            self._reply_error(msg, f"{exc} (is the group configured as a SpindleAxis?)")
            return
        self._spinning = True
        self._notify(RGVSpinStateUpdate(spinning=True,
                                        velocity_deg_s=float(msg.velocity_deg_s)))
        self._reply_ok(msg)

    @worker_thread
    def _on_stop_spin(self, msg: StopSpinRGV) -> None:
        if not self._ready():
            self._reply_error(msg, self._not_ready_msg())
            return
        try:
            self._end_spin()
            # The plate has settled, so its angle is meaningful again. Push it, or the
            # handle keeps discarding read-backs it believes are sampled off a spin.
            self._notify(self._pos_update_msg(self._read_value()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("Rgv100blWorker: stop spin failed")
            self._reply_error(msg, str(exc))

    def _end_spin(self) -> None:
        """Ramp any running spin to a stop. Safe to call when not spinning.

        Every path out of the spinning state goes through here, including the lifecycle
        ones, so there is no way to leave the worker with the plate still turning.
        """
        if not self._spinning or self._rotator is None:
            return
        self._spinning = False
        try:
            self._rotator.stop_spin()
        except Exception:
            log.exception("Rgv100blWorker: stopping the spin failed")
        self._notify(RGVSpinStateUpdate(spinning=False, velocity_deg_s=0.0))
