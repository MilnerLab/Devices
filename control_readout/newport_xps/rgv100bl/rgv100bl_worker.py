"""RGV100BL rotation worker (HWP) — command-style, notifies angle after each move."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread

from base_core.math.models import Angle
from control_readout.newport_xps.controller import XPSController
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




class Rgv100blWorker(ThreadedWorker):
    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        controller: XPSController,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._controller = controller
        self._rotator: Optional[RGV] = None
        # True between an accepted SpinRGV and the stop that ends it. The worker owns this
        # rather than polling the controller: every path that commands a position has to
        # stop the spin first, and that check must not depend on the network being up.
        self._spinning = False

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(RotateRGVTo, self._on_rotate))
        self._unsubs.append(self._bus.subscribe(HomeRGV, self._on_home))
        self._unsubs.append(self._bus.subscribe(GetCurrentRGVAngle, self._on_get_angle))
        self._unsubs.append(self._bus.subscribe(SpinRGV, self._on_spin))
        self._unsubs.append(self._bus.subscribe(StopSpinRGV, self._on_stop_spin))

    def _start(self) -> None:
        if self._rotator is None:
            self._rotator = RGV("rot", group="GROUP1", controller=self._controller,positioner="POSITIONER")
            self._rotator.start()
            self._rotator.initialize()
            self._rotator.home()  # required after initialize before any move (else XPS error -22)

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

    @worker_thread
    def _on_rotate(self, msg: RotateRGVTo) -> None:
        if self._rotator is None:
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            # A position command against a spinning group is refused by the XPS. Stopping
            # here rather than erroring means a move always wins over a spin, which is the
            # required precedence: an explicit command overrides the free-running mode.
            self._end_spin()
            self._rotator.rotate(msg.angle)
            self._notify(RGVAngleUpdate(angle=self._rotator.angle()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("Rgv100blWorker: rotate failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_home(self, msg: HomeRGV) -> None:
        if self._rotator is None:
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            self._end_spin()
            self._rotator.home()
            self._notify(RGVAngleUpdate(angle=self._rotator.angle()))
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("Rgv100blWorker: home failed")
            self._reply_error(msg, str(exc))

    @worker_thread
    def _on_spin(self, msg: SpinRGV) -> None:
        if self._rotator is None:
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            self._rotator.spin(msg.velocity_deg_s)
            self._spinning = True
            self._notify(RGVSpinStateUpdate(spinning=True,
                                            velocity_deg_s=float(msg.velocity_deg_s)))
            self._reply_ok(msg)
        except Exception as exc:
            # The likeliest cause is a group that is not a SpindleAxis; say so, because the
            # controller's own error text for it is not obviously about that.
            log.exception("Rgv100blWorker: spin failed")
            self._reply_error(msg, f"{exc} (is the group configured as a SpindleAxis?)")

    @worker_thread
    def _on_stop_spin(self, msg: StopSpinRGV) -> None:
        if self._rotator is None:
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            self._end_spin()
            # Only now is there a position worth reporting -- the angle was meaningless
            # while the plate was turning, so the panel readout has been blank since the
            # spin started and this is what fills it back in.
            self._notify(RGVAngleUpdate(angle=self._rotator.angle()))
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

    @worker_thread
    def _on_get_angle(self, msg: GetCurrentRGVAngle) -> None:
        if self._rotator is None:
            self._reply_error(msg, "RGV100BL not started")
            return
        try:
            self._reply(RGVAngleReply(angle=self._rotator.angle(), request_id=msg.id))
        except Exception as exc:
            log.exception("Rgv100blWorker: get angle failed")
            self._reply_error(msg, str(exc))