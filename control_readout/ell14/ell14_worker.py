from __future__ import annotations

from typing import TYPE_CHECKING

from base_core.ipc.connection_mode import ConnectionMode
from base_core.ipc.device_worker import DeviceWorkerMixin
from base_core.math.models import Angle
from control_readout.base.mock_controller import MockElliptecController
from control_readout.base.mock_params import MockMotionProfile
from control_readout.ell14.controller import ElliptecController
from control_readout.ell14.config import ELL14Config
from control_readout.ell14.device import ELL14Rotator
from control_readout.ell14.messages import (
    CurrentELL14Position,
    ELL14PositionReply,
    GetCurrentELL14Position,
    HomeELL14Rotator,
    RotateELL14,
)
from control_readout.base.motorized_worker import MotorizedWorker

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

WORKER_ID = "rotator"

#: The ELL14 addresses positions in encoder counts, not degrees, so the mock profile is
#: expressed in counts to match what ELL14Rotator actually sends down the bus.
COUNTS_PER_REV = 262144
MOCK_PROFILE = MockMotionProfile(
    lower=-4 * COUNTS_PER_REV,
    upper=4 * COUNTS_PER_REV,
    velocity=COUNTS_PER_REV / 2.0,  # about two seconds per revolution
    home_time_s=1.0,
)


class ELL14RotatorWorker(DeviceWorkerMixin, MotorizedWorker):
    MOVE_MSG = RotateELL14
    HOME_MSG = HomeELL14Rotator
    GET_POS_MSG = GetCurrentELL14Position

    def __init__(
        self,
        bus: EventBus,
        connector: SubprocessPipelineConnector,
        port: str,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = ELL14Config()
        self._port = port
        self._controller: ElliptecController | None = None
        self._rotator: ELL14Rotator | None = None
        self._is_paused = False

    def _start(self) -> None:
        if self._rotator is None:
            self._rotator = self._open_device()
        self._is_paused = False

    def _connect(self) -> ELL14Rotator:
        controller = ElliptecController(self._port)
        controller.connect()
        return self._attach(controller)

    def _connect_mock(self) -> ELL14Rotator:
        controller = MockElliptecController(
            default_profile=MOCK_PROFILE)
        controller.connect()
        return self._attach(controller)

    def _attach(self, controller) -> ELL14Rotator:
        # Held so _stop() can disconnect it: this worker owns its bus outright, unlike
        # the ESP301 stages, which share one controller through a provider.
        self._controller = controller
        address = controller.resolve_address()
        rotator = ELL14Rotator("rotator", address, controller, self._config)
        rotator.start()
        rotator.apply_config()
        return rotator

    def _pause(self) -> None:
        self._is_paused = True

    def _resume(self) -> None:
        self._is_paused = False

    def _stop(self) -> None:
        if self._rotator is not None:
            self._rotator.stop()
            self._rotator = None
        if self._controller is not None:
            self._controller.disconnect()
            self._controller = None
        self._is_paused = False

    def _ready(self) -> bool:
        return self._rotator is not None and not self._is_paused

    def _not_ready_msg(self) -> str:
        return "Rotator not started or paused!"

    def _move_value(self, msg: RotateELL14) -> Angle:
        return msg.angle

    def _do_move(self, value: Angle) -> None:
        self._rotator.rotate(value)

    def _do_home(self) -> None:
        self._rotator.home()

    def _read_value(self) -> Angle:
        return self._rotator.current_angle

    def _pos_update_msg(self, value: Angle) -> CurrentELL14Position:
        return CurrentELL14Position(angle=value)

    def _pos_reply_msg(self, value: Angle, request_id: str) -> ELL14PositionReply:
        return ELL14PositionReply(angle=value, request_id=request_id)
