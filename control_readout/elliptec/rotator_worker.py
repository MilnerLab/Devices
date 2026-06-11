from __future__ import annotations

from typing import Optional

from base_core.math.enums import AngleUnit
from base_core.math.models import Angle
from base_core.framework.subprocess.worker import CommandWorker
from elliptec.config import ELL14Config
from elliptec.elliptec_ell14 import Rotator
from elliptec.messages import HomeRotator, Rotate, RotatorHomed, RotatorMoved
from messages import Message


class RotatorWorker(CommandWorker):
    """
    CommandWorker that drives a single ELL14 rotator.

    Hardware is opened in setup() when the worker thread starts and closed in
    teardown() when it stops (including on error). Each Rotate/HomeRotator
    command blocks the worker thread until motion completes.

    Emits RotatorMoved after every successful rotation,
    and RotatorHomed after homing.
    """

    name = "rotator"
    messages = [Rotate, HomeRotator]

    def __init__(self, port: str, config: ELL14Config, address: Optional[str] = None) -> None:
        super().__init__()
        self._port = port
        self._address = address
        self._config = config
        self._rotator: Optional[Rotator] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._rotator = Rotator(config=self._config)
        self._rotator.open(self._port, self._address)
        self._rotator.apply_config()

    def close(self) -> None:
        if self._rotator is not None:
            self._rotator.close()
            self._rotator = None

    def _reset(self) -> None:
        super()._reset()        # clears command queue
        self._rotator = None    # close() already ran before _reset() on restart

    # ------------------------------------------------------------------
    # Command handling
    # ------------------------------------------------------------------

    def handle(self, msg: Message, request_id: Optional[str]) -> None:
        if isinstance(msg, Rotate):
            self._handle_rotate(msg, request_id)
        elif isinstance(msg, HomeRotator):
            self._handle_home(msg, request_id)
        else:
            super().handle(msg, request_id)

    def _handle_rotate(self, msg: Rotate, request_id: Optional[str]) -> None:
        angle = Angle(msg.angle_rad, AngleUnit.RAD, wrap=False)
        self._rotator.rotate(angle)
        self.emit(RotatorMoved(position_rad=self._rotator.current_angle.Rad))
        self.reply_ok(request_id)

    def _handle_home(self, _msg: HomeRotator, request_id: Optional[str]) -> None:
        self._rotator.home()
        self.emit(RotatorHomed())
        self.reply_ok(request_id)
