from __future__ import annotations

from typing import Optional

from base_core.math.enums import AngleUnit
from base_core.math.models import Angle
from base_core.framework.subprocess.worker import Worker
from elliptec.config import ELL14Config
from elliptec.elliptec_ell14 import Rotator
from elliptec.messages import HomeRotator, Rotate, RotatorHomed, RotatorMoved
from messages import Message


class RotatorWorker(Worker):
    """
    Worker that drives a single ELL14 rotator.

    Hardware is opened in start() when the worker thread begins and closed in
    close() when it stops (including on error). Each Rotate/HomeRotator
    command blocks the worker thread until motion completes.
    """

    name = "rotator"
    bus_messages = (Rotate, HomeRotator)

    def __init__(self, port: str, config: ELL14Config, address: Optional[str] = None) -> None:
        super().__init__()
        self._port = port
        self._address = address
        self._config = config
        self._rotator: Optional[Rotator] = None

    def start(self) -> None:
        self._rotator = Rotator(config=self._config)
        self._rotator.open(self._port, self._address)
        self._rotator.apply_config()

    def close(self) -> None:
        if self._rotator is not None:
            self._rotator.close()
            self._rotator = None

    def _reset(self) -> None:
        super()._reset()
        self._rotator = None

    def handle(self, msg: Message) -> None:
        if isinstance(msg, Rotate):
            angle = Angle(msg.angle_rad, AngleUnit.RAD, wrap=False)
            self._rotator.rotate(angle)
            self.emit(RotatorMoved(position_rad=self._rotator.current_angle.Rad))
            self.reply_ok(msg.request_id)
        elif isinstance(msg, HomeRotator):
            self._rotator.home()
            self.emit(RotatorHomed())
            self.reply_ok(msg.request_id)
