"""
Stub shutter driver — tracks per-arm blocked state and prompts for MANUAL blocking.

Per D16 we build the high-level flow now; real servo actuation (Arduino/ESP32) is a
**TODO** once comms details exist. Swap this for a real driver with the same interface.
"""
from __future__ import annotations

import logging

from control_readout.servo_shutter.config import ServoShutterConfig

log = logging.getLogger(__name__)


class ManualShutterStub:
    def __init__(self, config: ServoShutterConfig) -> None:
        self._config = config
        self._blocked: dict[int, bool] = {arm: False for arm in config.arms}

    def open(self) -> None: ...
    def close(self) -> None: ...

    def block(self, arm: int) -> None:
        log.warning("SHUTTER (manual): please BLOCK arm %s", arm)
        self._blocked[arm] = True

    def unblock(self, arm: int) -> None:
        log.warning("SHUTTER (manual): please UNBLOCK arm %s", arm)
        self._blocked[arm] = False

    def is_blocked(self, arm: int) -> bool:
        return self._blocked.get(arm, False)
