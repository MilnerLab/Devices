"""
Manual shutter driver — tracks per-arm blocked state and prompts a human to do it.

This is the **real** driver, not a mock, and the distinction matters. The blocking does
happen in the lab; a person performs it, prompted by the warning below. Real servo
actuation over Arduino/ESP32 is a TODO (D16) and will replace the prompt with a wire,
at which point this file becomes the mock it currently only resembles.
"""
from __future__ import annotations

import logging

from control_readout.servo_shutter.config import ServoShutterConfig

log = logging.getLogger(__name__)


class ManualShutter:
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
