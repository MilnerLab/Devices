"""Mock shutter — the same state tracking, without asking anyone to do anything."""
from __future__ import annotations

import logging

from control_readout.servo_shutter.config import ServoShutterConfig

log = logging.getLogger(__name__)


class MockShutter:
    """Stands in for :class:`ManualShutter` when nobody is at the bench.

    The only difference is the silence, and that is the whole point: the manual driver
    prints instructions for a human to follow, and a headless or unattended run that
    emits instructions nobody will act on is worse than one that emits none, because
    the log then reads as though the arm was blocked.
    """

    def __init__(self, config: ServoShutterConfig) -> None:
        self._config = config
        self._blocked: dict[int, bool] = {arm: False for arm in config.arms}

    def open(self) -> None: ...
    def close(self) -> None: ...

    def block(self, arm: int) -> None:
        log.debug("SHUTTER (mock): arm %s blocked", arm)
        self._blocked[arm] = True

    def unblock(self, arm: int) -> None:
        log.debug("SHUTTER (mock): arm %s unblocked", arm)
        self._blocked[arm] = False

    def is_blocked(self, arm: int) -> bool:
        return self._blocked.get(arm, False)
