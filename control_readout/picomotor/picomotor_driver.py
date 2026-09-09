"""
Real Newport 8742 picomotor driver via ``pylablib`` (Ethernet).

Not exercised in CI (needs the controller + lib). Mirrors :class:`MockPicomotor`.
Proven low-level usage in ``App_Apps/test/picomotors_ethernet_test.py``.
"""
from __future__ import annotations

import time

from control_readout.picomotor.config import PicomotorConfig


class Picomotor8742:
    def __init__(self, config: PicomotorConfig) -> None:
        self._config = config
        self._dev = None

    def open(self) -> None:
        from pylablib.devices import Newport

        self._dev = Newport.Picomotor8742(self._config.host)

    def close(self) -> None:
        if self._dev is not None:
            self._dev.close()
            self._dev = None

    def move_by(self, axis: int, steps: int) -> None:
        self._require().move_by(axis=axis, steps=steps)
        self.wait_for_stop(axis)

    def move_to(self, axis: int, steps: int) -> None:
        self._require().move_to(axis=axis, position=int(steps))
        self.wait_for_stop(axis)

    def zero(self, axis: int) -> None:
        """Re-reference this axis' counter to zero. Commands no motion."""
        self._require().set_position_reference(axis=axis, position=0)

    def position(self, axis: int) -> int:
        return int(self._require().get_position(axis=axis))

    def is_moving(self, axis: int) -> bool:
        return bool(self._require().is_moving(axis=axis))

    def wait_for_stop(self, axis: int, timeout_s: float = 30.0) -> bool:
        """Block until the axis settles. Returns False if it never did.

        The 8742's move commands return when the command is *accepted*, not when the
        motion completes, so a position read taken straight afterwards reports the
        pre-move count. Every move here waits, or the readout is one command stale.
        """
        deadline = time.monotonic() + timeout_s
        while self.is_moving(axis):
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return True

    def _require(self):
        if self._dev is None:
            raise RuntimeError("Picomotor8742: not open")
        return self._dev
