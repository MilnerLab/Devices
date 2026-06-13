"""
Real Newport 8742 picomotor driver via ``pylablib`` (Ethernet).

Not exercised in CI (needs the controller + lib). Mirrors :class:`MockPicomotor`.
Proven low-level usage in ``App_Apps/test/picomotors_ethernet_test.py``.
"""
from __future__ import annotations

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

    def position(self, axis: int) -> int:
        return int(self._require().get_position(axis=axis))

    def _require(self):
        if self._dev is None:
            raise RuntimeError("Picomotor8742: not open")
        return self._dev
