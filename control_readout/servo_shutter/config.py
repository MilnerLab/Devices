from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ServoShutterConfig:
    """Per-arm centrifuge shutters (D5/D16).

    ``manual=True`` (default) uses the stub driver that assumes a human blocks the arm
    — the real servos run off an Arduino/ESP32 whose comms are TBD (**TODO**).
    Arms are addressed by integer id (0 = left arm, 1 = right arm, by convention).
    """

    arms: tuple[int, ...] = (0, 1)
    manual: bool = True
