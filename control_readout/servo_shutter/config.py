from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ServoShutterConfig:
    """Per-arm centrifuge shutters (D5/D16).

    Hardware only. Which driver stands behind it is a start-time decision carried by
    ``ConnectionMode``: today the real path prompts a human, and servo actuation over
    Arduino/ESP32 is still TBD (**TODO**).

    Arms are addressed by integer id (0 = left arm, 1 = right arm, by convention).
    """

    arms: tuple[int, ...] = (0, 1)
