from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class FrameData:
    """
    One acquired frame from a camera.

    Internal to the boundary between a Camera subclass and CameraWorker --
    never crosses the subprocess pipe (only the raw frame written into the
    shared memory slot does).
    """
    frame: np.ndarray
    timestamp_ns: int
