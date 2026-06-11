from __future__ import annotations

from base_core.framework.subprocess.shared_memory.shared_memory_base_messages import base_registry
from base_core.framework.subprocess.subprocess_app import SubprocessApp
from elliptec.config import ELL14Config
from elliptec.messages import HomeRotator, Rotate, RotatorHomed, RotatorMoved
from elliptec.rotator_worker import RotatorWorker


if __name__ == "__main__":
    app = SubprocessApp(
        base_registry().extend(Rotate, HomeRotator, RotatorMoved, RotatorHomed),
        source="control_readout",
    )
    app.add_worker(RotatorWorker(port="COM3", config=ELL14Config()))
    app.run()
