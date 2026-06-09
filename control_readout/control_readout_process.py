from __future__ import annotations

from base_core.framework.subprocess.shared_memory.base_protocol import base_registry
from base_core.framework.subprocess.subprocess_app import SubprocessApp
from elliptec.config import ELL14Config
from elliptec.rotator_worker import RotatorWorker


if __name__ == "__main__":
    app = SubprocessApp(base_registry(), source="control_readout")
    app.add_worker(RotatorWorker(port="COM3", config=ELL14Config()))
    app.run()
