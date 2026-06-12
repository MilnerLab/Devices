from __future__ import annotations

from base_core.framework.shm.writer_subprocess_main import WriterSubprocessMain
from control_readout.elliptec.config import ELL14Config
from control_readout.elliptec.ell14_worker import RotatorWorker
from control_readout.esp_301.config import Esp301Config
from control_readout.esp_301.esp_301_worker import Esp301Worker


class ControlReadoutProcess(WriterSubprocessMain):
    """
    Subprocess entry point for the control readout service.

    Hosts the RotatorWorker (ELL14 half-wave plate rotator).
    The pressure buffer and worker are registered here when implemented.
    """

    def setup(self) -> None:
        self.register_worker(RotatorWorker(
            bus=self.bus,
            connector=self.connector,
            config=ELL14Config(),
            port="COM3",
        ))
        # ESP301 motion stages (probe/delay/truncation) — additive registration.
        self.register_worker(Esp301Worker(
            bus=self.bus,
            connector=self.connector,
            config=Esp301Config(),
        ))


if __name__ == "__main__":
    ControlReadoutProcess.main()
