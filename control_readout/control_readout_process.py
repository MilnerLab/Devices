from __future__ import annotations

from base_core.framework.shm.writer_subprocess_main import WriterSubprocessMain
from control_readout.elliptec.config import ELL14Config
from control_readout.elliptec.ell14_worker import RotatorWorker
from control_readout.esp_301.config import Esp301Config
from control_readout.esp_301.esp_301_worker import Esp301Worker
from control_readout.picomotor.config import PicomotorConfig
from control_readout.picomotor.picomotor_worker import PicomotorWorker
from control_readout.rgv100bl.config import Rgv100Config
from control_readout.rgv100bl.rgv100bl_worker import Rgv100blWorker
from control_readout.servo_shutter.config import ServoShutterConfig
from control_readout.servo_shutter.servo_shutter_worker import ServoShutterWorker


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
        # RGV100BL HWP rotator (replaces ELL14 for HWP, D11).
        self.register_worker(Rgv100blWorker(
            bus=self.bus,
            connector=self.connector,
            config=Rgv100Config(),
        ))
        # Mirror picomotors (manual, no PID).
        self.register_worker(PicomotorWorker(
            bus=self.bus,
            connector=self.connector,
            config=PicomotorConfig(),
        ))
        # Centrifuge-arm servo shutters (manual stub for now, D16).
        self.register_worker(ServoShutterWorker(
            bus=self.bus,
            connector=self.connector,
            config=ServoShutterConfig(),
        ))


if __name__ == "__main__":
    ControlReadoutProcess.main()
