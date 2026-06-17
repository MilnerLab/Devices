from __future__ import annotations

from base_core.ipc.subprocess_main import BaseSubprocessMain
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


class ControlReadoutProcess(BaseSubprocessMain):
    """
    Subprocess entry point for the control readout service.

    Hosts the RotatorWorker (ELL14 half-wave plate rotator), ESP301 stages,
    RGV100BL HWP, picomotors, and servo shutters.
    A pressure-sensor WriterWorker will be added here when implemented.
    """

    def setup(self) -> None:
        self.register_worker(RotatorWorker(
            bus=self.bus,
            connector=self.connector,
            config=ELL14Config(),
            port="COM3",
        ))
        self.register_worker(Esp301Worker(
            bus=self.bus,
            connector=self.connector,
            config=Esp301Config(),
        ))
        self.register_worker(Rgv100blWorker(
            bus=self.bus,
            connector=self.connector,
            config=Rgv100Config(),
        ))
        self.register_worker(PicomotorWorker(
            bus=self.bus,
            connector=self.connector,
            config=PicomotorConfig(),
        ))
        self.register_worker(ServoShutterWorker(
            bus=self.bus,
            connector=self.connector,
            config=ServoShutterConfig(),
        ))


if __name__ == "__main__":
    ControlReadoutProcess.main()
