from __future__ import annotations

from base_core.ipc.subprocess_main import BaseSubprocessMain
from control_readout.ell14.ell14_worker import ELL14RotatorWorker
from control_readout.esp_301.config import Esp301Config
from control_readout.esp_301.esp_301_worker import Esp301Worker
from control_readout.newport_xps.controller import XPSController
from control_readout.newport_xps.rgv100bl.rgv100bl_worker import Rgv100blWorker
from control_readout.picomotor.config import PicomotorConfig
from control_readout.picomotor.picomotor_worker import PicomotorWorker
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
        
        xps_controller = XPSController('10.1.137.137', username='PyControl', password='labview2python')
        xps_controller.connect()
        
        self.register_worker(ELL14RotatorWorker(
            bus=self.bus,
            connector=self.connector,
            port="COM3",
        ))
        
        self.register_worker(Rgv100blWorker(
            self.bus,
            self.connector,
            xps_controller))


if __name__ == "__main__":
    ControlReadoutProcess.main()
