from __future__ import annotations

from base_core.ipc.subprocess_main import BaseSubprocessMain
from control_readout.ell14.ell14_worker import ELL14RotatorWorker
from control_readout.esp_301.controller import ESP301Controller
from control_readout.esp_301.fms300pp.fms300pp_worker import Fms300ppWorker
from control_readout.esp_301.mfa_cc.mfa_cc_worker import MfaccWorker
from control_readout.esp_301.uts150cc.uts150cc_worker import Uts150ccWorker
from control_readout.newport_xps.controller import XPSController
from control_readout.newport_xps.rgv100bl.rgv100bl_worker import Rgv100blWorker
from control_readout.picomotor.config import PicomotorConfig
from control_readout.picomotor.picomotor_worker import PicomotorWorker
from control_readout.servo_shutter.config import ServoShutterConfig
from control_readout.servo_shutter.servo_shutter_worker import ServoShutterWorker


class ControlReadoutProcess(BaseSubprocessMain):
    """
    Subprocess entry point for the control readout service.

    Hosts the RotatorWorker (ELL14 half-wave plate rotator), the three ESP301
    linear stages (FMS300PP, MFA-CC, UTS150CC), and the RGV100BL HWP.
    Picomotors and servo shutters are implemented but not yet registered here;
    a pressure-sensor WriterWorker will be added here when implemented.
    """

    def setup(self) -> None:

        xps_controller = XPSController('10.1.137.137', username='PyControl', password='labview2python')
        xps_controller.connect()

        esp_controller = ESP301Controller(port="COM4")  # TODO: confirm real COM port
        esp_controller.connect()

        self.register_worker(ELL14RotatorWorker(
            bus=self.bus,
            connector=self.connector,
            port="COM3",
        ))

        self.register_worker(Rgv100blWorker(
            self.bus,
            self.connector,
            xps_controller))

        self.register_worker(Fms300ppWorker(
            bus=self.bus,
            connector=self.connector,
            controller=esp_controller))

        self.register_worker(MfaccWorker(
            bus=self.bus,
            connector=self.connector,
            controller=esp_controller))

        self.register_worker(Uts150ccWorker(
            bus=self.bus,
            connector=self.connector,
            controller=esp_controller))


if __name__ == "__main__":
    ControlReadoutProcess.main()
