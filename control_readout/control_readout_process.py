from __future__ import annotations

import logging
from typing import Callable

from base_core.ipc.subprocess_main import BaseSubprocessMain
from control_readout.base.controller import Controller
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


log = logging.getLogger(__name__)

#: Newport ESP301, all three linear stages, over USB. Verified live 2026-07-19:
#: this is a TI-3410 USB bridge fixed at 921600 baud, *not* the front-panel
#: RS-232 port. There is no COM14 on this machine (defect G1).
ESP301_PORT = "COM7"

#: Thorlabs ELL14 half-wave-plate rotator, on a MosChip PCI serial port.
ELL14_PORT = "COM3"

XPS_HOST = "10.1.137.137"


class ControlReadoutProcess(BaseSubprocessMain):
    """
    Subprocess entry point for the control readout service.

    Hosts the RotatorWorker (ELL14 half-wave plate rotator), the three ESP301
    linear stages (FMS300PP, MFA-CC, UTS150CC), and the RGV100BL HWP.
    Picomotors and servo shutters are implemented but not yet registered here;
    a pressure-sensor WriterWorker will be added here when implemented.

    Failure isolation (defect G2)
    -----------------------------
    Every controller is constructed and connected inside its own try/except, and
    **every worker is registered regardless**. Previously an unreachable XPS —
    connected unconditionally, at the top of this method, before anything else —
    raised out of ``setup()``. ``BaseSubprocessMain.run()`` catches that and
    returns without connecting the socket, so the parent blocks in
    ``srv.accept()`` until a bare 10 s ``socket.timeout`` propagates out of
    ``ModuleManager.bootstrap`` and **the whole application fails to launch**
    (defect G18). One unplugged device must not take the other five down, and it
    certainly must not take down the stages, which do not depend on it.

    A worker whose controller failed to connect still registers and still answers.
    It fails at the point of use, with an ``ErrorReply`` naming the real problem,
    which is where the failure is actually diagnosable.
    """

    def setup(self) -> None:
        xps_controller = self._connect(
            "XPS",
            lambda: XPSController(XPS_HOST, username='PyControl', password='labview2python'),
        )
        esp_controller = self._connect(
            "ESP301",
            lambda: ESP301Controller(port=ESP301_PORT),
        )

        self.register_worker(ELL14RotatorWorker(
            bus=self.bus,
            connector=self.connector,
            port=ELL14_PORT,
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

    @staticmethod
    def _connect(label: str, make: Callable[[], Controller]) -> Controller:
        """Build a controller and try to connect it. Never raises.

        Returns the controller whether or not the connection succeeded, so its
        workers can still be registered. An unconnected controller raises
        ``ControllerError`` from ``_require_connected()`` on first use, which the
        worker turns into an ``ErrorReply`` — a diagnosable failure at the point
        of use rather than a dead subprocess at boot.
        """
        controller = make()
        try:
            controller.connect()
            log.info("%s: connected (%r)", label, controller)
        except Exception:
            log.exception(
                "%s: connect failed — registering its workers anyway; commands to "
                "this device will return an error until it is reachable", label,
            )
        return controller


if __name__ == "__main__":
    ControlReadoutProcess.main()
