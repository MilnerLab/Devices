from __future__ import annotations

import logging
import threading

from base_core.ipc.message import OKReply
from base_core.ipc.subprocess_main import BaseSubprocessMain
from control_readout.base.controller_provider import ControllerProvider
from control_readout.base.mock_controller import MockESP301Controller, MockXPSController
from control_readout.ell14.ell14_worker import ELL14RotatorWorker
from control_readout.esp_301.controller import ESP301Controller
from control_readout.esp_301.fms300pp.fms300pp_worker import Fms300ppWorker
from control_readout.esp_301.mfa_cc.mfa_cc_worker import MfaccWorker
from control_readout.esp_301.mock_params import MOCK_PROFILES as ESP_MOCK_PROFILES
from control_readout.esp_301.uts150cc.uts150cc_worker import Uts150ccWorker
from control_readout.messages import ReleaseHardware
from control_readout.newport_xps.controller import XPSController
from control_readout.newport_xps.rgv100bl.rgv100bl_worker import Rgv100blWorker
from control_readout.picomotor.config import PicomotorConfig
from control_readout.picomotor.picomotor_worker import PicomotorWorker
from control_readout.servo_shutter.config import ServoShutterConfig
from control_readout.servo_shutter.servo_shutter_worker import ServoShutterWorker

log = logging.getLogger(__name__)

XPS_HOST = "10.1.137.137"
#: Newport ESP301, all three linear stages, over USB. Verified live 2026-07-19:
#: this is a TI-3410 USB bridge fixed at 921600 baud, *not* the front-panel
#: RS-232 port. There is no COM14 on this machine (defect G1).
ESP301_PORT = "COM7"

#: Thorlabs ELL14 half-wave-plate rotator, on a MosChip PCI serial port.
ELL14_PORT = "COM3"

#: How long to wait for a controller's IO lock before giving up on closing its port.
LOCK_TIMEOUT_S = 5.0


class ControlReadoutProcess(BaseSubprocessMain):
    """
    Subprocess entry point for the control readout service.

    Hosts the ELL14 half-wave plate rotator, the three ESP301 linear stages, the
    RGV100BL HWP, the mirror picomotors and the servo shutters.

    Nothing connects to hardware here. Controllers are handed to workers through a
    :class:`ControllerProvider`, which connects on first use, so an absent instrument
    costs that worker a demotion to its mock rather than taking the whole subprocess
    down with it. Connecting eagerly in setup() is what used to make a missing COM4
    kill every device in the process, including the ones on other transports.
    """

    def setup(self) -> None:
        self._providers = [
            ControllerProvider(
                "ESP301",
                real_factory=lambda: ESP301Controller(port=ESP301_PORT),
                mock_factory=lambda: MockESP301Controller(profiles=ESP_MOCK_PROFILES),
            ),
            ControllerProvider(
                "XPS",
                real_factory=lambda: XPSController(
                    XPS_HOST, username="PyControl", password="labview2python"),
                mock_factory=MockXPSController,
            ),
        ]
        esp, xps = self._providers

        self._unsub_release = self.bus.subscribe(ReleaseHardware, self._on_release_hardware)

        self.register_worker(ELL14RotatorWorker(
            bus=self.bus, connector=self.connector, port=ELL14_PORT))

        self.register_worker(Rgv100blWorker(self.bus, self.connector, xps))

        for worker_cls in (Fms300ppWorker, MfaccWorker, Uts150ccWorker):
            self.register_worker(worker_cls(
                bus=self.bus, connector=self.connector, provider=esp))

        self.register_worker(PicomotorWorker(
            bus=self.bus, connector=self.connector, config=PicomotorConfig()))

        self.register_worker(ServoShutterWorker(
            bus=self.bus, connector=self.connector, config=ServoShutterConfig()))

    # -- graceful shutdown (defect G19) ------------------------------------ #

    @property
    def _controllers(self) -> list:
        return [c for p in getattr(self, "_providers", []) for c in p.controllers]

    def _on_release_hardware(self, msg: ReleaseHardware) -> None:
        """Disconnect every controller, then confirm — always.

        The parent blocks on this reply before it terminates us, and on Windows that
        termination is an uncatchable TerminateProcess. So a failure to close one port
        must not stop the others, and must not withhold the reply: a parent that never
        hears back kills us anyway, and then every port closes abruptly instead of one.
        """
        for controller in self._controllers:
            try:
                self._disconnect_quiescent(controller)
            except Exception:
                log.exception("ReleaseHardware: failed to disconnect %r", controller)
        self.connector.send(OKReply(request_id=msg.id))

    @staticmethod
    def _disconnect_quiescent(controller, lock_timeout: float = LOCK_TIMEOUT_S) -> None:
        """Close a controller's port, but only while no command holds its IO lock.

        Taking the lock first is the whole point. Closing a serial port out from under
        an in-flight command is what wedges the ESP301's TI-3410 USB bridge, and a
        wedged bridge outlives the process. If the lock cannot be had in time we leave
        the port open and let the OS reclaim it, which is the lesser harm.
        """
        lock = getattr(controller, "_lock", None)
        if lock is None:
            controller.disconnect()
            return
        if not lock.acquire(timeout=lock_timeout):
            log.warning(
                "ReleaseHardware: %r is still busy after %.1fs; leaving its port open "
                "rather than closing it mid-command", controller, lock_timeout)
            return
        try:
            controller.disconnect()
        finally:
            lock.release()


if __name__ == "__main__":
    ControlReadoutProcess.main()
