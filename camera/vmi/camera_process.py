from __future__ import annotations

from base_core.ipc.subprocess_main import BaseSubprocessMain
from camera.base.buffer import CameraBuffer
from camera.base.camera_worker import CameraWorker
from camera.vmi.vmi_camera import VmiCamera

WORKER_ID = "camera_vmi"


class VmiCameraProcess(BaseSubprocessMain):
    """
    Subprocess entry point for the VMI (Blackfly S) camera.

    Wires up CameraBuffer attachment and the generic CameraWorker with
    VmiCamera as its hardware driver, then delegates the IPC message loop to
    BaseSubprocessMain.

    Launched by CameraService via:
        python -m camera.vmi.camera_process <port>
    """

    def setup(self) -> None:
        self.register_buffer_class(CameraBuffer)
        self.register_worker(
            CameraWorker(
                worker_id=WORKER_ID,
                bus=self.bus,
                connector=self.connector,
                get_buffer=lambda: self.get_buffer(CameraBuffer),
                camera_factory=VmiCamera,
            )
        )


if __name__ == "__main__":
    VmiCameraProcess.main()
