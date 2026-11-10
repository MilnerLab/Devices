from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from camera.base.camera_worker import CameraWorker
from camera.base.messages import SetCameraConfig

if TYPE_CHECKING:
    from camera.vmi.vmi_camera import VmiCamera

log = logging.getLogger(__name__)


class VmiCameraWorker(CameraWorker):
    """CameraWorker for the Blackfly S, adding a live trigger-mode switch.

    The trigger nodes can only be written with the stream ended, and ending it while the
    acquisition loop is blocked in GetNextImage would fail that call and kill the loop.
    So a config whose trigger_mode differs pauses the loop, switches, and resumes it;
    everything else (exposure, gain) goes through the generic live update as before.
    """

    _camera: VmiCamera | None

    def _apply_set_config(self, msg: SetCameraConfig) -> None:
        camera = self._camera
        wanted = msg.config.trigger_mode
        if camera is not None and camera.is_open and wanted != camera.config.trigger_mode:
            producing = self._prod_handle is not None
            if producing:
                self._pause()
            try:
                camera.set_trigger_mode(wanted)
            except Exception as exc:
                log.exception("VmiCameraWorker: switching trigger mode failed")
                self._reply_error(msg, f"Trigger mode not changed: {exc}")
                return
            finally:
                if producing:
                    self._resume()
            log.debug("VmiCameraWorker: trigger mode %s", "on" if wanted else "off")
        super()._apply_set_config(msg)
