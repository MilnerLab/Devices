from __future__ import annotations

import time

import PySpin

from base_core.quantities.enums import Prefix
from camera.base.camera import Camera
from camera.base.exceptions import CameraError
from camera.base.models import FrameData

_INCOMPLETE_RETRY_LIMIT = 5


class VmiCamera(Camera):
    """
    PySpin (FLIR Spinnaker) driver for the Blackfly S VMI camera.

    Ported from the App_Apps/test/cameraworker.py prototype's _open_camera /
    _acquire_loop / _cleanup, restructured to the Camera interface so it plugs
    into the generic CameraWorker without any camera-worker-side vendor code.
    """

    def __init__(self, config) -> None:
        super().__init__(config)
        self._system = None
        self._cam_list = None
        self._cam = None
        self._is_open = False

    @property
    def is_open(self) -> bool:
        return self._is_open

    def open(self) -> None:
        if self._is_open:
            return

        self._system = PySpin.System.GetInstance()
        self._cam_list = self._system.GetCameras()

        if self._cam_list.GetSize() == 0:
            self._cam_list.Clear()
            self._system.ReleaseInstance()
            self._system = None
            self._cam_list = None
            raise CameraError("No camera detected. Check USB / Spinnaker install / permissions.")

        self._cam = self._cam_list.GetByIndex(self.config.device_index)
        self._cam.Init()
        self._is_open = True

    def apply_config(self) -> None:
        if not self._is_open:
            raise CameraError("apply_config() called before open().")

        cam = self._cam
        cfg = self.config

        nodemap = cam.GetNodeMap()
        s_nodemap = cam.GetTLStreamNodeMap()

        node_bh = PySpin.CEnumerationPtr(s_nodemap.GetNode("StreamBufferHandlingMode"))
        if PySpin.IsReadable(node_bh) and PySpin.IsWritable(node_bh):
            entry = node_bh.GetEntryByName("NewestOnly")
            if PySpin.IsReadable(entry):
                node_bh.SetIntValue(entry.GetValue())

        node_am = PySpin.CEnumerationPtr(nodemap.GetNode("AcquisitionMode"))
        if PySpin.IsReadable(node_am) and PySpin.IsWritable(node_am):
            cont = node_am.GetEntryByName("Continuous")
            if PySpin.IsReadable(cont):
                node_am.SetIntValue(cont.GetValue())

        if cam.OffsetX.GetAccessMode() == PySpin.RW:
            cam.OffsetX.SetValue(max(cam.OffsetX.GetMin(), min(cam.OffsetX.GetMax(), cfg.offset_x)))
        if cam.OffsetY.GetAccessMode() == PySpin.RW:
            cam.OffsetY.SetValue(max(cam.OffsetY.GetMin(), min(cam.OffsetY.GetMax(), cfg.offset_y)))
        if cam.Width.GetAccessMode() == PySpin.RW:
            cam.Width.SetValue(max(cam.Width.GetMin(), min(cam.Width.GetMax(), cfg.width)))
        if cam.Height.GetAccessMode() == PySpin.RW:
            cam.Height.SetValue(max(cam.Height.GetMin(), min(cam.Height.GetMax(), cfg.height)))

        self._set_exposure(cfg)
        self._set_gain(cfg)

        if cfg.pixel_format and cam.PixelFormat.GetAccessMode() == PySpin.RW:
            pixel_format_value = getattr(PySpin, f"PixelFormat_{cfg.pixel_format}", None)
            if pixel_format_value is None:
                raise CameraError(f"Unknown pixel format: {cfg.pixel_format!r}")
            cam.PixelFormat.SetValue(pixel_format_value)

        cam.BeginAcquisition()

    def update_live(self, config) -> None:
        if not self._is_open:
            raise CameraError("update_live() called before open().")
        self.config = config
        # ExposureTime and Gain are writable while streaming on the Blackfly S; ROI and
        # pixel format are not, so they are left to apply_config() on the next start.
        self._set_exposure(config)
        self._set_gain(config)

    def _set_exposure(self, cfg) -> None:
        cam = self._cam
        if cam.ExposureAuto.GetAccessMode() == PySpin.RW:
            cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
            time.sleep(0.05)

        if cam.ExposureTime.GetAccessMode() == PySpin.RW:
            exp_min = float(cam.ExposureTime.GetMin())
            exp_max = float(cam.ExposureTime.GetMax())
            exp_set = max(exp_min, min(exp_max, cfg.exposure_time.value(Prefix.MICRO)))
            cam.ExposureTime.SetValue(exp_set)
        else:
            raise CameraError("ExposureTime node not writable.")

    def _set_gain(self, cfg) -> None:
        cam = self._cam
        if cam.Gain.GetAccessMode() == PySpin.RW:
            cam.Gain.SetValue(cfg.gain)

    def acquire_frame(self) -> FrameData:
        if not self._is_open:
            raise CameraError("acquire_frame() called before open().")

        # The timeout must outlast one exposure, or a long exposure set live kills the loop.
        timeout_ms = int(max(self.config.timeout_ms, self.config.exposure_time.value(Prefix.MILLI) + 500))
        for _ in range(_INCOMPLETE_RETRY_LIMIT):
            img = self._cam.GetNextImage(timeout_ms)
            if img.IsIncomplete():
                img.Release()
                continue
            frame = img.GetNDArray()
            img.Release()
            return FrameData(frame=frame.copy(), timestamp_ns=time.time_ns())

        raise CameraError(f"GetNextImage returned incomplete images {_INCOMPLETE_RETRY_LIMIT} times in a row.")

    def close(self) -> None:
        if not self._is_open:
            return

        try:
            self._cam.EndAcquisition()
        except Exception:
            pass
        try:
            self._cam.DeInit()
        except Exception:
            pass

        self._cam = None

        try:
            if self._cam_list is not None:
                self._cam_list.Clear()
        except Exception:
            pass
        self._cam_list = None

        try:
            if self._system is not None:
                self._system.ReleaseInstance()
        except Exception:
            pass
        self._system = None

        self._is_open = False
