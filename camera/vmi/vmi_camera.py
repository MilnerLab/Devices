from __future__ import annotations

import time

import PySpin

from base_core.quantities.enums import Prefix
from camera.base.camera import Camera
from camera.base.exceptions import CameraError
from camera.base.models import FrameData

_INCOMPLETE_RETRY_LIMIT = 5

# Hardware trigger wiring for the VMI setup: a frame starts on a rising edge on Line0.
_TRIGGER_SELECTOR = "FrameStart"
_TRIGGER_SOURCE = "Line0"
_TRIGGER_ACTIVATION = "RisingEdge"

# PySpin's errorcode for GetNextImage running out of time. getattr because the constant
# is not exported by every PySpin build.
_ERR_TIMEOUT = getattr(PySpin, "SPINNAKER_ERR_TIMEOUT", -1011)


def _fit(node, value: int) -> int:
    """Clamp an integer node's value to its range and round down to its increment.

    The ROI nodes only take multiples of their increment (e.g. 4 or 8 px for Width) and
    raise on anything else, so the value from the config is snapped rather than sent.
    """
    lo, hi, inc = node.GetMin(), node.GetMax(), max(1, node.GetInc())
    value = max(lo, min(hi, int(value)))
    return value - (value - lo) % inc


def _set_enum(nodemap, name: str, entry: str) -> None:
    """Set an enumeration node by entry name, or say which node refused."""
    node = PySpin.CEnumerationPtr(nodemap.GetNode(name))
    if not (PySpin.IsReadable(node) and PySpin.IsWritable(node)):
        raise CameraError(f"{name} node is not writable.")
    value = node.GetEntryByName(entry)
    if not PySpin.IsReadable(value):
        raise CameraError(f"{name} has no entry {entry!r}.")
    node.SetIntValue(value.GetValue())


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

        # Offsets to 0, then size, then offsets: each node's max depends on the others'
        # current values, so in any other order a requested offset is clamped against the
        # previous ROI's width (or a width against the previous offset).
        if cam.OffsetX.GetAccessMode() == PySpin.RW:
            cam.OffsetX.SetValue(cam.OffsetX.GetMin())
        if cam.OffsetY.GetAccessMode() == PySpin.RW:
            cam.OffsetY.SetValue(cam.OffsetY.GetMin())
        if cam.Width.GetAccessMode() == PySpin.RW:
            cam.Width.SetValue(_fit(cam.Width, cfg.width))
        if cam.Height.GetAccessMode() == PySpin.RW:
            cam.Height.SetValue(_fit(cam.Height, cfg.height))
        if cam.OffsetX.GetAccessMode() == PySpin.RW:
            cam.OffsetX.SetValue(_fit(cam.OffsetX, cfg.offset_x))
        if cam.OffsetY.GetAccessMode() == PySpin.RW:
            cam.OffsetY.SetValue(_fit(cam.OffsetY, cfg.offset_y))

        self._set_exposure(cfg)
        self._set_gain(cfg)

        if cfg.pixel_format and cam.PixelFormat.GetAccessMode() == PySpin.RW:
            pixel_format_value = getattr(PySpin, f"PixelFormat_{cfg.pixel_format}", None)
            if pixel_format_value is None:
                raise CameraError(f"Unknown pixel format: {cfg.pixel_format!r}")
            cam.PixelFormat.SetValue(pixel_format_value)

        self._set_trigger(cfg.trigger_mode)

        cam.BeginAcquisition()

    def set_trigger_mode(self, enabled: bool) -> None:
        """Switch between triggered and free-running on an open, streaming camera.

        Ends and restarts the stream around the change, since the trigger nodes are not
        writable mid-acquisition. The caller must not be inside acquire_frame() -- that
        is what VmiCameraWorker pauses its acquisition loop for.
        """
        if not self._is_open:
            raise CameraError("set_trigger_mode() called before open().")
        try:
            self._cam.EndAcquisition()
        except PySpin.SpinnakerException:
            pass  # not streaming yet
        self._set_trigger(enabled)
        self._cam.BeginAcquisition()
        self.config.trigger_mode = enabled

    def _set_trigger(self, enabled: bool) -> None:
        # Mode goes Off first and On last: TriggerSelector and TriggerSource can only be
        # written while TriggerMode is Off.
        nodemap = self._cam.GetNodeMap()
        _set_enum(nodemap, "TriggerMode", "Off")
        if not enabled:
            return
        _set_enum(nodemap, "TriggerSelector", _TRIGGER_SELECTOR)
        _set_enum(nodemap, "TriggerSource", _TRIGGER_SOURCE)
        _set_enum(nodemap, "TriggerActivation", _TRIGGER_ACTIVATION)
        _set_enum(nodemap, "TriggerMode", "On")

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

    def acquire_frame(self) -> FrameData | None:
        if not self._is_open:
            raise CameraError("acquire_frame() called before open().")

        # The timeout must outlast one exposure, or a long exposure set live kills the loop.
        timeout_ms = int(max(self.config.timeout_ms, self.config.exposure_time.value(Prefix.MILLI) + 500))
        for _ in range(_INCOMPLETE_RETRY_LIMIT):
            try:
                img = self._cam.GetNextImage(timeout_ms)
            except PySpin.SpinnakerException as exc:
                # Triggered, a timeout only means no trigger came; the caller asks again,
                # and a frame that lands just after is kept by the stream for that call.
                # Free-running, a timeout is a real fault and still raises.
                if self.config.trigger_mode and exc.errorcode == _ERR_TIMEOUT:
                    return None
                raise
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
