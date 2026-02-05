from __future__ import annotations

import threading
from typing import Any, Dict

from spm_002.config import SpectrometerConfig
from spm_002.enums import CmdName, MsgType
from spm_002.models import SpectrumData
from spm_002.spectrometer import Spectrometer

from base_core.framework.json.json_process_base import JsonlStdioAppBase


class SpectrometerServer(JsonlStdioAppBase):
    def __init__(self) -> None:
        super().__init__()
        self._cfg = SpectrometerConfig()
        self._cfg_lock = threading.Lock()
        self._cfg_ready = threading.Event()
        self._cfg_dirty = threading.Event()

    def _cfg_snapshot(self) -> SpectrometerConfig:
        with self._cfg_lock:
            return SpectrometerConfig.from_json(self._cfg.to_json())

    def _meta(self, spectrum: SpectrumData) -> Dict[str, Any]:
        return {
            "type": MsgType.META,
            "device_index": spectrum.device_index,
            "num_pixels": len(spectrum),
            "wavelengths": spectrum.wavelengths,
        }

    def _frame(self, spectrum: SpectrumData) -> Dict[str, Any]:
        return {
            "type": MsgType.FRAME,
            "timestamp": spectrum.timestamp.isoformat(),
            "device_index": spectrum.device_index,
            "counts": spectrum.counts,
        }

    # stdin thread -> on_message
    def on_message(self, msg: dict) -> None:
        if msg.get("type") != MsgType.CMD:
            return

        name = msg.get("name")
        args = msg.get("args") or {}

        if name == CmdName.SHUTDOWN:
            self.reply(msg, {"type": MsgType.ACK, "ok": True})
            self.stop()
            return

        if name == CmdName.SET_CONFIG and isinstance(args, dict):
            with self._cfg_lock:
                self._cfg.update_from_json(args)
            self._cfg_ready.set()
            self._cfg_dirty.set()
            self.reply(msg, {"type": MsgType.ACK, "ok": True})
            return

    # acquisition loop
    def main(self, stop_event: threading.Event) -> None:
        self._cfg_ready.wait()
        if stop_event.is_set():
            return

        with Spectrometer(config=self._cfg_snapshot()) as spec:
            first = spec.acquire_spectrum()
            self.emit(self._meta(first))
            self.emit(self._frame(first))

            while not stop_event.is_set():
                if self._cfg_dirty.is_set():
                    self._cfg_dirty.clear()
                    spec.configure(self._cfg_snapshot())

                self.emit(self._frame(spec.acquire_spectrum()))


if __name__ == "__main__":
    SpectrometerServer().run()
