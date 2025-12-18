# acquisition/config_gui.py
import tkinter as tk
from tkinter import ttk
from typing import Optional

from spm_002.config import SpectrometerConfig
from spm_002.runtime_config import ConfigManager



class ConfigWindow:
    """
    Simple Tkinter window to edit SpectrometerConfig.

    - User enters exposure, average, dark subtraction, etc.
    - On "Apply / Start" a SpectrometerConfig is built and passed
      to the ConfigManager via set_config().
    """

    def __init__(self, manager: ConfigManager) -> None:
        self._manager = manager

        self._root = tk.Tk()
        self._root.title("SPM-002 Configuration (x32)")

        # Tk variables
        self._exposure_var = tk.StringVar(value="50.0")   # ms
        self._average_var = tk.StringVar(value="5")
        self._dark_var = tk.IntVar(value=0)               # 0/1
        self._mode_var = tk.StringVar(value="0")
        self._scan_delay_var = tk.StringVar(value="0")

        self._build_ui()

        # closing the window should just end mainloop()
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ #
    # UI construction
    # ------------------------------------------------------------------ #

    def _build_ui(self) -> None:
        pad = {"padx": 8, "pady": 4}

        frame = ttk.Frame(self._root)
        frame.grid(row=0, column=0, sticky="nsew")
        self._root.columnconfigure(0, weight=1)
        self._root.rowconfigure(0, weight=1)

        # Exposure
        ttk.Label(frame, text="Exposure [ms]:").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(frame, textvariable=self._exposure_var, width=12).grid(
            row=0, column=1, sticky="w", **pad
        )

        # Average
        ttk.Label(frame, text="Average:").grid(row=1, column=0, sticky="w", **pad)
        ttk.Entry(frame, textvariable=self._average_var, width=12).grid(
            row=1, column=1, sticky="w", **pad
        )

        # Dark subtraction
        ttk.Checkbutton(
            frame,
            text="Dark subtraction",
            variable=self._dark_var,
        ).grid(row=2, column=0, columnspan=2, sticky="w", **pad)

        # Mode
        ttk.Label(frame, text="Mode:").grid(row=3, column=0, sticky="w", **pad)
        ttk.Entry(frame, textvariable=self._mode_var, width=12).grid(
            row=3, column=1, sticky="w", **pad
        )

        # Scan delay
        ttk.Label(frame, text="Scan delay:").grid(row=4, column=0, sticky="w", **pad)
        ttk.Entry(frame, textvariable=self._scan_delay_var, width=12).grid(
            row=4, column=1, sticky="w", **pad
        )

        # Buttons
        button_frame = ttk.Frame(frame)
        button_frame.grid(row=5, column=0, columnspan=2, sticky="ew", **pad)
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)

        ttk.Button(
            button_frame,
            text="Apply / Start",
            command=self._on_apply,
        ).grid(row=0, column=0, sticky="ew", padx=4)

        ttk.Button(
            button_frame,
            text="Close",
            command=self._on_close,
        ).grid(row=0, column=1, sticky="ew", padx=4)

    # ------------------------------------------------------------------ #
    # Event handlers
    # ------------------------------------------------------------------ #

    def _parse_float(self, value: str, default: float) -> float:
        try:
            return float(value.replace(",", "."))
        except ValueError:
            return default

    def _parse_int(self, value: str, default: int) -> int:
        try:
            return int(value)
        except ValueError:
            return default

    def _on_apply(self) -> None:
        """
        Build a SpectrometerConfig from the current UI values and
        send it to the ConfigManager.
        """
        exposure_ms = self._parse_float(self._exposure_var.get(), 50.0)
        average = self._parse_int(self._average_var.get(), 1)
        dark_sub = 1 if self._dark_var.get() else 0
        mode = self._parse_int(self._mode_var.get(), 0)
        scan_delay = self._parse_int(self._scan_delay_var.get(), 0)

        cfg = SpectrometerConfig(
            device_index=0,
            exposure_ms=exposure_ms,
            average=average,
            dark_subtraction=dark_sub,
            mode=mode,
            scan_delay=scan_delay,
        )

        self._manager.set_config(cfg)

    def _on_close(self) -> None:
        """
        Close the window; mainloop() will return.
        """
        self._root.destroy()

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def run(self) -> None:
        """
        Run the Tk main loop. This should be called from the main thread.
        """
        self._root.mainloop()
