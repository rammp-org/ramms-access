"""OpenBCI Galea adapter (BrainFlow) — v0: EMG-driven control.

v0 strategy (robust, low-latency, minimal calibration): facial EMG.
  - left / right jaw or temple clench  -> turn left / right
  - sustained bilateral clench        -> forward
  - double blink (frontal channels)   -> gripper toggle   [TODO]
EEG decoding (SSVEP / motor imagery) layers on later behind the same
adapter; recorded raw sessions feed offline decoder development.

Calibration: run with --calibrate to record RELAX and CLENCH windows and
persist per-channel RMS thresholds; without a calibration file, conservative
defaults are used and printed so you can eyeball signal quality first.

Requires: pip install "ramms-access[galea]"  (brainflow)
"""

from __future__ import annotations

import json
import pathlib
import time

from ramms_access.adapters.base import Adapter
from ramms_access.intents import Intent

CALIBRATION_FILE = pathlib.Path.home() / ".ramms_access" / "galea_emg.json"


class GaleaAdapter(Adapter):
    name = "galea"

    def __init__(self, publisher, rate_hz: float = 25.0, ip: str = "",
                 left_ch: int = 0, right_ch: int = 1, window_s: float = 0.2):
        super().__init__(publisher, rate_hz)
        self.ip = ip
        self.left_ch = left_ch
        self.right_ch = right_ch
        self.window_s = window_s
        self.board = None
        self._emg_rows: list[int] = []
        self._rate = 250
        self._thresholds = {"on": 25.0, "off": 15.0}  # uV RMS, hysteresis pair
        self._left_active = False
        self._right_active = False

    def start(self) -> None:
        try:
            from brainflow.board_shim import BoardIds, BoardShim, BrainFlowInputParams
        except ImportError as e:
            raise SystemExit(
                "galea adapter needs brainflow: pip install 'ramms-access[galea]'") from e

        if CALIBRATION_FILE.exists():
            self._thresholds = json.loads(CALIBRATION_FILE.read_text())
            print(f"[galea] thresholds from {CALIBRATION_FILE}: {self._thresholds}")
        else:
            print(f"[galea] no calibration at {CALIBRATION_FILE} — using defaults "
                  f"{self._thresholds}; run 'ramms-access galea --calibrate' with the "
                  f"headset donned")

        params = BrainFlowInputParams()
        if self.ip:
            params.ip_address = self.ip
        board_id = BoardIds.GALEA_BOARD.value
        self.board = BoardShim(board_id, params)
        self.board.prepare_session()
        self.board.start_stream()
        self._emg_rows = BoardShim.get_emg_channels(board_id) or BoardShim.get_exg_channels(board_id)
        self._rate = BoardShim.get_sampling_rate(board_id)
        print(f"[galea] streaming: emg_rows={self._emg_rows} rate={self._rate}Hz")

    def _rms(self, data, row: int) -> float:
        import numpy as np  # brainflow depends on numpy, so this is available
        x = data[row]
        if x.size == 0:
            return 0.0
        x = x - x.mean()
        return float(np.sqrt((x * x).mean()))

    def poll(self) -> Intent | None:
        n = int(self.window_s * self._rate)
        data = self.board.get_current_board_data(max(n, 8))
        if data.shape[1] < 8:
            return None

        left = self._rms(data, self._emg_rows[self.left_ch])
        right = self._rms(data, self._emg_rows[self.right_ch])

        # Schmitt-trigger per side so RMS jitter doesn't chatter the drive.
        on, off = self._thresholds["on"], self._thresholds["off"]
        self._left_active = left > on or (self._left_active and left > off)
        self._right_active = right > on or (self._right_active and right > off)

        if self._left_active and self._right_active:
            return Intent(drive=(0.0, 0.7))   # both -> forward
        if self._left_active:
            return Intent(drive=(-0.6, 0.0))  # left clench -> turn left
        if self._right_active:
            return Intent(drive=(0.6, 0.0))
        return Intent(drive=(0.0, 0.0))

    def stop(self) -> None:
        if self.board is not None:
            try:
                self.board.stop_stream()
                self.board.release_session()
            except Exception:
                pass

    # ------------------------------------------------------------------
    def calibrate(self) -> None:
        """Record RELAX and CLENCH windows, persist hysteresis thresholds."""
        import numpy as np

        def window_rms(seconds: float) -> list[float]:
            time.sleep(0.5)
            self.board.get_board_data()  # flush
            time.sleep(seconds)
            data = self.board.get_board_data()
            return [self._rms(data, self._emg_rows[c]) for c in (self.left_ch, self.right_ch)]

        self.start()
        try:
            input("[galea] RELAX for 5s — press enter to start")
            relax = window_rms(5.0)
            input("[galea] CLENCH (both sides) for 5s — press enter to start")
            clench = window_rms(5.0)
        finally:
            self.stop()

        lo = float(np.mean(relax))
        hi = float(np.mean(clench))
        if hi <= lo * 1.5:
            print(f"[galea] WARNING: weak separation (relax={lo:.1f} clench={hi:.1f}) — "
                  f"check electrode contact; thresholds not saved")
            return
        on = lo + 0.6 * (hi - lo)
        off = lo + 0.3 * (hi - lo)
        CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
        CALIBRATION_FILE.write_text(json.dumps({"on": on, "off": off}, indent=2))
        print(f"[galea] saved thresholds on={on:.1f} off={off:.1f} -> {CALIBRATION_FILE}")
