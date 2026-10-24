"""OpenBCI Galea adapter — Phase 1 demo: facial-EMG direct drive.

Gestures -> wheelchair (held gesture = held command, release = stop):
  jaw clench   -> drive forward
  right smirk  -> rotate right in place
  left smirk   -> rotate left in place
  relaxed      -> stop

Why a trained gesture model instead of fixed per-channel thresholds (galea.py):
a clench and a smirk both light up several of the Galea's six EMG channels,
and which channels respond depends on headset fit and the wearer's face. So
calibration records each gesture, and live decoding picks the nearest gesture
template in a per-channel log-RMS feature space. Nothing assumes which
electrode sits where.

Pipeline per poll:
  last 1 s of each EMG row -> detrend, mains band-stop, 20-100 Hz band-pass
  -> RMS over the last 0.25 s -> log10 -> nearest gesture centroid
  (distance scaled by per-channel spread) -> reject/ambiguity gate
  -> debounce -> Intent.
Safety bias: anything uncertain decodes as REST, and the debouncer releases
to REST faster than it engages motion.

Usage:
  ramms-access galea-p0                # live channel levels (galea_phase_0.py), nothing sent
  ramms-access galea-p1 --calibrate    # record REST/CLENCH/SMIRK_L/SMIRK_R
  ramms-access galea-p1                # drive the sim
Add --synthetic to use BrainFlow's synthetic board (no headset) for a
pipeline smoke test; its signals carry no gestures.

Requires: pip install "ramms-access[galea]"  (brainflow, numpy)
"""

from __future__ import annotations

import json
import pathlib
import time
from dataclasses import dataclass
import numpy as np
from brainflow.board_shim import BoardIds, BoardShim, BrainFlowInputParams
from brainflow.data_filter import DataFilter, DetrendOperations, FilterTypes
from ramms_access.adapters.base import Adapter
from ramms_access.intents import Intent

MODEL_FILE = pathlib.Path.home() / ".ramms_access" / "galea_phase1.json"

REST = "rest"
CLENCH = "clench"
SMIRK_LEFT = "smirk_left"
SMIRK_RIGHT = "smirk_right"
GESTURES = [REST, CLENCH, SMIRK_LEFT, SMIRK_RIGHT]

PROMPTS = {
    REST: "RELAX your whole face (teeth apart, jaw loose)",
    CLENCH: "CLENCH your jaw (teeth together, both sides) and hold",
    SMIRK_LEFT: "SMIRK with the LEFT corner of your mouth and hold",
    SMIRK_RIGHT: "SMIRK with the RIGHT corner of your mouth and hold",
}

N_EMG = 6               # the Galea's EMG channel count
BUFFER_S = 1.0          # filtered history per poll (keeps filter transients out of the window)
WINDOW_S = 0.25         # RMS window
BAND_HZ = (20.0, 100.0) # surface-EMG band that fits under the 125 Hz Nyquist
SIGMA_FLOOR = 0.05      # log10 units; stops one ultra-quiet channel dominating the distance
AMBIGUITY_RATIO = 1.3   # second-best motion gesture this close to the best -> REST
SETTLE_S = 0.75         # dropped at the start of each calibration hold (reaction + filter settle)
STEP_S = 0.05           # calibration window hop
FORWARD_SPEED = 1.0     # forward speed
TURN_SPEED = 1.0        # turning speed
CALIBRATION_ROUNDS = 2  # number of rounds of calibration that is run
CALIBRATION_SECS = 5.0  # the number of seconds each step of calibration runs for



# --- signal processing ------------------------------------------------------
def filter_emg(data: np.ndarray, rows: list[int], fs: int) -> np.ndarray:
    """Mains band-stop + EMG band-pass on each row; returns (len(rows), n) in uV.

    Mains hum sits inside the EMG band. BrainFlow's remove_environmental_noise
    only takes ~15 dB off it, so a strong hum (common with a loose electrode)
    left ~40 uV at rest, enough to decode as a clench. A 4th-order band-stop
    at mains +/- 2 Hz brings it down to the noise floor.
    """
    out = np.empty((len(rows), data.shape[1]))
    for i, row in enumerate(rows):
        x = np.ascontiguousarray(data[row], dtype=np.float64)
        DataFilter.detrend(x, DetrendOperations.CONSTANT.value)
        DataFilter.perform_bandstop(x, fs, 60 - 2.0, 60 + 2.0, 4,
                                    FilterTypes.BUTTERWORTH.value, 0.0)
        DataFilter.perform_bandpass(x, fs, BAND_HZ[0], BAND_HZ[1], 4,
                                    FilterTypes.BUTTERWORTH.value, 0.0)
        out[i] = x
    return out


def window_rms(filtered: np.ndarray) -> np.ndarray:
    return np.sqrt((filtered * filtered).mean(axis=1))


def features(filtered: np.ndarray) -> np.ndarray:
    """log10 RMS per channel: gesture strength varies multiplicatively, so log
    makes 'twice as hard' the same step on a quiet channel and a loud one."""
    return np.log10(window_rms(filtered) + 1e-3)


# --- gesture model ----------------------------------------------------------
@dataclass
class GestureModel:
    labels: list[str]
    means: np.ndarray   # (gestures, channels) log10 RMS centroids
    sigma: np.ndarray   # (channels,) pooled within-gesture spread
    reject: float       # motion gestures farther than this decode as REST

    def distances(self, f: np.ndarray) -> np.ndarray:
        return (((f - self.means) / self.sigma) ** 2).mean(axis=1)

    def predict(self, f: np.ndarray) -> str:
        d = self.distances(f)
        order = np.argsort(d)
        best, second = self.labels[order[0]], self.labels[order[1]]
        if best == REST:
            return REST
        if d[order[0]] > self.reject:
            return REST  # unlike anything seen in calibration
        if second != REST and d[order[1]] < AMBIGUITY_RATIO * d[order[0]]:
            return REST  # e.g. clench vs smirk too close to call: don't guess
        return best

    @staticmethod
    def fit(samples: dict[str, np.ndarray]) -> "GestureModel":
        labels = [g for g in GESTURES if g in samples]
        means = np.stack([samples[g].mean(axis=0) for g in labels])
        var = np.stack([samples[g].var(axis=0) for g in labels]).mean(axis=0)
        sigma = np.maximum(np.sqrt(var), SIGMA_FLOOR)
        model = GestureModel(labels, means, sigma, reject=np.inf)
        own = [model.distances(f)[i] for i, g in enumerate(labels) for f in samples[g]]
        model.reject = float(2.0 * np.percentile(own, 99))
        return model

    def to_json(self) -> dict:
        return {"labels": self.labels, "means": self.means.tolist(),
                "sigma": self.sigma.tolist(), "reject": self.reject}

    @staticmethod
    def from_json(d: dict) -> "GestureModel":
        return GestureModel(d["labels"], np.asarray(d["means"]),
                            np.asarray(d["sigma"]), float(d["reject"]))


class Debouncer:
    """A new gesture must persist before it takes effect. Release to REST
    needs fewer polls than engaging motion: stopping late is worse than
    starting late."""

    def __init__(self, onset_polls: int, release_polls: int):
        self.onset = max(1, onset_polls)
        self.release = max(1, release_polls)
        self.state = REST
        self._candidate: str | None = None
        self._count = 0

    def update(self, label: str) -> str:
        if label == self.state:
            self._candidate, self._count = None, 0
            return self.state
        if label != self._candidate:
            self._candidate, self._count = label, 0
        self._count += 1
        if self._count >= (self.release if label == REST else self.onset):
            self.state, self._candidate, self._count = label, None, 0
        return self.state


# --- adapter ----------------------------------------------------------------
class GaleaPhase1Adapter(Adapter):
    name = "galea-p1"

    def __init__(self, publisher, rate_hz: float = 50.0, ip: str = "",
                onset_s: float = 0.1, release_s: float = 0.04):
        super().__init__(publisher, rate_hz)
        self.ip = ip
        self.debouncer = Debouncer(round(onset_s * self.rate_hz), round(release_s * self.rate_hz))
        self.board: BoardShim | None = None
        self.model: GestureModel | None = None
        self._rows: list[int] = []
        self._fs = 250
        self._last_print = 0.0

    # --- device -----------------------------------------------------------
    def _connect(self) -> None:
        params = BrainFlowInputParams()
        if self.ip:
            params.ip_address = self.ip
        board_id = BoardIds.GALEA_BOARD.value
        self.board = BoardShim(board_id, params)
        self.board.prepare_session()
        self.board.start_stream()
        self._rows = BoardShim.get_emg_channels(board_id)[:N_EMG]
        self._fs = BoardShim.get_sampling_rate(board_id)
        print(f"[galea-p1] streaming 'Galea': "
              f"emg_rows={self._rows} rate={self._fs}Hz")

    def _load_model(self) -> None:
        if MODEL_FILE.exists():
            self.model = GestureModel.from_json(json.loads(MODEL_FILE.read_text()))
            print(f"[galea-p1] gesture model from {MODEL_FILE}")
        else:
            raise SystemExit(f"[galea-p1] no gesture model at {MODEL_FILE} — "
                             f"run 'ramms-access galea-p1 --calibrate' with the headset on")

    def _live_window(self) -> np.ndarray | None:
        n = int(BUFFER_S * self._fs)
        data = self.board.get_current_board_data(n)
        if data.shape[1] < n:
            return None
        return filter_emg(data, self._rows, self._fs)[:, -int(WINDOW_S * self._fs):]

    def start(self) -> None:
        self._load_model()
        self._connect()

    def stop(self) -> None:
        if self.board is not None:
            try:
                self.board.stop_stream()
                self.board.release_session()
            except Exception:
                pass
            self.board = None

    # --- control ----------------------------------------------------------
    def poll(self) -> Intent | None:
        win = self._live_window()
        if win is None:
            return None
        raw = self.model.predict(features(win))
        state = self.debouncer.update(raw)

        now = time.monotonic()
        if now - self._last_print > 0.25:
            self._last_print = now
            levels = " ".join(f"{v:6.1f}" for v in window_rms(win))
            print(f"\r[galea-p1] {state.upper():<12} (raw {raw:<11}) uV: {levels}   ",
                  end="", flush=True)

        if state == CLENCH:
            return Intent(drive=(0.0, FORWARD_SPEED))
        if state == SMIRK_RIGHT:
            return Intent(drive=(TURN_SPEED, 0.0))
        if state == SMIRK_LEFT:
            return Intent(drive=(-TURN_SPEED, 0.0))
        return Intent(drive=(0.0, 0.0))

    # --- calibration ------------------------------------------------------
    def calibrate(self) -> None:
        """Record each gesture multiple times, fit the gesture model, save it."""
        self._connect()
        win_n, step_n, settle_n = (int(s * self._fs) for s in (WINDOW_S, STEP_S, SETTLE_S))
        samples: dict[str, list[np.ndarray]] = {g: [] for g in GESTURES}
        try:
            time.sleep(1.0)
            for r in range(CALIBRATION_ROUNDS):
                for g in GESTURES:
                    input(f"\n[galea-p1] round {r + 1}/{CALIBRATION_ROUNDS}: {PROMPTS[g]} for "
                          f"{CALIBRATION_SECS:.0f}s — press Enter, then start")
                    self.board.get_board_data()
                    time.sleep(CALIBRATION_SECS)
                    data = self.board.get_board_data()
                    filt = filter_emg(data, self._rows, self._fs)[:, settle_n:]
                    for s in range(0, filt.shape[1] - win_n + 1, step_n):
                        samples[g].append(features(filt[:, s:s + win_n]))
                    print(f"[galea-p1]   ok — mean uV: "
                          f"{' '.join(f'{v:.1f}' for v in window_rms(filt))}")
        finally:
            self.stop()

        arrays = {g: np.stack(v) for g, v in samples.items() if v}
        if len(arrays) != len(GESTURES):
            print("[galea-p1] calibration incomplete — nothing saved")
            return
        model = GestureModel.fit(arrays)

        print("\n[galea-p1] how well each gesture is recognized on its own recordings:")
        weak = False
        for g in GESTURES:
            preds = [model.predict(f) for f in arrays[g]]
            acc = sum(p == g for p in preds) / len(preds)
            confused = {p: preds.count(p) for p in set(preds) if p != g}
            print(f"  {g:<12} {acc:6.1%}   confused with: {confused or '-'}")
            weak |= acc < 0.9
        if weak:
            print("[galea-p1] WARNING: some gestures are hard to tell apart. Check electrode "
                  "contact, make the gestures more distinct, and re-run --calibrate. "
                  "Unrecognized gestures decode as REST (chair stops).")

        MODEL_FILE.parent.mkdir(parents=True, exist_ok=True)
        payload = model.to_json() | {"emg_rows": self._rows, "rate": self._fs,
                                     "created": time.time()}
        MODEL_FILE.write_text(json.dumps(payload, indent=2))
        print(f"[galea-p1] saved gesture model -> {MODEL_FILE}")
