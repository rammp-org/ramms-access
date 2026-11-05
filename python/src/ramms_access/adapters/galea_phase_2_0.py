"""OpenBCI Galea + Pupil Labs Neon — Phase 2.0: clench-cycled gaze steering.

Three modes; each jaw clench advances to the next (REST -> ROTATE -> DRIVE -> REST):
  REST    nothing happens: gaze is ignored, the chair is stopped
  ROTATE  gaze steers rotation in place, in real time: look right -> turn
          right, look left -> turn left, look straight ahead -> stop turning
  DRIVE   straight forward at a set speed, in whatever direction the chair faces

No chair feedback and no IMU. In ROTATE the user closes the loop: keep your
eyes on the target, and the chair turns toward it at a constant speed until
the gaze is back within the deadband. Gaze is measured relative to the head, so this
assumes the head stays roughly facing forward relative to the chair.

Clench detection uses phase 1's gesture model and Galea connection, but its
own calibration: only REST and CLENCH are recorded, saved to MODEL_FILE
(separate from phase 1's). A mode change needs a clench onset after a
release, so holding a clench advances once, not repeatedly. A clench in
DRIVE therefore stops the chair (-> REST).

Usage:
  ramms-access galea-p2.0 --ip 192.168.0.6 --calibrate              # Galea only
  ramms-access galea-p2.0 --ip 192.168.0.6 --neon-ip 192.168.0.42

Requires: pip install "ramms-access[galea,neon]"
"""

from __future__ import annotations

import json
import math
import pathlib
import time

import numpy as np
from pupil_labs.realtime_api.simple import Device

from ramms_access.adapters.galea_phase_1 import (
    CALIBRATION_ROUNDS, CALIBRATION_SECS, CLENCH, PROMPTS, REST, SETTLE_S, STEP_S, WINDOW_S,
    GaleaPhase1Adapter, GestureModel, features, filter_emg, window_rms)
from ramms_access.adapters.neon_phase_0 import PORT as NEON_PORT, gaze_angles
from ramms_access.intents import Intent

MODEL_FILE = pathlib.Path.home() / ".ramms_access" / "galea_phase2_0.json"
CALIBRATION_GESTURES = [REST, CLENCH]

MODE_REST = "rest"
MODE_ROTATE = "rotate"
MODE_DRIVE = "drive"
NEXT_MODE = {MODE_REST: MODE_ROTATE, MODE_ROTATE: MODE_DRIVE, MODE_DRIVE: MODE_REST}

FORWARD_SPEED = 0.5       # DRIVE mode forward speed (0..1)
TURN_SPEED = 0.5          # ROTATE mode turn speed whenever gaze is outside the deadband (0..1)
GAZE_DEADBAND_DEG = 5.0   # gaze within this of straight ahead -> no turn
GAZE_SMOOTHING = 0.2      # weight of each new gaze sample in the running average (saccades jump)
GAZE_STALE_S = 0.2        # no new gaze sample for this long -> no turn
CLENCH_ONSET_S = 0.15     # clench must hold this long to count
CLENCH_RELEASE_S = 0.15   # ...and drop out this long to count as released (no double-advance on a flicker)
SWITCH_LOCKOUT_S = 0.6    # minimum time between mode changes


class GaleaPhase2Adapter(GaleaPhase1Adapter):
    name = "galea-p2.0"

    def __init__(self, publisher, rate_hz: float = 50.0, ip: str = "", neon_ip: str = ""):
        super().__init__(publisher, rate_hz, ip, onset_s=CLENCH_ONSET_S, release_s=CLENCH_RELEASE_S)
        self.neon_ip = neon_ip
        self.neon: Device | None = None
        self.camera_matrix = None
        self.mode = MODE_REST
        self._was_clenched = False
        self._last_switch = -math.inf
        self._gaze_yaw: float | None = None  # smoothed, degrees, + = right
        self._gaze_worn = False
        self._gaze_rx = -math.inf            # local receive time of the last gaze sample

    # --- devices ----------------------------------------------------------
    def start(self) -> None:
        super().start()  # gesture model + Galea
        try:
            print(f"[galea-p2.0] connecting to Neon at {self.neon_ip}:{NEON_PORT} ...")
            self.neon = Device(address=self.neon_ip, port=NEON_PORT)
            self.camera_matrix = self.neon.get_calibration().scene_camera_matrix
        except BaseException:
            self.stop()
            raise
        print(f"[galea-p2.0] mode {self.mode.upper()} — clench to cycle REST -> ROTATE -> DRIVE")

    def stop(self) -> None:
        super().stop()
        if self.neon is not None:
            try:
                self.neon.close()
            except Exception:
                pass
            self.neon = None

    # --- clench model -----------------------------------------------------
    def _load_model(self) -> None:
        if MODEL_FILE.exists():
            self.model = GestureModel.from_json(json.loads(MODEL_FILE.read_text()))
            print(f"[galea-p2.0] clench model from {MODEL_FILE}")
        else:
            raise SystemExit(f"[galea-p2.0] no clench model at {MODEL_FILE} — "
                             f"run 'ramms-access galea-p2.0 --calibrate' with the headset on")

    def calibrate(self) -> None:
        """Record REST and CLENCH, fit the model, save it. Galea only."""
        self._connect()
        win_n, step_n, settle_n = (int(s * self._fs) for s in (WINDOW_S, STEP_S, SETTLE_S))
        samples: dict[str, list[np.ndarray]] = {g: [] for g in CALIBRATION_GESTURES}
        try:
            time.sleep(1.0)
            for r in range(CALIBRATION_ROUNDS):
                for g in CALIBRATION_GESTURES:
                    input(f"\n[galea-p2.0] round {r + 1}/{CALIBRATION_ROUNDS}: {PROMPTS[g]} for "
                          f"{CALIBRATION_SECS:.0f}s — press Enter, then start")
                    self.board.get_board_data()
                    time.sleep(CALIBRATION_SECS)
                    data = self.board.get_board_data()
                    filt = filter_emg(data, self._rows, self._fs)[:, settle_n:]
                    for s in range(0, filt.shape[1] - win_n + 1, step_n):
                        samples[g].append(features(filt[:, s:s + win_n]))
                    print(f"[galea-p2.0]   ok — mean uV: "
                          f"{' '.join(f'{v:.1f}' for v in window_rms(filt))}")
        finally:
            self.stop()

        arrays = {g: np.stack(v) for g, v in samples.items() if v}
        if len(arrays) != len(CALIBRATION_GESTURES):
            print("[galea-p2.0] calibration incomplete — nothing saved")
            return
        model = GestureModel.fit(arrays)

        print("\n[galea-p2.0] how well each is recognized on its own recordings:")
        for g in CALIBRATION_GESTURES:
            preds = [model.predict(f) for f in arrays[g]]
            acc = sum(p == g for p in preds) / len(preds)
            print(f"  {g:<8} {acc:6.1%}")
            if acc < 0.9:
                print(f"[galea-p2.0] WARNING: {g} is hard to recognize. Check electrode "
                      f"contact, clench more firmly, and re-run --calibrate.")

        MODEL_FILE.parent.mkdir(parents=True, exist_ok=True)
        payload = model.to_json() | {"emg_rows": self._rows, "rate": self._fs,
                                     "created": time.time()}
        MODEL_FILE.write_text(json.dumps(payload, indent=2))
        print(f"[galea-p2.0] saved clench model -> {MODEL_FILE}")

    # --- gaze -------------------------------------------------------------
    def _update_gaze(self, now: float) -> None:
        """Take the newest gaze sample, if one arrived since the last poll."""
        d = self.neon.receive_gaze_datum(timeout_seconds=0)
        if d is None or not hasattr(d, "x"):
            return
        yaw, _ = gaze_angles(d.x, d.y, self.camera_matrix)
        self._gaze_yaw = yaw if self._gaze_yaw is None else (
            GAZE_SMOOTHING * yaw + (1 - GAZE_SMOOTHING) * self._gaze_yaw)
        self._gaze_worn = bool(d.worn)
        self._gaze_rx = now

    def _gaze_turn(self, now: float) -> float:
        """Smoothed gaze yaw -> turn command; 0 when gaze is missing, stale,
        the glasses aren't worn, or the user is looking straight ahead."""
        if self._gaze_yaw is None or not self._gaze_worn or now - self._gaze_rx > GAZE_STALE_S:
            return 0.0
        if abs(self._gaze_yaw) <= GAZE_DEADBAND_DEG:
            return 0.0
        return math.copysign(TURN_SPEED, self._gaze_yaw)  # + = turn right

    # --- control ----------------------------------------------------------
    def poll(self) -> Intent | None:
        win = self._live_window()
        if win is None:
            return None
        now = time.monotonic()

        clenched = self.debouncer.update(self.model.predict(features(win))) == CLENCH
        if clenched and not self._was_clenched and now - self._last_switch >= SWITCH_LOCKOUT_S:
            self.mode = NEXT_MODE[self.mode]
            self._last_switch = now
            if self.mode == MODE_ROTATE:
                self._gaze_yaw = None
            print(f"\n[galea-p2.0] -> {self.mode.upper()}")
        self._was_clenched = clenched

        self._update_gaze(now)

        if self.mode == MODE_ROTATE:
            intent = Intent(drive=(self._gaze_turn(now), 0.0))
        elif self.mode == MODE_DRIVE:
            intent = Intent(drive=(0.0, FORWARD_SPEED))
        else:
            intent = Intent(drive=(0.0, 0.0))

        if now - self._last_print > 0.25:
            self._last_print = now
            gaze = f"{self._gaze_yaw:+6.1f}°" if self._gaze_yaw is not None else "   -  "
            print(f"\r[galea-p2.0] {self.mode.upper():<7} clench {'Y' if clenched else '-'}  "
                  f"gaze yaw {gaze}  worn {'Y' if self._gaze_worn else '-'}  "
                  f"drive x {intent.drive[0]:+.2f} y {intent.drive[1]:+.2f}   ", end="", flush=True)
        return intent
