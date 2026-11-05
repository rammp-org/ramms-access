"""OpenBCI Galea + Pupil Labs Neon + chair heading — Phase 2.1: look, clench, turn, clench, go.

Written as if sitting in the real chair: every angle is 1:1. Look 90 deg to
the right of center and the chair turns 90 deg.

Stages, moved between by a jaw clench:
  COLLECT  gaze, glasses IMU and chair heading are read continuously; the chair
           is stopped.
     clench: those three are frozen, the turn needed to face where you were
             looking is calculated, and ->
  TURN     the chair turns by exactly that amount (closed loop on the chair
           heading), then stops by itself and waits, facing the target.
     clench (even if the turn hasn't finished) ->
  DRIVE    straight forward at a set speed in the direction it now faces.
     clench -> COLLECT (stopped)

Turn calculation from the frozen values (yaws in degrees, + = right):
    head vs chair = head yaw - head yaw at start-up
    turn          = head vs chair + gaze yaw
Head yaw is zeroed at start-up, once the glasses' yaw has settled (it eases
back by several degrees after any movement), so look straight ahead (along
the chair) and hold still when it starts. Empty IMU samples (all-zero, which
the phone occasionally sends) are ignored. The chair's own rotation is NOT subtracted: in the sim the head
doesn't turn with the chair. (On a real chair the glasses IMU would also pick
up the chair's rotation, which would then need subtracting.)

Safety: the turn stops (-> COLLECT) if the chair heading goes stale or the
turn takes longer than TURN_TIMEOUT_S; a clench is ignored in COLLECT if gaze,
glasses IMU or chair heading isn't fresh. Clench detection is phase 2.0's
(same calibration file).

Usage:
  ramms-access galea-p2.1 --ip 192.168.0.6 --calibrate             # clench model (Galea only)
  ramms-access galea-p2.1 --ip 192.168.0.6 --neon-ip 192.168.0.42
"""

from __future__ import annotations

import math
import threading
import time

from pupil_labs.realtime_api.simple import Device

import numpy as np

from ramms_access.adapters.galea_phase_1 import CLENCH, REST, Debouncer, GaleaPhase1Adapter, features
from ramms_access.adapters.galea_phase_2_0 import (
    CLENCH_RELEASE_S, GAZE_STALE_S, SWITCH_LOCKOUT_S, GaleaPhase2Adapter)
from ramms_access.adapters.neon_phase_0 import PORT as NEON_PORT, quat_to_ypr
from ramms_access.intents import Intent
from ramms_access.sim_chair import SimChairReader, wrap_deg

STAGE_COLLECT = "collect"
STAGE_TURN = "turn"
STAGE_DRIVE = "drive"

FORWARD_SPEED = 0.5       # DRIVE forward speed (0..1)
TURN_SPEED = 0.5          # TURN speed when far from the target (0..1)
TURN_MIN_SPEED = 0.15     # floor near the target, so the chair doesn't stall short of it
TURN_SLOWDOWN_DEG = 20.0  # start slowing down this far from the target
TURN_TOLERANCE_DEG = 2.0  # turn done when within this of the target
TURN_TIMEOUT_S = 15.0     # give up (-> COLLECT) if a turn takes longer
CHAIR_STALE_S = 0.25      # chair heading older than this counts as missing
IMU_STALE_S = 0.25        # glasses IMU older than this counts as missing
IMU_FIRST_SAMPLE_S = 10.0 # wait this long at start-up for the first glasses IMU sample
ZERO_STEADY_DEG = 0.5     # zero once the glasses' yaw stays within this ...
ZERO_STEADY_S = 1.0       # ... for this long
ZERO_TIMEOUT_S = 15.0     # zero anyway (with a warning) if it never settles
CLENCH_STRENGTH = 0.6     # a trigger must reach this fraction of the way from calibrated rest (0) to calibrated clench (1)
CLENCH_TRIGGER_S = 0.3    # ...and hold for this long
HEAD_YAW_SIGN = 1.0       # flip to -1.0 if turning the head right reads as left


class GaleaPhase21Adapter(GaleaPhase2Adapter):
    name = "galea-p2.1"

    def __init__(self, publisher, rate_hz: float = 50.0, ip: str = "", neon_ip: str = ""):
        super().__init__(publisher, rate_hz, ip, neon_ip)
        self.debouncer = Debouncer(round(CLENCH_TRIGGER_S * self.rate_hz),
                                   round(CLENCH_RELEASE_S * self.rate_hz))
        self._strength = 0.0
        self.stage = STAGE_COLLECT
        self.frozen: dict | None = None       # gaze, head, chair at the clench + turn
        self.target: float | None = None      # chair heading to turn to
        self._turn_started = 0.0
        self._turn_done = False
        self._head_yaw: float | None = None   # raw glasses IMU yaw
        self._head_rx = -math.inf
        self._head_zero: float | None = None
        self._chair_lock = threading.Lock()
        self._chair: tuple[float, float] | None = None  # (yaw, local receive time)
        self._chair_thread: threading.Thread | None = None
        self._chair_running = False

    # --- devices ----------------------------------------------------------
    def start(self) -> None:
        GaleaPhase1Adapter.start(self)  # Galea + phase 2.0's clench model (its _load_model)
        try:
            print(f"[galea-p2.1] connecting to Neon at {self.neon_ip}:{NEON_PORT} ...")
            self.neon = Device(address=self.neon_ip, port=NEON_PORT)
            self.camera_matrix = self.neon.get_calibration().scene_camera_matrix
            self._chair_running = True
            ready = threading.Event()
            self._chair_thread = threading.Thread(target=self._chair_loop, args=(ready,), daemon=True)
            self._chair_thread.start()
            if not ready.wait(10.0) or self._chair is None:
                raise SystemExit("[galea-p2.1] no chair heading from the sim — is the editor in "
                                 "Play on a map with the chair? (try 'ramms-access chair-p0')")
            self._zero()
        except BaseException:
            self.stop()
            raise
        print("[galea-p2.1] COLLECT — look where you want to go and clench")

    def stop(self) -> None:
        self._chair_running = False
        if self._chair_thread is not None:
            self._chair_thread.join(timeout=2.0)
            self._chair_thread = None
        super().stop()

    def _chair_loop(self, ready: threading.Event) -> None:
        """Background: keep the latest chair heading (each read waits ~1 frame)."""
        try:
            with SimChairReader() as reader:
                while self._chair_running:
                    state = reader.read()
                    if state is not None:
                        with self._chair_lock:
                            self._chair = (state.yaw_deg, time.monotonic())
                        ready.set()
                    else:
                        time.sleep(0.1)  # editor not in Play
        except Exception as e:
            print(f"\n[galea-p2.1] chair heading readout stopped: {e}")
        finally:
            ready.set()

    def _zero(self) -> None:
        """Head zeroed: the head points along the chair now. Waits for the
        glasses' yaw to settle first, since it eases back for a few seconds
        after any movement (e.g. putting the glasses on)."""
        print("[galea-p2.1] look straight ahead along the chair and hold still — zeroing ...")
        # the IMU stream's first sample takes ~3 s to arrive after it starts
        deadline = time.monotonic() + IMU_FIRST_SAMPLE_S
        while self._head_yaw is None and time.monotonic() < deadline:
            self._update_head(time.monotonic(), timeout_s=0.5)
        if self._head_yaw is None:
            raise SystemExit(f"[galea-p2.1] no glasses IMU data from the Neon within {IMU_FIRST_SAMPLE_S:.0f} s")
        recent: list[tuple[float, float]] = []  # (time, yaw)
        start = time.monotonic()
        while True:
            now = time.monotonic()
            if self._update_head(now, timeout_s=0.5):
                recent.append((now, self._head_yaw))
                recent = [(t, y) for t, y in recent if now - t <= ZERO_STEADY_S]
                spread = max(abs(wrap_deg(y - recent[0][1])) for _, y in recent)
                if now - start >= ZERO_STEADY_S and spread <= ZERO_STEADY_DEG:
                    break
            if now - start > ZERO_TIMEOUT_S:
                print(f"[galea-p2.1] WARNING: head yaw never settled within {ZERO_STEADY_DEG} deg — "
                      f"zeroing anyway; head angles may drift")
                break
        self._head_zero = self._head_yaw
        print(f"[galea-p2.1] zeroed after {time.monotonic() - start:.1f} s")

    # --- live values ------------------------------------------------------
    def _chair_yaw(self, now: float) -> float | None:
        with self._chair_lock:
            c = self._chair
        return c[0] if c is not None and now - c[1] <= CHAIR_STALE_S else None

    def _update_head(self, now: float, timeout_s: float = 0.0) -> bool:
        """Take the newest glasses IMU sample; True if a valid one arrived.
        The phone occasionally sends empty samples (all-zero quaternion,
        timestamp 0); those would read as yaw 0, so they are skipped."""
        imu = self.neon.receive_imu_datum(timeout_seconds=timeout_s)
        if imu is None or imu.timestamp_unix_seconds <= 0:
            return False
        q = imu.quaternion
        if q.x == 0 and q.y == 0 and q.z == 0 and q.w == 0:
            return False
        self._head_yaw = quat_to_ypr(q)[0]
        self._head_rx = now
        return True

    def _head_ok(self, now: float) -> bool:
        return self._head_yaw is not None and now - self._head_rx <= IMU_STALE_S

    def _gaze_ok(self, now: float) -> bool:
        return self._gaze_yaw is not None and self._gaze_worn and now - self._gaze_rx <= GAZE_STALE_S

    def _head_vs_chair(self, head_yaw: float) -> float:
        return HEAD_YAW_SIGN * wrap_deg(head_yaw - self._head_zero)

    def _classify(self, f: np.ndarray) -> str:
        """CLENCH only if the model says clench AND it is strong enough: the
        average log-RMS must be CLENCH_STRENGTH of the way from the calibrated
        rest level to the calibrated clench level."""
        rest = self.model.means[self.model.labels.index(REST)].mean()
        clench = self.model.means[self.model.labels.index(CLENCH)].mean()
        self._strength = float((f.mean() - rest) / (clench - rest)) if clench > rest else 0.0
        if self.model.predict(f) == CLENCH and self._strength >= CLENCH_STRENGTH:
            return CLENCH
        return REST

    # --- stages -----------------------------------------------------------
    def _on_clench(self, now: float) -> None:
        if self.stage == STAGE_COLLECT:
            chair_yaw = self._chair_yaw(now)
            missing = [name for name, ok in (("gaze", self._gaze_ok(now)), ("glasses IMU", self._head_ok(now)),
                                             ("chair heading", chair_yaw is not None)) if not ok]
            if missing:
                print(f"\n[galea-p2.1] clench ignored — no fresh {', '.join(missing)}")
                return
            head = self._head_vs_chair(self._head_yaw)
            turn = wrap_deg(head + self._gaze_yaw)
            self.frozen = {"gaze": self._gaze_yaw, "head": head, "chair": chair_yaw, "turn": turn}
            self.target = wrap_deg(chair_yaw + turn)
            self._turn_started = now
            self._turn_done = False
            self.stage = STAGE_TURN
            print(f"\n[galea-p2.1] frozen: gaze {self._gaze_yaw:+.1f}  head {head:+.1f}  "
                  f"chair {chair_yaw:+.1f}  -> TURN {turn:+.1f} deg (to heading {self.target:+.1f})")
        elif self.stage == STAGE_TURN:
            self.stage = STAGE_DRIVE
            print("\n[galea-p2.1] -> DRIVE" + ("" if self._turn_done else " (turn not finished)"))
        else:
            self.stage = STAGE_COLLECT
            print("\n[galea-p2.1] -> COLLECT (stopped)")

    def _turn_command(self, now: float) -> float | None:
        """Turn toward the target; None = finished (stay in TURN, stopped) or
        aborted (-> COLLECT)."""
        if self._turn_done:
            return None
        chair_yaw = self._chair_yaw(now)
        if chair_yaw is None:
            print("\n[galea-p2.1] chair heading stale — stopping -> COLLECT")
            self.stage = STAGE_COLLECT
            return None
        if now - self._turn_started > TURN_TIMEOUT_S:
            print("\n[galea-p2.1] turn timed out — stopping -> COLLECT")
            self.stage = STAGE_COLLECT
            return None
        error = wrap_deg(self.target - chair_yaw)
        if abs(error) <= TURN_TOLERANCE_DEG:
            print(f"\n[galea-p2.1] turned (off by {error:+.1f} deg), stopped — clench to drive")
            self._turn_done = True
            return None
        speed = max(TURN_MIN_SPEED, TURN_SPEED * min(1.0, abs(error) / TURN_SLOWDOWN_DEG))
        return math.copysign(speed, error)  # + = turn right

    # --- control ----------------------------------------------------------
    def poll(self) -> Intent | None:
        win = self._live_window()
        if win is None:
            return None
        now = time.monotonic()
        self._update_gaze(now)
        self._update_head(now)

        clenched = self.debouncer.update(self._classify(features(win))) == CLENCH
        if clenched and not self._was_clenched and now - self._last_switch >= SWITCH_LOCKOUT_S:
            self._last_switch = now
            self._on_clench(now)
        self._was_clenched = clenched

        turn = 0.0
        if self.stage == STAGE_TURN:
            turn = self._turn_command(now) or 0.0
        intent = Intent(drive=(turn, FORWARD_SPEED if self.stage == STAGE_DRIVE else 0.0))

        if now - self._last_print > 0.25:
            self._last_print = now
            if self.stage == STAGE_COLLECT:
                chair_yaw = self._chair_yaw(now)
                gaze = f"{self._gaze_yaw:+6.1f}" if self._gaze_ok(now) else "   -  "
                head = f"{self._head_vs_chair(self._head_yaw):+6.1f}" if self._head_ok(now) else "   -  "
                chair = f"{chair_yaw:+7.1f}" if chair_yaw is not None else "   -   "
                detail = f"live  gaze {gaze}  head {head}  chair {chair}"
            else:
                f = self.frozen
                detail = (f"frozen gaze {f['gaze']:+6.1f}  head {f['head']:+6.1f}  "
                          f"chair {f['chair']:+7.1f}  turn {f['turn']:+6.1f}")
            label = "TURNED" if self.stage == STAGE_TURN and self._turn_done else self.stage.upper()
            print(f"\r[galea-p2.1] {label:<7} clench {'Y' if clenched else '-'} "
                  f"{self._strength:4.0%}  {detail}  "
                  f"drive x {intent.drive[0]:+.2f} y {intent.drive[1]:+.2f}   ", end="", flush=True)
        return intent
