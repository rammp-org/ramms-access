"""Sim chair — Phase 0: live heading monitor (nothing sent).

Reads the chair's heading from the running sim through Remote Execution
(ramms_access.sim_chair) and shows it with the turn rate and how fast and how
delayed the readout is — the numbers that decide whether it is good enough
to feed a control loop.

  yaw        Unreal yaw in degrees, + = turned right, wraps at +/-180
  rate       turn rate in deg/s, from successive yaws over the sim's own clock
  delay      round trip of one read (request -> editor -> answer), ms
  reads/s    how many reads completed in the last second

Usage (editor open with the RAMMS project, a map with the chair in Play):
  ramms-access chair-p0
"""

from __future__ import annotations

import os
import time
from collections import deque

from ramms_access.sim_chair import SimChairReader, wrap_deg

REFRESH_S = 0.2


def monitor() -> None:
    print("[chair-p0] connecting to the Unreal Editor (Remote Execution) ...")
    with SimChairReader() as chair:
        os.system("")  # turns on ANSI cursor control in the Windows console
        print("[chair-p0] connected — press Play on a map with the chair. Ctrl-C to quit.")
        prev = None
        rate = 0.0
        delays: deque[float] = deque(maxlen=200)
        done: deque[float] = deque()
        last_draw = 0.0
        drawn = 0
        try:
            while True:
                t0 = time.monotonic()
                state = chair.read()
                t1 = time.monotonic()
                delays.append((t1 - t0) * 1000)
                done.append(t1)
                while done and t1 - done[0] > 1.0:
                    done.popleft()
                if state is not None and prev is not None and state.sim_time_s > prev.sim_time_s:
                    rate = wrap_deg(state.yaw_deg - prev.yaw_deg) / (state.sim_time_s - prev.sim_time_s)
                prev = state if state is not None else None

                if t1 - last_draw >= REFRESH_S:
                    last_draw = t1
                    if state is None:
                        lines = ["  (editor not in Play, or no drivable chair on this map)", "", ""]
                    else:
                        lines = [f"  yaw   {state.yaw_deg:+8.2f} deg     rate {rate:+8.2f} deg/s",
                                 f"  sim time {state.sim_time_s:9.2f} s", ""]
                    s = sorted(delays)
                    lines.append(f"  delay  median {s[len(s) // 2]:6.1f} ms   worst {s[-1]:6.1f} ms   "
                                 f"reads/s {len(done):4d}")
                    print(("\r\x1b[%dF" % drawn if drawn else "\r") + "\n".join(
                        line + "\x1b[K" for line in lines), flush=True)
                    drawn = len(lines)
        except KeyboardInterrupt:
            print()
