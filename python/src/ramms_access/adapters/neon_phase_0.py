"""Pupil Labs Neon — Phase 0: live gaze / IMU / eye-event monitor (nothing sent).

The Neon streams from its Companion phone over the local network (not
BrainFlow). This shows the latest sample of each stream so you can check the
glasses, the network path, and what the numbers do before using them:

  gaze   x, y in scene-camera pixels, converted to yaw/pitch degrees from the
         scene camera's intrinsics (pinhole model, lens distortion ignored:
         good near the center, less so at the edges); worn flag; pupil
         diameter and eyelid aperture when the phone's gaze mode provides them
  imu    gyro (deg/s), accel (m/s^2), orientation quaternion (+ yaw/pitch/roll
         derived from it; the axis convention is the device's, unverified here)
  events blink count / last blink, last fixation
  age    PC time minus sample timestamp; includes any phone-PC clock offset

Usage:
  ramms-access neon-p0 --ip 192.168.0.42  # the phone's IP, shown in the Companion app

The phone and the PC must be on the same network, with Neon Companion open
and the glasses plugged in.

Requires: pip install "ramms-access[neon]"  (pupil-labs-realtime-api)
"""

from __future__ import annotations

import math
import os
import time

from pupil_labs.realtime_api.simple import Device
from pupil_labs.realtime_api.streaming.eye_events import BlinkEventData, FixationEventData

PORT = 8080         # Neon Companion's real-time API port
REFRESH_S = 0.2
RECV_TIMEOUT_S = 0.02


def connect(ip: str) -> Device:
    print(f"[neon-p0] connecting to {ip}:{PORT} ...")
    return Device(address=ip, port=PORT)


def gaze_angles(x: float, y: float, camera_matrix) -> tuple[float, float]:
    """Scene-camera pixel -> (yaw, pitch) degrees relative to the camera axis.
    yaw > 0 = looking right, pitch > 0 = looking up (image y grows downward)."""
    fx, fy = camera_matrix[0][0], camera_matrix[1][1]
    cx, cy = camera_matrix[0][2], camera_matrix[1][2]
    return math.degrees(math.atan((x - cx) / fx)), math.degrees(math.atan(-(y - cy) / fy))


def quat_to_ypr(q) -> tuple[float, float, float]:
    """Quaternion (x, y, z, w) -> (yaw, pitch, roll) degrees, Z-Y-X convention."""
    x, y, z, w = q.x, q.y, q.z, q.w
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def _age_ms(ts: float) -> str:
    return f"{(time.time() - ts) * 1000:7.0f} ms"


def monitor(ip: str) -> None:
    device = connect(ip)
    try:
        camera_matrix = device.get_calibration().scene_camera_matrix
    except Exception as e:  # angles are a convenience; pixels still show without them
        print(f"[neon-p0] could not read scene camera calibration ({e}); showing pixels only")
        camera_matrix = None

    os.system("")  # turns on ANSI cursor control in the Windows console
    print(f"[neon-p0] {device.phone_name} @ {device.phone_ip}  glasses {device.serial_number_glasses}"
          f"  battery {device.battery_level_percent}%  — Ctrl-C to quit")

    gaze = imu = last_fixation = None
    last_blink_ts, blinks = None, 0
    drawn = 0
    try:
        while True:
            gaze = device.receive_gaze_datum(timeout_seconds=RECV_TIMEOUT_S) or gaze
            imu = device.receive_imu_datum(timeout_seconds=RECV_TIMEOUT_S) or imu
            event = device.receive_eye_events(timeout_seconds=RECV_TIMEOUT_S)
            if isinstance(event, BlinkEventData):
                blinks, last_blink_ts = blinks + 1, event.rtp_ts_unix_seconds
            elif isinstance(event, FixationEventData):
                last_fixation = event

            lines = ["gaze"]
            if gaze is None:
                lines.append("  (waiting for gaze — glasses plugged in and worn?)")
            elif not hasattr(gaze, "x"):  # dual-monocular mode reports left/right only
                lines.append(f"  left  {gaze.left}   right {gaze.right}")
                lines.append(f"  worn  {gaze.worn}     age {_age_ms(gaze.timestamp_unix_seconds)}")
            else:
                lines.append(f"  pixel      x {gaze.x:8.1f}   y {gaze.y:8.1f}")
                if camera_matrix is not None:
                    yaw, pitch = gaze_angles(gaze.x, gaze.y, camera_matrix)
                    lines.append(f"  angle    yaw {yaw:+8.1f}°  pitch {pitch:+8.1f}°   (+ = right / up)")
                lines.append(f"  worn  {str(gaze.worn):<5}      age {_age_ms(gaze.timestamp_unix_seconds)}")
                if hasattr(gaze, "pupil_diameter_left"):
                    lines.append(f"  pupil mm   L {gaze.pupil_diameter_left:6.2f}    R {gaze.pupil_diameter_right:6.2f}")
                if hasattr(gaze, "eyelid_aperture_left"):
                    lines.append(f"  eyelid mm  L {gaze.eyelid_aperture_left:6.2f}    R {gaze.eyelid_aperture_right:6.2f}")

            lines.append("imu")
            if imu is None:
                lines.append("  (waiting for imu)")
            else:
                g, a, q = imu.gyro_data, imu.accel_data, imu.quaternion
                yaw, pitch, roll = quat_to_ypr(q)
                lines.append(f"  gyro  deg/s  x {g.x:+8.2f}  y {g.y:+8.2f}  z {g.z:+8.2f}")
                lines.append(f"  accel m/s^2  x {a.x:+8.2f}  y {a.y:+8.2f}  z {a.z:+8.2f}")
                lines.append(f"  quat         x {q.x:+.3f}  y {q.y:+.3f}  z {q.z:+.3f}  w {q.w:+.3f}")
                lines.append(f"  orient deg   yaw {yaw:+7.1f}  pitch {pitch:+7.1f}  roll {roll:+7.1f}")
                lines.append(f"  age   {_age_ms(imu.timestamp_unix_seconds)}")

            lines.append("events")
            blink_ago = f"{time.time() - last_blink_ts:5.1f} s ago" if last_blink_ts else "-"
            lines.append(f"  blinks {blinks:<5}  last {blink_ago}")
            if last_fixation is not None:
                f = last_fixation
                lines.append(f"  last fixation  {(f.end_time_ns - f.start_time_ns) / 1e6:6.0f} ms at "
                             f"({f.mean_gaze_x:.0f}, {f.mean_gaze_y:.0f}) px, "
                             f"amplitude {f.amplitude_angle_deg:.1f}°")
            else:
                lines.append("  last fixation  -")

            # redraw in place: move the cursor back up over the previous block,
            # padding so a shorter block blanks the old block's leftover lines
            lines += [""] * (drawn - len(lines))
            print(("\r\x1b[%dF" % drawn if drawn else "\r") + "\n".join(
                line + "\x1b[K" for line in lines), flush=True)
            drawn = len(lines)
            time.sleep(REFRESH_S)
    except KeyboardInterrupt:
        print()
    finally:
        device.close()
