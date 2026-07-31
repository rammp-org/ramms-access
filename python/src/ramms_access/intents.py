"""Intent protocol v1 — the contract between device adapters and consumers.

Keep this file dependency-free and boring: both the UE component
(RammsAccessInputComponent) and the future robot bridge parse exactly this.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field


PROTOCOL_VERSION = 1

# Discrete events consumers understand (processed in order of arrival).
EVENT_ESTOP = "estop"
EVENT_RESUME = "resume"
EVENT_STOP = "stop"
EVENT_GRIPPER_OPEN = "gripper_open"
EVENT_GRIPPER_CLOSE = "gripper_close"
EVENT_GRIPPER_TOGGLE = "gripper_toggle"
EVENT_SYNC_TARGET = "sync_target"


def _clamp(v: float) -> float:
    return max(-1.0, min(1.0, float(v)))


@dataclass
class Intent:
    """One intent packet.

    drive: wheelchair-joystick semantics (x=turn, y=forward), -1..1.
    ee_lin / ee_ang: normalized end-effector rates, -1..1
        (linear xyz in the EE frame; angular pitch/yaw/roll).
    events: discrete commands (see EVENT_* constants).
    confidence: decoder confidence 0..1; consumers scale continuous rates
        by it, so an uncertain BCI decode moves the robot slowly, not wrongly.
    """

    source: str = "unknown"
    drive: tuple[float, float] | None = None
    ee_lin: tuple[float, float, float] | None = None
    ee_ang: tuple[float, float, float] | None = None
    events: list[str] = field(default_factory=list)
    confidence: float = 1.0

    def to_payload(self, seq: int, sid: int = 0) -> bytes:
        msg: dict = {
            "v": PROTOCOL_VERSION,
            "t": time.time(),
            "src": self.source,
            "seq": seq,
            # Session id: random per publisher instance. Sequence numbers only
            # order packets WITHIN a session; a new sid tells consumers to
            # reset their staleness cursor (otherwise a restarted adapter's
            # packets — starting again at seq 1 — all look stale and die).
            "sid": sid,
        }
        if self.drive is not None:
            msg["drive"] = {"x": _clamp(self.drive[0]), "y": _clamp(self.drive[1])}
        ee: dict = {}
        if self.ee_lin is not None:
            ee["lin"] = [_clamp(v) for v in self.ee_lin]
        if self.ee_ang is not None:
            ee["ang"] = [_clamp(v) for v in self.ee_ang]
        if ee:
            msg["ee"] = ee
        if self.events:
            msg["events"] = list(self.events)
        if self.confidence != 1.0:
            msg["conf"] = max(0.0, min(1.0, self.confidence))
        return json.dumps(msg, separators=(",", ":")).encode("utf-8")

    @staticmethod
    def neutral(source: str) -> "Intent":
        """A keep-alive intent: zero motion, feeds the consumer watchdog."""
        return Intent(source=source, drive=(0.0, 0.0))
