"""Synthetic intent patterns — no hardware, no extra deps.

The vertical-slice smoke test: `ramms-access demo --pattern circle` should
drive the chair in PIE; `--pattern arm-wave` should move the end effector.
"""

from __future__ import annotations

import math
import time

from ramms_access.adapters.base import Adapter
from ramms_access.intents import Intent


class DemoAdapter(Adapter):
    name = "demo"

    def __init__(self, publisher, rate_hz: float = 50.0, pattern: str = "circle"):
        super().__init__(publisher, rate_hz)
        self.pattern = pattern
        self._t0 = time.monotonic()

    def poll(self) -> Intent | None:
        t = time.monotonic() - self._t0
        if self.pattern == "circle":
            # gentle constant-turn drive
            return Intent(drive=(0.35, 0.5))
        if self.pattern == "weave":
            return Intent(drive=(0.6 * math.sin(0.5 * t), 0.5))
        if self.pattern == "arm-wave":
            # slow vertical EE oscillation, chair stationary
            return Intent(drive=(0.0, 0.0), ee_lin=(0.0, 0.0, 0.8 * math.sin(0.4 * t)))
        return None  # "idle": neutral keep-alive only
