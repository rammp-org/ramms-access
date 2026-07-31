"""Pointer adapter: OS-level cursor -> virtual joystick intents.

This is the Neuralink-ready path: a Link user's control surface is the
system pointer, so anything that presents as pointer/HID (Neuralink cursor,
head mice, eye-gaze pointers) drives the robot through this adapter with no
device-specific code. The screen (or a configured sub-region) becomes a
virtual joystick pad: cursor displacement from pad center maps to drive
x/y with a center dead-zone; primary click toggles the gripper.

Requires: pip install "ramms-access[pointer]"  (pynput)
"""

from __future__ import annotations

from ramms_access.adapters.base import Adapter
from ramms_access.intents import EVENT_GRIPPER_TOGGLE, Intent


class PointerAdapter(Adapter):
    name = "pointer"

    def __init__(self, publisher, rate_hz: float = 50.0, dead_zone: float = 0.15,
                 pad_size_px: int = 600):
        super().__init__(publisher, rate_hz)
        self.dead_zone = dead_zone
        self.pad_size_px = pad_size_px
        self._center: tuple[float, float] | None = None
        self._click_pending = False
        self._mouse = None
        self._listener = None

    def start(self) -> None:
        try:
            from pynput import mouse
        except ImportError as e:
            raise SystemExit(
                "pointer adapter needs pynput: pip install 'ramms-access[pointer]'") from e
        self._mouse = mouse.Controller()
        # First observed position defines the pad center; click re-centers
        # via the on_click handler below (press = gripper, so use a modifier-
        # free convention: single click -> gripper toggle, and the pad center
        # follows a slow decay toward the current position when idle later).
        self._center = tuple(self._mouse.position)

        def on_click(x, y, button, pressed):
            if pressed:
                self._click_pending = True

        self._listener = mouse.Listener(on_click=on_click)
        self._listener.start()

    def poll(self) -> Intent | None:
        x, y = self._mouse.position
        cx, cy = self._center
        half = self.pad_size_px / 2.0
        nx = max(-1.0, min(1.0, (x - cx) / half))
        ny = max(-1.0, min(1.0, (cy - y) / half))  # screen y is down; forward is up
        if abs(nx) < self.dead_zone:
            nx = 0.0
        if abs(ny) < self.dead_zone:
            ny = 0.0
        events = []
        if self._click_pending:
            self._click_pending = False
            events.append(EVENT_GRIPPER_TOGGLE)
        return Intent(drive=(nx, ny), events=events)

    def stop(self) -> None:
        if self._listener is not None:
            self._listener.stop()
