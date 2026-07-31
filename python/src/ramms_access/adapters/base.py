"""Adapter base: a fixed-rate publish loop with safety conventions.

Subclasses implement poll() -> Intent | None. The loop publishes whatever
poll returns; None publishes a neutral keep-alive (consumers watchdog on
silence, so a healthy adapter must keep the stream alive even when idle —
silence means 'adapter died', and the robot stops).
"""

from __future__ import annotations

import time

from ramms_access.intents import EVENT_ESTOP, EVENT_RESUME, Intent
from ramms_access.transport import UdpIntentPublisher


class Adapter:
    name = "base"

    def __init__(self, publisher: UdpIntentPublisher, rate_hz: float = 50.0):
        self.publisher = publisher
        self.rate_hz = max(1.0, rate_hz)
        self._running = False

    # --- subclass API ---------------------------------------------------
    def start(self) -> None:
        """Acquire the device. Raise with a clear message on failure."""

    def poll(self) -> Intent | None:
        """Produce the current intent, or None for neutral keep-alive."""
        return None

    def stop(self) -> None:
        """Release the device."""

    # --- loop -----------------------------------------------------------
    def run(self) -> None:
        self.start()
        self._running = True
        period = 1.0 / self.rate_hz
        # A fresh adapter session takes control deliberately: clear any estop
        # left latched by a previous session's shutdown.
        self.publisher.send(Intent(source=self.name, events=[EVENT_RESUME]))
        try:
            while self._running:
                t0 = time.monotonic()
                intent = self.poll()
                if intent is None:
                    intent = Intent.neutral(self.name)
                intent.source = self.name
                self.publisher.send(intent)
                dt = time.monotonic() - t0
                if dt < period:
                    time.sleep(period - dt)
        except KeyboardInterrupt:
            pass
        finally:
            # Last will: explicit estop beats relying on the watchdog.
            try:
                self.publisher.send(Intent(source=self.name, events=[EVENT_ESTOP]))
            except OSError:
                pass
            self.stop()
