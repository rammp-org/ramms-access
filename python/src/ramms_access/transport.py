"""UDP publisher for the intent stream (latest-wins, fire-and-forget)."""

from __future__ import annotations

import random
import socket

from ramms_access.intents import Intent

DEFAULT_PORT = 30040


class UdpIntentPublisher:
    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_PORT):
        self.addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq = 0
        self._sid = random.getrandbits(31)

    def send(self, intent: Intent) -> None:
        self._seq += 1
        self._sock.sendto(intent.to_payload(self._seq, self._sid), self.addr)

    def close(self) -> None:
        self._sock.close()
