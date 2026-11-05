"""Read the chair's heading out of a running RAMMS sim (Unreal Editor, Play mode).

Uses Unreal's Python Remote Execution (enabled in the project's
DefaultEngine.ini: multicast 239.0.0.1:6766 on loopback), the same route as the
maintainer's Scripts/editor_remote_exec.py and Scripts/pie_tests scripts, but
keeps ONE connection open so the heading can be read repeatedly. No IMU
component or C++ change is needed: it reads the chair pawn's actor rotation
directly, which is exact.

    with SimChairReader() as chair:
        r = chair.read()     # ChairState(yaw_deg, sim_time_s) or None (not in Play)

yaw_deg follows Unreal: degrees, increasing when the chair turns RIGHT
(drive.turn = +1), wrapping at +/-180. sim_time_s is the Play world's clock,
so turn rate = change in yaw / change in sim_time_s, free of network delay.

Local machine only: the editor binds Remote Execution to 127.0.0.1.
"""

from __future__ import annotations

import ast
import json
import socket
import struct
import time
import uuid
from typing import NamedTuple

MULTICAST_GROUP = ("239.0.0.1", 6766)
BIND_ADDR = "127.0.0.1"
EDITOR_ENDPOINT = (BIND_ADDR, MULTICAST_GROUP[1])  # see editor_remote_exec.py for why unicast
MAGIC = "ue_py"
PROTO_VERSION = 1
DISCOVER_TIMEOUT_S = 6.0
COMMAND_TIMEOUT_S = 2.0
PING_EVERY_S = 1.0  # keeps the editor from timing our node out

# Runs once in the editor: defines the reader there. Finds the chair the way
# Scripts/pie_tests does (the pawn carrying a differential-drive controller)
# and re-finds it whenever Play restarts.
_EDITOR_SETUP = '''
import unreal
def _ramms_access_chair():
    w = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
    if w is None:
        return None
    p = getattr(unreal, "_ramms_access_pawn", None)
    try:
        if p is not None and p.get_world() == w:
            return (p.get_actor_rotation().yaw, unreal.GameplayStatics.get_time_seconds(w))
    except Exception:
        pass
    for p in unreal.GameplayStatics.get_all_actors_of_class(w, unreal.Pawn):
        if p.get_component_by_class(unreal.RammsDifferentialDriveController):
            unreal._ramms_access_pawn = p
            return (p.get_actor_rotation().yaw, unreal.GameplayStatics.get_time_seconds(w))
    return None
unreal._ramms_access_chair = _ramms_access_chair
'''
_EDITOR_READ = "unreal._ramms_access_chair()"


class ChairState(NamedTuple):
    yaw_deg: float     # Unreal yaw, + = turned right, wraps at +/-180
    sim_time_s: float  # Play-world clock


def _msg(msg_type: str, source: str, dest: str | None = None, data: dict | None = None) -> bytes:
    m = {"version": PROTO_VERSION, "magic": MAGIC, "type": msg_type, "source": source}
    if dest is not None:
        m["dest"] = dest
    if data is not None:
        m["data"] = data
    return json.dumps(m).encode("utf-8")


def _parse(payload: bytes) -> dict | None:
    try:
        m = json.loads(payload.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return None
    return m if m.get("magic") == MAGIC and m.get("version") == PROTO_VERSION else None


class SimChairReader:
    def __init__(self):
        self._node = str(uuid.uuid4())
        self._remote: str | None = None
        self._mcast: socket.socket | None = None
        self._conn: socket.socket | None = None
        self._last_ping = 0.0

    def __enter__(self) -> "SimChairReader":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --- connection -------------------------------------------------------
    def connect(self) -> None:
        mcast = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        mcast.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            mcast.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        mcast.bind(("", MULTICAST_GROUP[1]))
        mreq = struct.pack("4s4s", socket.inet_aton(MULTICAST_GROUP[0]), socket.inet_aton(BIND_ADDR))
        mcast.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        mcast.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(BIND_ADDR))
        mcast.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        mcast.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 0)
        mcast.settimeout(0.5)
        self._mcast = mcast

        deadline = time.monotonic() + DISCOVER_TIMEOUT_S
        while time.monotonic() < deadline and self._remote is None:
            mcast.sendto(_msg("ping", self._node), EDITOR_ENDPOINT)
            try:
                payload, _ = mcast.recvfrom(4096)
            except socket.timeout:
                continue
            m = _parse(payload)
            if m and m.get("type") == "pong" and m.get("source") != self._node:
                self._remote = m["source"]
        if self._remote is None:
            self.close()
            raise ConnectionError("no Unreal Editor answered Remote Execution on 127.0.0.1 — "
                                  "is the editor open with the RAMMS project?")

        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((BIND_ADDR, 0))
        listener.listen(1)
        listener.settimeout(DISCOVER_TIMEOUT_S)
        ip, port = listener.getsockname()
        mcast.sendto(_msg("open_connection", self._node, self._remote,
                          {"command_ip": ip, "command_port": port}), EDITOR_ENDPOINT)
        try:
            self._conn, _ = listener.accept()
        except socket.timeout:
            self.close()
            raise ConnectionError("the editor did not open the command connection")
        finally:
            listener.close()
        self._conn.settimeout(COMMAND_TIMEOUT_S)
        self._last_ping = time.monotonic()

        # ExecuteFile also takes literal script text; ExecuteStatement only takes
        # a single statement, so a multi-line setup is a SyntaxError there.
        result = self._command(_EDITOR_SETUP, "ExecuteFile")
        if not result.get("success"):
            out = " ".join(e.get("output", "") for e in result.get("output", []))
            self.close()
            raise RuntimeError(f"editor-side setup failed: {out.strip() or result.get('result')} "
                               f"(details in the editor's Output Log, LogPython)")

    def close(self) -> None:
        if self._mcast is not None and self._remote is not None:
            try:
                self._mcast.sendto(_msg("close_connection", self._node, self._remote), EDITOR_ENDPOINT)
            except OSError:
                pass
        for s in (self._conn, self._mcast):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
        self._conn = self._mcast = self._remote = None

    def _command(self, command: str, exec_mode: str) -> dict:
        """Send one command, wait for its result (one JSON object per command)."""
        now = time.monotonic()
        if now - self._last_ping > PING_EVERY_S:
            self._mcast.sendto(_msg("ping", self._node), EDITOR_ENDPOINT)
            self._last_ping = now
        self._conn.sendall(_msg("command", self._node, self._remote,
                                {"command": command, "unattended": True, "exec_mode": exec_mode}))
        buf = b""
        while True:
            chunk = self._conn.recv(65536)
            if not chunk:
                raise ConnectionError("the editor closed the connection")
            buf += chunk
            m = _parse(buf)
            if m and m.get("type") == "command_result":
                return m.get("data", {})

    # --- reading ----------------------------------------------------------
    def read(self) -> ChairState | None:
        """The chair's current heading, or None when the editor isn't in Play
        mode (or the map has no drivable chair)."""
        data = self._command(_EDITOR_READ, "EvaluateStatement")
        if not data.get("success"):
            raise RuntimeError(f"editor-side read failed: {data.get('result')}")
        value = ast.literal_eval(data.get("result") or "None")
        return None if value is None else ChairState(float(value[0]), float(value[1]))


def wrap_deg(a: float) -> float:
    """Angle difference into [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0
