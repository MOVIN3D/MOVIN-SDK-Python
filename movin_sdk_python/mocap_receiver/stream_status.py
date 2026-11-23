"""
Replies to MOVIN Studio's OSC streaming status probe.

While streaming over OSC, MOVIN Studio sends ``/MOVIN/OSC/Status/Request``
about once a second to the stream destination::

    [token: str (32 hex digits), reply_port: int, character: str]

and expects ``/MOVIN/OSC/Status`` back on ``reply_port`` of the sender's
address, with type tags ``sisffffffi``::

    [token, version (1), character,
     motion_fps, pointcloud_fps, viewer_fps,
     motion_age_sec, pointcloud_age_sec, viewer_age_sec,
     same_source (0 or 1)]

Without a reply within 3 seconds Studio reports the OSC receiver as
"not connected" even though frames are arriving.

What the SDK reports:
    character       the requested character if it is being received, otherwise
                    the most recently received actor ("" before any frame)
    motion          complete frames per second assembled for ``character``
    pointcloud      not supported by the SDK: 0 fps, age -1 (Studio shows "-")
    viewer          frames per second handed to the application through
                    ``get_latest_frame()``; age -1 until the first one
    ages            seconds since the last such frame (-1 = never)
    same_source     1 if ``character``'s frames arrive from the same UDP address
                    that sent the request
"""

from __future__ import annotations

import threading
from collections import OrderedDict, deque

from .osc_writer import encode_osc_message


OSC_STATUS_REQUEST = "/MOVIN/OSC/Status/Request"
OSC_STATUS_REPLY = "/MOVIN/OSC/Status"
OSC_STATUS_VERSION = 1
OSC_STATUS_TYPETAGS = "sisffffffi"

RATE_WINDOW_SEC = 1.0
_MAX_CHARACTER_LEN = 256  # Studio rejects longer names
_MAX_TRACKED_ACTORS = 64
_HEX_DIGITS = frozenset("0123456789abcdefABCDEF")


class RateMeter:
    """Events per second over a sliding window, plus time since the last event."""

    def __init__(self, window_sec: float = RATE_WINDOW_SEC, max_events: int = 4096):
        self.window_sec = window_sec
        self._times = deque(maxlen=max_events)
        self._last = None  # survives the window pruning in rate()

    def tick(self, now: float):
        self._times.append(now)
        self._last = now

    def rate(self, now: float) -> float:
        times = self._times
        while times and now - times[0] > self.window_sec:
            times.popleft()
        if len(times) < 2:
            return 0.0
        span = times[-1] - times[0]
        return (len(times) - 1) / span if span > 0 else 0.0

    def age(self, now: float) -> float:
        return -1.0 if self._last is None else max(0.0, now - self._last)

    def clear(self):
        self._times.clear()
        self._last = None


def parse_status_request(args):
    """Validate a status request's arguments.

    Returns:
        ``(token, reply_port, character)`` or None if the request is malformed.
    """
    if len(args) != 3:
        return None
    token, reply_port, character = args
    if not (isinstance(token, str) and isinstance(reply_port, int) and isinstance(character, str)):
        return None
    if len(token) != 32 or not set(token) <= _HEX_DIGITS:
        return None
    if not 0 < reply_port < 65536:
        return None
    return token, reply_port, character


class StreamStatus:
    """Thread-safe bookkeeping for the status reply.

    ``on_frame`` runs on the receiver thread, ``on_consumed`` on the
    application thread that polls frames.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._actors = OrderedDict()  # actor -> (RateMeter, source address)
        self._last_actor = None
        self._viewer = RateMeter()

    def reset(self):
        with self._lock:
            self._actors.clear()
            self._last_actor = None
            self._viewer.clear()

    def on_frame(self, actor: str, source, now: float):
        """Record one complete frame of ``actor`` received from ``source``."""
        with self._lock:
            entry = self._actors.get(actor)
            if entry is None:
                meter = RateMeter()
                if len(self._actors) >= _MAX_TRACKED_ACTORS:
                    self._actors.popitem(last=False)
            else:
                meter = entry[0]
                self._actors.move_to_end(actor)
            meter.tick(now)
            self._actors[actor] = (meter, source)
            self._last_actor = actor

    def on_consumed(self, now: float):
        """Record one frame handed to the application."""
        with self._lock:
            self._viewer.tick(now)

    def reply_args(self, token: str, character: str, request_source, now: float):
        """Build the ``/MOVIN/OSC/Status`` arguments for one request."""
        with self._lock:
            if character not in self._actors:
                character = self._last_actor or ""
            entry = self._actors.get(character)
            if entry is not None:
                meter, source = entry
                motion, motion_age = meter.rate(now), meter.age(now)
                same_source = 1 if source == request_source else 0
            else:
                motion, motion_age, same_source = 0.0, -1.0, 0
            viewer, viewer_age = self._viewer.rate(now), self._viewer.age(now)
        return [
            token,
            OSC_STATUS_VERSION,
            character[:_MAX_CHARACTER_LEN],
            motion,
            0.0,          # point cloud fps: not received by the SDK
            viewer,
            motion_age,
            -1.0,         # point cloud age: unknown
            viewer_age,
            same_source,
        ]

    def reply_packet(self, token: str, character: str, request_source, now: float) -> bytes:
        """Encode the ``/MOVIN/OSC/Status`` reply for one request."""
        return encode_osc_message(
            OSC_STATUS_REPLY,
            self.reply_args(token, character, request_source, now),
            OSC_STATUS_TYPETAGS,
        )
