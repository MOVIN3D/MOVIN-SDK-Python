"""
OscPlayer - Loads and replays recorded OSC messages.
"""

from __future__ import annotations

import pickle
import time
from typing import Iterator


class OscPlayer:
    """Load a recorded OSC file and iterate over messages."""

    def __init__(self, recording_path: str):
        """
        Load a recording file.

        Args:
            recording_path: Path to a pickle recording file saved by OscRecorder.
        """
        with open(recording_path, "rb") as f:
            self._recording = pickle.load(f)

        if self._recording.get("version", 0) != 1:
            raise ValueError(
                f"Unsupported recording version: {self._recording.get('version')}"
            )

    @property
    def stream_type(self) -> str:
        """Stream type: 'movin', 'nova', or 'unknown'."""
        return self._recording.get("stream_type", "unknown")

    @property
    def num_messages(self) -> int:
        """Total number of recorded messages."""
        return self._recording.get("num_messages", 0)

    @property
    def duration_sec(self) -> float:
        """Duration of the recording in seconds."""
        return self._recording.get("duration_sec", 0.0)

    def messages(self, realtime: bool = False) -> Iterator[tuple[str, list]]:
        """
        Yield (address, args) for each recorded message.

        Args:
            realtime: If True, sleep between messages to match original timing.
        """
        msgs = self._recording.get("messages", [])
        start = time.time()
        for msg in msgs:
            if realtime and msg["t"] > 0:
                elapsed = time.time() - start
                delay = msg["t"] - elapsed
                if delay > 0:
                    time.sleep(delay)
            yield msg["addr"], msg["args"]

    def messages_with_timing(self) -> Iterator[tuple[float, str, list]]:
        """
        Yield (relative_time, address, args) without sleeping.
        """
        for msg in self._recording.get("messages", []):
            yield msg["t"], msg["addr"], msg["args"]
