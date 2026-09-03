"""
OscRecorder - Records parsed OSC messages to a pickle file.

Records at the OSC message level so that replay exercises the same
assembler/chunk logic as live mode.
"""

from __future__ import annotations

import pickle
import threading
import time
from datetime import datetime, timezone


class OscRecorder:
    """Record OSC messages to a pickle file for later replay."""

    def __init__(self, output_path: str, stream_type: str = "auto"):
        """
        Initialize the recorder.

        Args:
            output_path: Path to write the pickle recording file.
            stream_type: One of "movin", "nova", or "auto" (auto-detect).
        """
        self.output_path = output_path
        self.stream_type = stream_type
        self._messages: list[dict] = []
        self._start_time: float | None = None
        self._detected_type: str | None = None
        self._lock = threading.RLock()

    def record(self, address: str, args: list, wall_time: float | None = None):
        """
        Append one parsed OSC message with timestamp.

        Args:
            address: OSC address string (e.g., "/MOVIN/Frame")
            args: Parsed OSC argument list
            wall_time: Wall-clock time in seconds (default: time.time())
        """
        with self._lock:
            now = wall_time if wall_time is not None else time.time()
            if self._start_time is None:
                self._start_time = now

            # Auto-detect stream type from address
            if self._detected_type is None and self.stream_type == "auto":
                if address in ("/MOVIN/Model/NOVA/Shape", "/MOVIN/Model/NOVA/Scale"):
                    self._detected_type = "nova"
                elif address == "/MOVIN/Frame":
                    # NOVA frames have an extra start_bone int field at args[7]
                    if (
                        len(args) > 10
                        and isinstance(args[7], int)
                        and isinstance(args[8], int)
                        and isinstance(args[9], int)
                        and isinstance(args[10], str)
                    ):
                        self._detected_type = "nova"
                    else:
                        self._detected_type = "movin"

            self._messages.append({
                "t": now - self._start_time,
                "addr": address,
                "args": list(args),
            })

    def save(self):
        """Write recording to disk as a pickle file."""
        with self._lock:
            stream_type = self.stream_type
            if stream_type == "auto":
                stream_type = self._detected_type or "unknown"

            duration = 0.0
            if self._messages and self._start_time is not None:
                duration = self._messages[-1]["t"]

            recording = {
                "version": 1,
                "stream_type": stream_type,
                "created": datetime.now(timezone.utc).isoformat(),
                "num_messages": len(self._messages),
                "duration_sec": duration,
                "messages": list(self._messages),
            }

        with open(self.output_path, "wb") as f:
            pickle.dump(recording, f, protocol=pickle.HIGHEST_PROTOCOL)

        print(
            f"[OscRecorder] Saved {len(self._messages)} messages "
            f"({duration:.1f}s, {stream_type}) to {self.output_path}"
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.save()
        return False
