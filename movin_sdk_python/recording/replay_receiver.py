"""
Replay receivers - Drop-in replacements for MocapReceiver
that replay recorded OSC data instead of listening on a UDP socket.
"""

from __future__ import annotations

import threading
import time

from .osc_player import OscPlayer
from ..mocap_receiver.movin_frame_assembler import MovinFrameAssembler


class ReplayMocapReceiver:
    """
    Drop-in replacement for MocapReceiver that replays recorded MOVIN data.

    Same API: start(), stop(), get_latest_frame(), get_receive_rate().
    """

    def __init__(self, recording_path: str, realtime: bool = True, loop: bool = True):
        self.player = OscPlayer(recording_path)
        self.assembler = MovinFrameAssembler(max_ready_frames=4)
        self.realtime = realtime
        self.loop = loop
        self.thread = None
        self.running = False
        self.lock = threading.Lock()
        self.recv_count = 0
        self.last_rate_time = time.time()
        self.recv_rate_hz = 0.0

    def start(self):
        """Spawn background thread that feeds recorded messages into assembler."""
        self.running = True
        self.thread = threading.Thread(target=self._replay_loop, daemon=True)
        self.thread.start()
        print(
            f"[ReplayMocapReceiver] Replaying {self.player.num_messages} messages "
            f"({self.player.duration_sec:.1f}s, loop={self.loop})"
        )

    def stop(self):
        """Stop the replay thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        self.thread = None
        print("[ReplayMocapReceiver] Stopped")

    def get_latest_frame(self):
        """Same API as MocapReceiver - returns latest assembled frame."""
        with self.lock:
            return self.assembler.pop_latest_frame()

    def get_receive_rate(self):
        """Get the current message replay rate in Hz."""
        return self.recv_rate_hz

    def _replay_loop(self):
        while self.running:
            for address, args in self.player.messages(realtime=self.realtime):
                if not self.running:
                    return
                now = time.time()
                with self.lock:
                    self.assembler.ingest(address, args, now=now)
                    self.recv_count += 1
                    dt = now - self.last_rate_time
                    if dt >= 1.0:
                        self.recv_rate_hz = self.recv_count / dt
                        self.recv_count = 0
                        self.last_rate_time = now
            if not self.loop:
                break
