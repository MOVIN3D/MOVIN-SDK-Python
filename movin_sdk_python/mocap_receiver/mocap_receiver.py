"""
MocapReceiver - Real-time motion capture data receiver via OSC protocol.

This module provides the MocapReceiver class for receiving motion capture data
from MOVIN (via Unity) over UDP using the OSC protocol. It handles frame
assembly from chunked packets in a background thread.
"""

import socket
import threading
import time

from .osc_reader import OscReader
from .movin_frame_assembler import MovinFrameAssembler


class MocapReceiver:
    """
    Manages OSC reception and frame assembly in a background thread.

    Receives motion capture data from MOVIN via OSC over UDP, assembles
    multi-chunk frames, and provides access to the latest complete frame.

    The data flow:
    1. UDP packets arrive containing OSC messages (/MOVIN/Frame)
    2. Each frame may be split across multiple chunks
    3. Chunks are assembled into complete frames
    4. Complete frames are stored in a queue for consumption

    Example usage:
        receiver = MocapReceiver(port=11235)
        receiver.start()

        while running:
            frame = receiver.get_latest_frame()
            if frame:
                print(f"Frame {frame['frame_idx']}: {len(frame['bones'])} bones")
                for bone in frame['bones']:
                    print(f"  {bone['bone_name']}: pos={bone['p']}")

        receiver.stop()

    Frame format:
        {
            "timestamp": str,        # Timestamp from mocap system
            "actor": str,            # Actor name
            "frame_idx": int,        # Frame index
            "bones": [               # List of bone data
                {
                    "bone_index": int,
                    "parent_index": int,
                    "bone_name": str,
                    "p": (px, py, pz),           # Local position
                    "rq": (w, x, y, z),          # Rest pose quaternion
                    "q": (w, x, y, z),           # Local rotation quaternion
                    "s": (sx, sy, sz),           # Scale
                },
                ...
            ]
        }
    """

    def __init__(self, port: int = 11235, recorder=None):
        """
        Initialize the MocapReceiver.

        Args:
            port: UDP port to listen on (default: 11235)
            recorder: Optional OscRecorder instance to record incoming messages
        """
        self.port = port
        self.recorder = recorder
        self.thread = None
        self.sock = None
        self.running = False
        self.lock = threading.Lock()
        self.assembler = MovinFrameAssembler(max_ready_frames=4)
        self.last_applied = None
        self.last_actor = ""
        self.last_ts = ""
        self.recv_count = 0
        self.last_rate_time = time.time()
        self.recv_rate_hz = 0.0

    def reset(self):
        """Reset all internal state and buffers."""
        with self.lock:
            self.assembler = MovinFrameAssembler(max_ready_frames=4)
            self.last_applied = None
            self.last_actor = ""
            self.last_ts = ""
            self.recv_count = 0
            self.recv_rate_hz = 0.0

    def start(self):
        """Start the UDP server thread."""
        self.reset()
        self.running = True
        self.thread = threading.Thread(target=self._udp_server_loop, daemon=True)
        self.thread.start()
        print(f"[MocapReceiver] Listening on UDP port {self.port}")

    def stop(self):
        """Stop the UDP server thread."""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        time.sleep(0.1)
        self.thread = None
        self.sock = None
        self.reset()
        print("[MocapReceiver] Stopped")

    def get_latest_frame(self):
        """
        Get the most recent complete frame, clearing older frames.

        Returns:
            Frame dict if available, None otherwise.
            Frame contains: timestamp, actor, frame_idx, bones
        """
        with self.lock:
            frame = self.assembler.pop_latest_frame()
            if frame is not None:
                self.last_applied = frame["frame_idx"]
                self.last_actor = frame["actor"]
                self.last_ts = frame["timestamp"]
            return frame

    def get_receive_rate(self):
        """
        Get the current packet receive rate.

        Returns:
            Receive rate in Hz (packets per second)
        """
        return self.recv_rate_hz

    def _udp_server_loop(self):
        """Background thread that receives OSC packets and assembles frames."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        except OSError:
            pass
        sock.bind(("0.0.0.0", self.port))
        sock.settimeout(0.5)
        self.sock = sock

        try:
            while self.running:
                try:
                    data, _addr = sock.recvfrom(65535)
                except socket.timeout:
                    now = time.time()
                    with self.lock:
                        self.assembler.prune_stale(now=now)
                    continue
                except OSError:
                    break

                try:
                    reader = OscReader(data)
                    address, args = reader.read_message()
                except Exception as e:
                    print(f"[MocapReceiver] OSC parse error: {e}")
                    continue

                if self.recorder:
                    self.recorder.record(address, args)

                now = time.time()
                with self.lock:
                    self.assembler.ingest(address, args, now=now)

                    # Update receive rate
                    self.recv_count += 1
                    dt = now - self.last_rate_time
                    if dt >= 1.0:
                        self.recv_rate_hz = self.recv_count / dt
                        self.recv_count = 0
                        self.last_rate_time = now

        finally:
            try:
                sock.close()
            except Exception:
                pass
            self.sock = None
