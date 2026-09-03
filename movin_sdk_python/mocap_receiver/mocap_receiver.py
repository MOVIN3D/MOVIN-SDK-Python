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
        receiver = MocapReceiver(host="0.0.0.0", port=11235)
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

    def __init__(
        self,
        port: int = 11235,
        recorder=None,
        *,
        host: str = "0.0.0.0",
    ):
        """
        Initialize the MocapReceiver.

        Args:
            port: UDP port to listen on (default: 11235)
            recorder: Optional OscRecorder instance to record incoming messages
            host: Local IPv4 address or hostname to bind (default: 0.0.0.0,
                meaning all local IPv4 interfaces)
        """
        self.host = host
        self.port = port
        self._recorder_lock = threading.RLock()
        self._recorder = recorder
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

    @property
    def recorder(self):
        """Recorder currently attached to the raw OSC stream."""
        with self._recorder_lock:
            return self._recorder

    @recorder.setter
    def recorder(self, recorder):
        # Serializes recorder replacement with in-flight record() calls. This
        # lets MovinSession safely detach a recorder before saving it.
        with self._recorder_lock:
            self._recorder = recorder

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
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        except OSError:
            pass

        try:
            sock.bind((self.host, self.port))
            sock.settimeout(0.5)
        except Exception:
            sock.close()
            self.running = False
            self.sock = None
            raise

        self.sock = sock
        self.running = True
        self.thread = threading.Thread(
            target=self._udp_server_loop,
            args=(sock,),
            daemon=True,
        )
        self.thread.start()
        bound_host, bound_port = sock.getsockname()[:2]
        print(f"[MocapReceiver] Listening on UDP {bound_host}:{bound_port}")

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

    def _udp_server_loop(self, sock):
        """Background thread that receives OSC packets and assembles frames."""
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

                with self._recorder_lock:
                    if self._recorder:
                        self._recorder.record(address, args)

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
            if self.sock is sock:
                self.sock = None
