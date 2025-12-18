"""
MocapReceiver - Real-time motion capture data receiver via OSC protocol.

This module provides the MocapReceiver class for receiving motion capture data
from MOVIN (via Unity) over UDP using the OSC protocol. It handles frame
assembly from chunked packets in a background thread.
"""

import socket
import threading
import time
from collections import deque

from .osc_reader import OscReader


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
    
    def __init__(self, port: int = 11235):
        """
        Initialize the MocapReceiver.
        
        Args:
            port: UDP port to listen on (default: 11235)
        """
        self.port = port
        self.thread = None
        self.sock = None
        self.running = False
        self.lock = threading.Lock()
        self.frame_buffers = {}
        self.ready_frames = deque(maxlen=4)
        self.last_applied = None
        self.last_actor = ""
        self.last_ts = ""
        self.recv_count = 0
        self.last_rate_time = time.time()
        self.recv_rate_hz = 0.0

    def reset(self):
        """Reset all internal state and buffers."""
        with self.lock:
            self.frame_buffers.clear()
            self.ready_frames.clear()
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
            if not self.ready_frames:
                return None
            frame = self.ready_frames.pop()
            self.ready_frames.clear()
            self.last_applied = frame["frame_idx"]
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

        PARTIAL_TTL_SEC = 0.5

        try:
            while self.running:
                try:
                    data, _addr = sock.recvfrom(65535)
                except socket.timeout:
                    now = time.time()
                    with self.lock:
                        stale = [k for k, v in self.frame_buffers.items()
                                if now - v.get("_t0", now) > PARTIAL_TTL_SEC]
                        for k in stale:
                            del self.frame_buffers[k]
                    continue
                except OSError:
                    break

                try:
                    reader = OscReader(data)
                    address, args = reader.read_message()
                except Exception as e:
                    print(f"[MocapReceiver] OSC parse error: {e}")
                    continue

                if address != "/MOVIN/Frame":
                    continue

                try:
                    ts = args[0]
                    actor_name = args[1]
                    frame_idx = int(args[2])
                    num_chunks = int(args[3])
                    chunk_idx = int(args[4])
                    total_bones = int(args[5])
                    chunk_bones = int(args[6])
                except Exception as e:
                    print(f"[MocapReceiver] Bad header args: {e}")
                    continue

                # Parse bone data from this chunk
                k = 7
                bones_in_chunk = []
                try:
                    for _ in range(chunk_bones):
                        bone_index = int(args[k]); k += 1
                        parent_index = int(args[k]); k += 1
                        bone_name = args[k]; k += 1
                        px = float(args[k]); py = float(args[k+1]); pz = float(args[k+2]); k += 3
                        # Rest quaternion: Unity sends (x,y,z,w), convert to (w,x,y,z)
                        rqx = float(args[k]); rqy = float(args[k+1]); rqz = float(args[k+2]); rqw = float(args[k+3]); k += 4
                        # Local quaternion: Unity sends (x,y,z,w), convert to (w,x,y,z)
                        qx = float(args[k]); qy = float(args[k+1]); qz = float(args[k+2]); qw = float(args[k+3]); k += 4
                        sx = float(args[k]); sy = float(args[k+1]); sz = float(args[k+2]); k += 3
                        bones_in_chunk.append({
                            "bone_index": bone_index,
                            "parent_index": parent_index,
                            "bone_name": bone_name,
                            "p": (px, py, pz),
                            "rq": (rqw, rqx, rqy, rqz),  # (w,x,y,z)
                            "q": (qw, qx, qy, qz),        # (w,x,y,z)
                            "s": (sx, sy, sz),
                        })
                except Exception as e:
                    print(f"[MocapReceiver] Truncated/invalid bone block: {e}")
                    continue

                # Assemble frame from chunks
                key = (actor_name, frame_idx)
                now = time.time()
                with self.lock:
                    buf = self.frame_buffers.get(key)
                    if buf is None:
                        buf = {
                            "_t0": now,
                            "timestamp": ts,
                            "actor": actor_name,
                            "frame_idx": frame_idx,
                            "num_chunks": num_chunks,
                            "total_bones": total_bones,
                            "chunks": {},
                        }
                        self.frame_buffers[key] = buf

                    buf["chunks"][chunk_idx] = bones_in_chunk

                    # Check if frame is complete
                    if len(buf["chunks"]) >= buf["num_chunks"]:
                        ordered = []
                        complete = True
                        for ci in range(buf["num_chunks"]):
                            part = buf["chunks"].get(ci)
                            if not part:
                                complete = False
                                break
                            ordered.extend(part)
                        if complete and ordered:
                            frame = {
                                "timestamp": buf["timestamp"],
                                "actor": buf["actor"],
                                "frame_idx": buf["frame_idx"],
                                "bones": ordered,
                            }
                            self.ready_frames.append(frame)
                            self.last_actor = buf["actor"]
                            self.last_ts = buf["timestamp"]
                        del self.frame_buffers[key]

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
