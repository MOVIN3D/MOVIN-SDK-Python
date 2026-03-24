"""
MovinFrameAssembler - Assembles chunked MOVIN OSC packets into complete frames.

Mirrors the NovaLiveStreamAssembler API: `ingest(address, args, now)`,
`pop_latest_frame()`, `prune_stale()`.
"""

from __future__ import annotations

import time
from collections import deque


class MovinFrameAssembler:
    """Assemble chunked MOVIN /MOVIN/Frame OSC packets into complete frames."""

    def __init__(self, max_ready_frames=4, partial_ttl_sec=0.5):
        self.partial_ttl_sec = partial_ttl_sec
        self.frame_buffers = {}
        self.ready_frames = deque(maxlen=max_ready_frames)
        self.warned_nova_stream = False

    def prune_stale(self, now=None):
        """Remove partial frame buffers older than partial_ttl_sec."""
        now = time.time() if now is None else now
        stale = [k for k, v in self.frame_buffers.items()
                 if now - v.get("_t0", now) > self.partial_ttl_sec]
        for k in stale:
            del self.frame_buffers[k]

    def pop_latest_frame(self):
        """Return the most recent complete frame and discard older ones."""
        if not self.ready_frames:
            return None
        frame = self.ready_frames.pop()
        self.ready_frames.clear()
        return frame

    def ingest(self, address, args, now=None):
        """
        Ingest a parsed OSC message. Only processes /MOVIN/Frame addresses.

        Args:
            address: OSC address string
            args: Parsed OSC argument list
            now: Current wall-clock time (default: time.time())
        """
        if address != "/MOVIN/Frame":
            return
        now = time.time() if now is None else now
        self._ingest_frame(args, now)

    def _ingest_frame(self, args, now):
        try:
            ts = args[0]
            actor_name = args[1]
            frame_idx = int(args[2])
            num_chunks = int(args[3])
            chunk_idx = int(args[4])
            total_bones = int(args[5])
            chunk_bones = int(args[6])
        except Exception:
            return

        # NOVA uses /MOVIN/Frame too, but with an extra start_bone header field and
        # 3x3 rotation matrices instead of quaternions. Detect and skip.
        if (
            len(args) > 10
            and isinstance(args[7], int)
            and isinstance(args[8], int)
            and isinstance(args[9], int)
            and isinstance(args[10], str)
        ):
            if not self.warned_nova_stream:
                print(
                    "[MovinFrameAssembler] Detected NOVA matrix stream on /MOVIN/Frame. "
                    "Use NovaMocapReceiver for this sender."
                )
                self.warned_nova_stream = True
            return

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
        except Exception:
            return

        # Assemble frame from chunks
        key = (actor_name, frame_idx)
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
            del self.frame_buffers[key]
