#!/usr/bin/env python3
"""
Example: Stream BVH-retargeted robot motion to GR00T over ZMQ (Protocol v1).

This script loads BVH motion from file (default: examples/test.bvh), retargets
it to a Unitree G1 robot in real-time, and publishes streamed reference motion
to a ZMQ endpoint using the same packed message format used by GR00T.

Usage:
    python mocap_to_robot_mujoco_stream.py --bvh_file examples/test.bvh --show_viewer
"""

import argparse
import json
import os
import sys
import time
import re
from typing import Generator, Tuple

import numpy as np

try:
    import zmq
except ImportError as exc:  # pragma: no cover - runtime dependency check
    raise ImportError(
        "pyzmq is required for ZMQ streaming. Install it with: pip install pyzmq"
    ) from exc

# Allow running without installing the package (add project root to path)
_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _root not in sys.path:
    sys.path.insert(0, _root)

from movin_sdk_python import MujocoViewer, Retargeter

HEADER_SIZE = 1280


def parse_bvh_frame_time(bvh_path: str) -> float | None:
    """Return BVH frame time in seconds if available."""
    frame_time_pattern = re.compile(r"^\s*Frame Time:\s*([0-9.+-eE]+)")
    try:
        with open(bvh_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                m = frame_time_pattern.match(line)
                if m:
                    return float(m.group(1))
    except OSError:
        return None
    return None


def pack_packed_message(
    topic: str,
    header_fields: list,
    payload_parts: Tuple[bytes, ...],
    version: int = 1,
    count: int = 1,
) -> bytes:
    """
    Pack a message in GR00T packed format.

    Layout:
        [topic][1280-byte JSON header][concatenated binary payload]
    """
    header = {
        "v": version,
        "endian": "le",
        "count": int(count),
        "fields": header_fields,
    }
    header_json = json.dumps(header, separators=(",", ":")).encode("utf-8")
    if len(header_json) > HEADER_SIZE:
        raise ValueError(
            f"ZMQ header is too large: {len(header_json)} > {HEADER_SIZE} bytes"
        )

    header_bytes = header_json.ljust(HEADER_SIZE, b"\x00")
    return topic.encode("utf-8") + header_bytes + b"".join(payload_parts)


def build_zmq_pose_message_v1(
    joint_pos: np.ndarray,
    joint_vel: np.ndarray,
    body_quat: np.ndarray,
    frame_index: np.ndarray,
    root_pos: np.ndarray | None = None,
    topic: str = "pose",
    catch_up: bool = True,
) -> bytes:
    """
    Build a Protocol v1 pose message for GR00T ZMQ streaming.
    Required fields:
      - joint_pos: [N, 29], f32
      - joint_vel: [N, 29], f32
      - body_quat: [N, 4], f32
      - root_pos/body_pos: [N, 3], f32 (optional, 로컬 기준 base translation)
      - frame_index: [N], i64
      - catch_up: [1], u8
    """
    joint_pos_arr = np.asarray(joint_pos, dtype=np.float32, order="C")
    joint_vel_arr = np.asarray(joint_vel, dtype=np.float32, order="C")
    body_quat_arr = np.asarray(body_quat, dtype=np.float32, order="C")
    root_pos_arr = np.asarray(root_pos, dtype=np.float32, order="C") if root_pos is not None else None
    frame_index_arr = np.asarray(frame_index, dtype=np.int64, order="C")

    if joint_pos_arr.ndim != 2:
        raise ValueError("joint_pos must be a 2-D array")
    if joint_vel_arr.ndim != 2:
        raise ValueError("joint_vel must be a 2-D array")
    if body_quat_arr.ndim != 2:
        raise ValueError("body_quat must be a 2-D array")
    if frame_index_arr.ndim != 1:
        raise ValueError("frame_index must be a 1-D array")

    if (
        joint_pos_arr.shape != joint_vel_arr.shape
        or joint_pos_arr.shape[0] == 0
        or body_quat_arr.shape[0] != joint_pos_arr.shape[0]
        or frame_index_arr.shape[0] != joint_pos_arr.shape[0]
    ):
        raise ValueError("Inconsistent pose message shapes")
    if root_pos_arr is not None:
        if root_pos_arr.ndim != 2 or root_pos_arr.shape[0] != joint_pos_arr.shape[0] or root_pos_arr.shape[1] != 3:
            raise ValueError("root_pos must have shape [N, 3] if provided")

    if body_quat_arr.shape[1] != 4:
        raise ValueError("body_quat must have shape [N, 4]")

    n_frames, n_joints = joint_pos_arr.shape
    if n_joints != 29:
        raise ValueError(f"joint_pos expected 29 dof, got {n_joints}")

    catch_up_arr = np.array([1 if catch_up else 0], dtype=np.uint8)

    header_fields = [
        {"name": "joint_pos", "dtype": "f32", "shape": [n_frames, n_joints]},
        {"name": "joint_vel", "dtype": "f32", "shape": [n_frames, n_joints]},
        {"name": "body_quat", "dtype": "f32", "shape": [n_frames, 4]},
    ]
    payload = [
        joint_pos_arr.tobytes(),
        joint_vel_arr.tobytes(),
        body_quat_arr.tobytes(),
    ]
    if root_pos_arr is not None:
        header_fields.append({"name": "root_pos", "dtype": "f32", "shape": [n_frames, 3]})
        header_fields.append({"name": "body_pos", "dtype": "f32", "shape": [n_frames, 3]})
        payload.append(root_pos_arr.tobytes())
        payload.append(root_pos_arr.tobytes())
    header_fields.extend(
        [
            {"name": "frame_index", "dtype": "i64", "shape": [n_frames]},
            {"name": "catch_up", "dtype": "u8", "shape": [1]},
        ]
    )
    payload.extend(
        [
            frame_index_arr.tobytes(),
            catch_up_arr.tobytes(),
        ]
    )

    return pack_packed_message(
        topic=topic,
        header_fields=header_fields,
        payload_parts=payload,
        version=1,
        count=n_frames,
    )


def stream_bvh_frames(
    frames,
    retargeter: Retargeter,
    send_fps: float,
    loop_bvh: bool = True,
) -> Generator[tuple, None, None]:
    """
    Generate retargeted frames from a BVH motion list.

    Yields:
      qpos, joint_pos, joint_vel, body_quat, root_pos, frame_index
    """
    if send_fps <= 0:
        raise ValueError("send_fps must be greater than 0")

    if not frames:
        return

    dt = 1.0 / float(send_fps)
    frame_offset = 0
    prev_joint = None
    prev_global_idx = None

    while True:
        for local_idx, frame in enumerate(frames):
            qpos = retargeter.retarget(frame)
            qpos = np.asarray(qpos, dtype=np.float64)

            if qpos.shape[0] < 36:
                raise ValueError(
                    f"Expected qpos length >= 36, got {qpos.shape[0]} in retargeted frame"
                )

            joint_pos = np.asarray(qpos[7:], dtype=np.float64)
            body_quat = np.asarray(qpos[3:7], dtype=np.float64)
            root_pos = np.asarray(qpos[:3], dtype=np.float64)
            frame_index = frame_offset + local_idx

            if prev_joint is not None and prev_global_idx is not None and frame_index == prev_global_idx + 1:
                joint_vel = (joint_pos - prev_joint) / dt
            else:
                joint_vel = np.zeros_like(joint_pos)

            yield qpos, joint_pos, joint_vel, body_quat, root_pos, np.int64(frame_index)

            prev_joint = joint_pos
            prev_global_idx = frame_index

        if not loop_bvh:
            break

        frame_offset += len(frames)


class ZMQPosePublisher:
    """Simple helper for publishing packed pose messages."""

    def __init__(
        self,
        host: str = "*",
        port: int = 5556,
        topic: str = "pose",
        use_conflate: bool = False,
    ):
        self.host = host
        self.port = port
        self.topic = topic
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.PUB)
        self.endpoint = f"tcp://{host}:{int(port)}"

        # Reduce stale backlog by default. Conflate is optional and can be enabled.
        self.socket.setsockopt(zmq.SNDHWM, 1 if use_conflate else 100)
        if use_conflate:
            try:
                self.socket.setsockopt(zmq.CONFLATE, 1)
            except zmq.ZMQError:
                # Some environments do not allow CONFLATE on PUB; keep working without it.
                pass

        self.socket.bind(self.endpoint)
        time.sleep(0.5)

        print(f"[ZMQPosePublisher] Bound on {self.endpoint}, topic='{self.topic}'")

    def send(self, message: bytes) -> None:
        self.socket.send(message, copy=False)

    def close(self) -> None:
        self.socket.close(0)
        self.context.term()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Stream retargeted BVH motion to GR00T ZMQ endpoint (Protocol v1)"
    )

    parser.add_argument(
        "--bvh_file",
        type=str,
        default="examples/test.bvh",
        help="BVH file path (default: examples/test.bvh)",
    )
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands"],
        default="unitree_g1",
        help="Target robot type",
    )
    parser.add_argument(
        "--human_height",
        type=float,
        default=1.75,
        help="Human height in meters for scaling (default: 1.75)",
    )
    parser.add_argument(
        "--send_fps",
        type=float,
        default=0.0,
        help="Send frame rate in Hz. 0 means infer from BVH Frame Time (default: 0)",
    )
    parser.add_argument(
        "--loop_bvh",
        action="store_true",
        default=True,
        help="Loop BVH motion continuously (default: True)",
    )
    parser.add_argument(
        "--no_loop_bvh",
        action="store_false",
        dest="loop_bvh",
        help="Play BVH only once and stop",
    )
    parser.add_argument(
        "--zmq_protocol",
        type=int,
        default=1,
        choices=[1],
        help="ZMQ protocol version (currently only 1 is supported)",
    )
    parser.add_argument(
        "--zmq_host",
        type=str,
        default="*",
        help="ZMQ bind host (default: *)",
    )
    parser.add_argument(
        "--zmq_port",
        type=int,
        default=5556,
        help="ZMQ bind port (default: 5556)",
    )
    parser.add_argument(
        "--zmq_topic",
        type=str,
        default="pose",
        help="ZMQ topic prefix (default: pose)",
    )
    parser.add_argument(
        "--zmq_conflate",
        action="store_true",
        default=False,
        help="Enable PUB socket conflation (latest message only)",
    )
    parser.add_argument(
        "--batch_size",
        type=int,
        default=1,
        help="Number of frames per ZMQ message batch (default: 1)",
    )
    parser.add_argument(
        "--catch_up",
        action="store_true",
        default=True,
        help="Enable catch_up behavior in streamed message (default: True)",
    )
    parser.add_argument(
        "--no_catch_up",
        action="store_false",
        dest="catch_up",
        help="Disable catch_up behavior in streamed message",
    )
    parser.add_argument(
        "--show_viewer",
        action="store_true",
        default=False,
        help="Enable MuJoCo viewer playback",
    )
    parser.add_argument(
        "--print_fps",
        action="store_true",
        default=False,
        help="Print streaming FPS every 2 seconds",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        default=False,
        help="Print debug output",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    bvh_path = os.path.expanduser(args.bvh_file)

    # Infer send_fps from BVH header when requested or when user passed 0.
    bvh_frame_time = parse_bvh_frame_time(bvh_path)
    inferred_fps = 30.0
    if bvh_frame_time and bvh_frame_time > 0:
        inferred_fps = 1.0 / bvh_frame_time

    if args.send_fps <= 0:
        args.send_fps = inferred_fps
    elif abs(args.send_fps - inferred_fps) > 1e-6:
        print(
            f"[Main] Warning: explicit --send_fps={args.send_fps:.4f} != BVH frame derived {inferred_fps:.4f}"
        )

    if args.batch_size <= 0:
        raise ValueError("--batch_size must be >= 1")
    if args.send_fps <= 0:
        raise ValueError("--send_fps must be > 0")
    if args.zmq_protocol != 1:
        raise ValueError("--zmq_protocol currently supports only 1")

    if not os.path.isabs(bvh_path):
        bvh_path = os.path.normpath(os.path.join(os.getcwd(), bvh_path))
    if not os.path.exists(bvh_path):
        raise FileNotFoundError(f"BVH file not found: {bvh_path}")

    print(f"[Main] Initializing Retargeter for {args.robot}...")
    retargeter = Retargeter(
        robot_type=args.robot,
        human_height=args.human_height,
        verbose=args.debug,
    )

    print(f"[Main] Loading BVH: {bvh_path}")
    frames, _human_height, _parents, _bones = retargeter.load_bvh(
        bvh_path, human_height=args.human_height
    )
    if len(frames) == 0:
        raise ValueError(f"No frames found in BVH file: {bvh_path}")

    print(f"[Main] Loaded BVH frames: {len(frames)}")
    print(f"[Main] ZMQ Protocol: v{args.zmq_protocol}, target: {args.zmq_host}:{args.zmq_port}/{args.zmq_topic}")
    print(f"[Main] Streaming options: send_fps={args.send_fps}, inferred_fps={inferred_fps:.4f}, batch_size={args.batch_size}, loop_bvh={args.loop_bvh}")

    publisher = ZMQPosePublisher(
        host=args.zmq_host,
        port=args.zmq_port,
        topic=args.zmq_topic,
        use_conflate=args.zmq_conflate,
    )

    viewer = None
    if args.show_viewer:
        viewer = MujocoViewer(robot_type=args.robot, motion_fps=int(args.send_fps))
        print("[Main] MuJoCo viewer enabled (rate limiting is disabled in sender loop)")

    send_interval = 1.0 / float(args.send_fps)
    next_tick = time.perf_counter()
    batch_joint_pos = []
    batch_joint_vel = []
    batch_body_quat = []
    batch_root_pos = []
    batch_frame_idx = []
    last_print = time.perf_counter()
    print_counter = 0
    frame_counter = 0

    try:
        for qpos, joint_pos, joint_vel, body_quat, root_pos, frame_idx in stream_bvh_frames(
            frames=frames,
            retargeter=retargeter,
            send_fps=args.send_fps,
            loop_bvh=args.loop_bvh,
        ):
            if viewer is not None:
                if not viewer.is_running():
                    print("[Main] Viewer closed. Exiting stream.")
                    break
                viewer.step(qpos, rate_limit=False)

            batch_joint_pos.append(joint_pos.astype(np.float32))
            batch_joint_vel.append(joint_vel.astype(np.float32))
            batch_body_quat.append(body_quat.astype(np.float32))
            batch_root_pos.append(root_pos.astype(np.float32))
            batch_frame_idx.append(np.array([frame_idx], dtype=np.int64))

            if len(batch_joint_pos) >= args.batch_size:
                stacked_joint_pos = np.stack(batch_joint_pos, axis=0)
                stacked_joint_vel = np.stack(batch_joint_vel, axis=0)
                stacked_body_quat = np.stack(batch_body_quat, axis=0)
                stacked_root_pos = np.stack(batch_root_pos, axis=0)
                stacked_frame_idx = np.concatenate(batch_frame_idx, axis=0)

                message = build_zmq_pose_message_v1(
                    joint_pos=stacked_joint_pos,
                    joint_vel=stacked_joint_vel,
                    body_quat=stacked_body_quat,
                    root_pos=stacked_root_pos,
                    frame_index=stacked_frame_idx,
                    topic=args.zmq_topic,
                    catch_up=args.catch_up,
                )
                publisher.send(message)

                if args.debug:
                    print(
                        "[ZMQPosePublisher] Sent "
                        f"{stacked_joint_pos.shape[0]} frames: "
                        f"frame_index={int(stacked_frame_idx[0])}..{int(stacked_frame_idx[-1])}, "
                        f"bytes={len(message)}"
                    )

                batch_joint_pos.clear()
                batch_joint_vel.clear()
                batch_body_quat.clear()
                batch_root_pos.clear()
                batch_frame_idx.clear()

                print_counter += 1
                frame_counter += args.batch_size

            now = time.perf_counter()
            next_tick += send_interval
            if args.print_fps:
                elapsed = now - last_print
                if elapsed >= 2.0:
                    stream_fps = frame_counter / elapsed
                    print(f"[Main] Streaming rate: {stream_fps:.2f} frames/sec, sent_packets={print_counter}")
                    print_counter = 0
                    frame_counter = 0
                    last_print = now

            sleep_sec = next_tick - time.perf_counter()
            if sleep_sec > 0:
                time.sleep(sleep_sec)
            else:
                next_tick = time.perf_counter()

        # Send remaining buffered frames when the stream ends (e.g. loop_bvh=False).
        if batch_joint_pos:
            stacked_joint_pos = np.stack(batch_joint_pos, axis=0)
            stacked_joint_vel = np.stack(batch_joint_vel, axis=0)
            stacked_body_quat = np.stack(batch_body_quat, axis=0)
            stacked_root_pos = np.stack(batch_root_pos, axis=0)
            stacked_frame_idx = np.concatenate(batch_frame_idx, axis=0)

            message = build_zmq_pose_message_v1(
                joint_pos=stacked_joint_pos,
                joint_vel=stacked_joint_vel,
                body_quat=stacked_body_quat,
                root_pos=stacked_root_pos,
                frame_index=stacked_frame_idx,
                topic=args.zmq_topic,
                catch_up=args.catch_up,
            )
            publisher.send(message)
            if args.debug:
                print(
                    "[ZMQPosePublisher] Sent "
                    f"{stacked_joint_pos.shape[0]} frames: "
                    f"frame_index={int(stacked_frame_idx[0])}..{int(stacked_frame_idx[-1])}, "
                    f"bytes={len(message)}"
                )
            print_counter += 1
            frame_counter += len(stacked_joint_pos)

    finally:
        if viewer is not None:
            viewer.close()
        if publisher is not None:
            publisher.close()
        print("[Main] Shutdown complete")


if __name__ == "__main__":
    main()
