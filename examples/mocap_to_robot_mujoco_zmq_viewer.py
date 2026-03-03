#!/usr/bin/env python3
"""
Visualize ZMQ pose stream on a MuJoCo G1 robot.

This subscribes to the same GR00T pose message format used by
 mocap_to_robot_mujoco_stream.py and feeds decoded joint data into MujocoViewer.

Usage:
  python mocap_to_robot_mujoco_zmq_viewer.py --host localhost --port 5556 --robot unitree_g1
"""

import argparse
import json
import os
import sys
import time
from typing import Dict, Tuple

import numpy as np

try:
    import zmq
except ImportError as exc:
    raise ImportError(
        "pyzmq is required. Install with: pip install pyzmq"
    ) from exc

# Allow running without installing the package.
_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _root not in sys.path:
    sys.path.insert(0, _root)

from movin_sdk_python import MujocoViewer

HEADER_SIZES = [1280, 1024]  # GR00T-style default is 1280.
DTYPE_MAP = {
    "f32": np.float32,
    "f64": np.float64,
    "i32": np.int32,
    "i64": np.int64,
    "u8": np.uint8,
    "i8": np.int8,
    "bool": np.uint8,
}


def _dtype_to_numpy(dtype_name: str) -> np.dtype:
    try:
        return np.dtype(DTYPE_MAP[dtype_name])
    except KeyError as exc:
        raise ValueError(f"Unsupported dtype in pose header: {dtype_name}") from exc


def _extract_header(payload: bytes) -> Tuple[int, dict, bytes]:
    """
    Parse and return (header_size, header_dict, binary_payload).
    Accepts both 1024-byte and 1280-byte header sizes.
    """
    for header_size in HEADER_SIZES:
        if len(payload) <= header_size:
            continue

        header_raw = payload[:header_size]
        null_pos = header_raw.find(b"\x00")
        if null_pos == -1:
            header_text = header_raw.decode("utf-8", errors="ignore").strip()
        else:
            header_text = header_raw[:null_pos].decode("utf-8", errors="ignore").strip()

        if not header_text:
            continue

        try:
            header = json.loads(header_text)
        except json.JSONDecodeError:
            continue

        if "fields" not in header:
            continue

        return header_size, header, payload[header_size:]

    raise ValueError("Failed to parse a valid JSON header from message payload")


def _decode_pose_payload(topic: str, data: bytes) -> Tuple[dict, bytes, int, int]:
    topic_b = topic.encode("utf-8")
    if topic:
        if not data.startswith(topic_b):
            raise ValueError(f"Message does not start with topic '{topic}'")
        data = data[len(topic_b) :]
    else:
        data = data

    header_size, header, binary_payload = _extract_header(data)

    header_fields = header.get("fields", [])
    endian = header.get("endian", "le")
    native_le = np.little_endian
    data_le = endian == "le"
    needs_swap = native_le != data_le
    payload_size = len(binary_payload)

    field_by_name: Dict[str, np.ndarray] = {}
    byte_offset = 0
    for field in header_fields:
        name = field.get("name")
        dtype_name = field.get("dtype")
        shape = field.get("shape", [])
        if not name or dtype_name is None or not shape:
            continue

        dtype = _dtype_to_numpy(dtype_name)
        shape_tuple = tuple(int(x) for x in shape)
        n_elements = int(np.prod(shape_tuple))
        if n_elements < 0:
            raise ValueError(f"Invalid shape for field '{name}': {shape_tuple}")

        n_bytes = n_elements * dtype.itemsize
        if byte_offset + n_bytes > payload_size:
            raise ValueError(
                f"Field '{name}' exceeds payload: need {n_bytes} bytes, available {payload_size - byte_offset}"
            )

        arr = np.frombuffer(
            binary_payload,
            dtype=dtype,
            count=n_elements,
            offset=byte_offset,
        ).reshape(shape_tuple)
        if needs_swap and arr.dtype.itemsize > 1:
            arr = arr.byteswap().newbyteorder()

        field_by_name[name] = arr.copy()
        byte_offset += n_bytes

    return {
        "header": header,
        "header_size": header_size,
        "fields": field_by_name,
        "payload_size": payload_size,
        "used_bytes": byte_offset,
        "version": int(header.get("v", -1)),
    }, binary_payload, byte_offset, payload_size


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay GR00T pose stream with MuJoCo viewer."
    )
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=5556)
    parser.add_argument("--topic", type=str, default="pose")
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands"],
        default="unitree_g1",
    )
    parser.add_argument("--motion_fps", type=float, default=60.0)
    parser.add_argument("--conflate", action="store_true", default=False)
    parser.add_argument("--timeout_ms", type=int, default=3000)
    parser.add_argument("--print_fps", action="store_true", default=False)
    parser.add_argument("--debug", action="store_true", default=False)
    parser.add_argument("--no_rate_limit", action="store_true", default=False)
    parser.add_argument("--no_follow_camera", action="store_true", default=False)
    return parser.parse_args()


def _build_qpos(
    joint_pos: np.ndarray,
    body_quat: np.ndarray,
    root_pos: np.ndarray | None = None,
) -> np.ndarray:
    n_frames, n_joints = joint_pos.shape
    if n_joints == 0:
        raise ValueError("joint_pos has no joints")

    qpos = np.zeros((n_frames, n_joints + 7), dtype=np.float64)
    if root_pos is not None:
        if root_pos.shape != (n_frames, 3):
            raise ValueError(f"root_pos shape must be (N,3), got {root_pos.shape}")
        qpos[:, :3] = root_pos.astype(np.float64)
    qpos[:, 7:] = joint_pos.astype(np.float64)
    qpos[:, 3:7] = body_quat.astype(np.float64)
    return qpos


def main() -> None:
    args = parse_args()

    print(f"[viewer] SUB connect tcp://{args.host}:{args.port}, topic='{args.topic}'")
    context = zmq.Context.instance()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, args.topic.encode("utf-8"))
    socket.setsockopt(zmq.RCVTIMEO, args.timeout_ms)
    if args.conflate:
        socket.setsockopt(zmq.CONFLATE, 1)
    socket.connect(f"tcp://{args.host}:{args.port}")

    # Initialize MuJoCo viewer
    viewer = MujocoViewer(robot_type=args.robot, motion_fps=int(args.motion_fps))
    print(
        f"[viewer] Initialized MujocoViewer(robot={args.robot}, motion_fps={int(args.motion_fps)})"
    )

    packet_count = 0
    frame_count = 0
    total_frames_rendered = 0
    last_report = time.perf_counter()
    last_idx = None

    try:
        while viewer.is_running():
            try:
                data = socket.recv()
            except zmq.Again:
                print("[viewer] No packet received (timeout)")
                continue
            except KeyboardInterrupt:
                break

            parsed, _payload, used_bytes, payload_size = _decode_pose_payload(args.topic, data)
            header = parsed["header"]
            fields = parsed["fields"]

            packet_count += 1
            frame_count += 1
            if args.debug:
                print(
                    f"[viewer] packet#{packet_count} v={parsed['version']} "
                    f"fields={list(fields.keys())} used={used_bytes}/{payload_size} "
                    f"header={parsed['header_size']}B"
                )

            if "joint_pos" not in fields or "frame_index" not in fields:
                if args.debug:
                    print("[viewer] Skip packet: missing joint_pos or frame_index")
                continue

            joint_pos = np.asarray(fields["joint_pos"])
            joint_vel = np.asarray(fields.get("joint_vel", np.zeros((0, 0), dtype=np.float32)))
            body_quat = fields.get("body_quat")
            if body_quat is None:
                body_quat = fields.get("body_quat_w")
            if body_quat is None:
                if args.debug:
                    print("[viewer] Skip packet: missing body_quat/body_quat_w")
                continue
            root_pos = fields.get("root_pos")
            if root_pos is None:
                root_pos = fields.get("body_pos")

            if body_quat.ndim > 2 and body_quat.shape[1] != 1 and body_quat.shape[1] != 4:
                if args.debug:
                    print(f"[viewer] Unexpected body_quat shape: {body_quat.shape}")
                continue

            frame_index = np.asarray(fields["frame_index"]).reshape(-1)
            if joint_pos.ndim != 2 or frame_index.ndim != 1:
                if args.debug:
                    print(f"[viewer] Skip packet due invalid rank: joint_pos={joint_pos.shape}, frame_index={frame_index.shape}")
                continue

            if joint_pos.shape[0] != frame_index.shape[0]:
                if args.debug:
                    print(
                        f"[viewer] Skip packet due mismatch nframes: "
                        f"joint_pos={joint_pos.shape[0]}, frame_index={frame_index.shape[0]}"
                    )
                continue

            if body_quat.ndim == 2:
                quat_for_frame = body_quat
            elif body_quat.ndim == 3 and body_quat.shape[1] == 1:
                quat_for_frame = body_quat[:, 0, :]
            else:
                # Any unsupported 3D shape is still rendered from first body by best effort.
                quat_for_frame = body_quat[:, 0, :]

            if quat_for_frame.shape[0] != joint_pos.shape[0] or quat_for_frame.shape[1] != 4:
                if args.debug:
                    print(
                        f"[viewer] Skip packet due invalid body_quat shape: {quat_for_frame.shape}"
                    )
                continue

            if root_pos is not None:
                root_pos = np.asarray(root_pos)
                if root_pos.ndim != 2 or root_pos.shape[1] != 3:
                    if args.debug:
                        print(f"[viewer] Skip packet due invalid root_pos shape: {root_pos.shape}")
                    continue
                if root_pos.shape[0] != joint_pos.shape[0]:
                    if args.debug:
                        print(
                            f"[viewer] Skip packet due mismatched root_pos nframes: "
                            f"{root_pos.shape[0]} vs {joint_pos.shape[0]}"
                        )
                    continue

            qpos = _build_qpos(joint_pos, quat_for_frame, root_pos=root_pos)

            if args.debug:
                root_span = np.max(qpos[:, :3], axis=0) - np.min(qpos[:, :3], axis=0)
                print(
                    f"[viewer] frames={qpos.shape[0]} idx={int(frame_index[0]) if frame_index.size else -1}"
                    f"..{int(frame_index[-1]) if frame_index.size else -1} "
                    f"joint_vel_shape={joint_vel.shape if isinstance(joint_vel, np.ndarray) else 'n/a'}"
                    f" root_span={root_span}"
                )

            # Optional frame continuity sanity log.
            if last_idx is not None and frame_index.size:
                diff = int(frame_index[0] - last_idx)
                if diff < 0:
                    print(f"[viewer] WARN: frame_index decreased ({last_idx} -> {int(frame_index[0])})")
                elif diff > 1:
                    print(f"[viewer] INFO: frame_index jump {diff-1} (expected +1)")
            if frame_index.size:
                last_idx = int(frame_index[-1])

            for local_i in range(qpos.shape[0]):
                if not viewer.is_running():
                    break
                viewer.step(
                    qpos[local_i],
                    rate_limit=not args.no_rate_limit,
                    follow_camera=not args.no_follow_camera,
                )
                total_frames_rendered += 1

            if args.print_fps:
                now = time.perf_counter()
                if now - last_report >= 2.0:
                    elapsed = now - last_report
                    print(
                        f"[viewer] rendered_fps~{total_frames_rendered / elapsed:.2f} "
                        f"(frames={total_frames_rendered}, packets={packet_count})"
                    )
                    total_frames_rendered = 0
                    last_report = now

    finally:
        socket.close(0)
        viewer.close()
        print("[viewer] Done")


if __name__ == "__main__":
    main()
