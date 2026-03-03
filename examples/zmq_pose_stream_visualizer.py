#!/usr/bin/env python3
"""Visualize and validate ZMQ pose stream payload used for GR00T reference motion streaming.

Usage:
  python examples/zmq_pose_stream_visualizer.py --host localhost --port 5556 --topic pose

The tool subscribes to the ZMQ topic, decodes GR00T packed messages
([topic][1280-byte header][binary payload]), and prints/plots:
- protocol/version/header schema
- frame_index monotonicity and jumps
- joint_pos/joint_vel/body_quat frame statistics
- optional live plots for selected joints and quaternion W component
"""

import argparse
import json
import os
import sys
import time
from collections import deque
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

try:
    import zmq
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "pyzmq is required for ZMQ visualization. Install with: pip install pyzmq"
    ) from exc

HEADER_SIZES = [1280, 1024]  # Accept 1280 (C++), fallback 1024 for older senders
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
        raise ValueError(f"Unsupported dtype '{dtype_name}' in header") from exc


def _parse_header(payload: bytes) -> Tuple[int, dict, bytes]:
    """
    Try parsing a JSON header from the payload.
    Supports 1280-byte header (deployed C++ protocol) and 1024-byte fallback.
    """
    for header_size in HEADER_SIZES:
        if len(payload) <= header_size:
            continue

        header_raw = payload[:header_size]
        terminator = header_raw.find(b"\x00")
        header_len = terminator if terminator >= 0 else header_size
        header_text = header_raw[:header_len].decode("utf-8", errors="ignore").strip()
        if not header_text:
            continue

        try:
            header = json.loads(header_text)
        except json.JSONDecodeError:
            continue

        if "fields" not in header:
            continue
        return header_size, header, payload[header_size:]

    raise ValueError("Failed to locate valid JSON header in incoming message")


def _decode_message(
    topic: str,
    data: bytes,
) -> Tuple[dict, Optional[bytes], int]:
    topic_b = topic.encode("utf-8")
    if topic:
        if not data.startswith(topic_b):
            raise ValueError(f"Received message does not start with topic '{topic}'")
        payload = data[len(topic_b) :]
    else:
        payload = data

    header_size, header, bin_payload = _parse_header(payload)

    version = int(header.get("v", -1))
    endian = header.get("endian", "le")
    fields = header.get("fields", [])
    if not isinstance(fields, list) or len(fields) == 0:
        raise ValueError("Header missing 'fields'")

    data_le = endian == "le"
    needs_swap = (np.little_endian != data_le)

    decoded: Dict[str, np.ndarray] = {}
    offset = 0
    total = len(bin_payload)

    for field in fields:
        name = field.get("name")
        dtype_name = field.get("dtype")
        shape = field.get("shape", [])
        if name is None or dtype_name is None or not shape:
            continue

        dtype = _dtype_to_numpy(dtype_name)
        shape_tuple = tuple(int(x) for x in shape)
        n_elements = int(np.prod(shape_tuple))
        n_bytes = n_elements * dtype.itemsize
        if n_bytes < 0 or offset + n_bytes > total:
            raise ValueError(f"Field '{name}' exceeds payload size (offset={offset}, need={n_bytes}, total={total})")

        arr = np.frombuffer(bin_payload, dtype=dtype, count=n_elements, offset=offset)
        arr = arr.astype(dtype, copy=True)
        arr = arr.reshape(shape_tuple)
        if needs_swap and arr.nbytes > 1:
            arr = arr.byteswap().newbyteorder()
        decoded[name] = arr
        offset += n_bytes

    return {
        "header_size": header_size,
        "version": version,
        "header": header,
        "decoded": decoded,
        "payload_size": total,
        "used_payload": offset,
    }, bin_payload, offset


def _format_stats(arr: np.ndarray) -> str:
    if arr.size == 0:
        return "empty"
    return (
        f"shape={tuple(arr.shape)}, min={float(np.min(arr)):.6f}, "
        f"max={float(np.max(arr)):.6f}, mean={float(np.mean(arr)):.6f}"
    )


def _init_plot(selected_joints: Sequence[int], window: int):
    try:
        import matplotlib.pyplot as plt  # type: ignore
        from matplotlib.animation import FuncAnimation  # type: ignore
    except ImportError:
        return None

    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    t_line = axes[0]
    q_line = axes[1]
    t_line.set_title("Frame Index")
    t_line.set_xlabel("time/sample")
    t_line.set_ylabel("frame_index")

    q_line.set_title(
        "Joint positions (selected joints: %s)" % ",".join(map(str, selected_joints))
    )
    q_line.set_xlabel("time/sample")
    q_line.set_ylabel("joint_pos (rad)")

    lines = {}
    x = deque(maxlen=window)
    y_by_joint = {idx: deque(maxlen=window) for idx in selected_joints}
    quat_w = deque(maxlen=window)

    # optional third axis for quaternion w
    ax3 = q_line.twinx()
    quat_line, = ax3.plot([], [], "k--", label="body_quat_w", linewidth=1.0)
    ax3.set_ylabel("body_quat_w")

    for idx in selected_joints:
        (line,) = q_line.plot([], [], label=f"joint{idx}", linewidth=1.0)
        lines[idx] = line

    q_line.legend(loc="upper right")
    ax3.legend(loc="lower right")
    fig.tight_layout()

    return {
        "plt": plt,
        "fig": fig,
        "axes": axes,
        "t_line": t_line,
        "q_line": q_line,
        "ax3": ax3,
        "lines": lines,
        "quat_line": quat_line,
        "x": x,
        "y_by_joint": y_by_joint,
        "quat_w": quat_w,
        "func_animation": FuncAnimation,
    }


def _update_plot_state(plot_ctx, frame_indexes: np.ndarray, joint_pos: np.ndarray, body_quat: np.ndarray) -> None:
    if plot_ctx is None:
        return

    plt = plot_ctx["plt"]
    x = plot_ctx["x"]
    y_by_joint = plot_ctx["y_by_joint"]
    quat_w = plot_ctx["quat_w"]
    lines = plot_ctx["lines"]

    selected_vals = {idx: joint_pos[:, idx] for idx in y_by_joint.keys()}
    for i in range(len(frame_indexes)):
        x.append(int(frame_indexes[i]))
        for idx, line_vals in selected_vals.items():
            y_by_joint[idx].append(float(line_vals[i]))
        # body_quat shape can be [N, 4] or [N, 1, 4]
        bq = body_quat[i]
        if bq.ndim == 2:
            quat_w.append(float(bq[0]))
        elif bq.ndim == 1 and bq.shape[-1] == 4:
            quat_w.append(float(bq[0]))

    if not x:
        return

    xs = np.array(x)
    plot_ctx["t_line"].relim()
    plot_ctx["t_line"].autoscale_view()

    for idx, ln in lines.items():
        ys = np.array(y_by_joint[idx])
        ln.set_data(xs[-len(ys):], ys)

    plot_ctx["q_line"].relim()
    plot_ctx["q_line"].autoscale_view()

    qw = np.array(quat_w)
    plot_ctx["quat_line"].set_data(xs[-len(qw):], qw)
    plot_ctx["ax3"].relim()
    plot_ctx["ax3"].autoscale_view()

    plot_ctx["fig"].canvas.draw_idle()
    plt.pause(0.001)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect and visualize GR00T ZMQ pose stream packets."
    )
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=5556)
    parser.add_argument("--topic", type=str, default="pose")
    parser.add_argument("--timeout_ms", type=int, default=2000)
    parser.add_argument("--conflate", action="store_true", default=False)
    parser.add_argument("--print_every", type=int, default=10, help="Log every N packets")
    parser.add_argument("--frame_jump_limit", type=int, default=2, help="Warn if frame gap exceeds this")
    parser.add_argument("--selected_joints", type=str, default="11,12")
    parser.add_argument("--window", type=int, default=1000, help="Plot window length in samples")
    parser.add_argument("--plot", action="store_true", default=True)
    parser.add_argument("--no_plot", action="store_false", dest="plot")
    parser.add_argument("--duration_sec", type=float, default=0.0, help="Auto-stop after seconds (0=inf)")
    parser.add_argument("--max_packets", type=int, default=0, help="Auto-stop after N packets (0=inf)")
    return parser.parse_args()


def _resolve_selected_joints(raw: str) -> Sequence[int]:
    vals = []
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        vals.append(int(token))
    return vals if vals else [11, 12]


def main() -> None:
    args = parse_args()
    selected_joints = _resolve_selected_joints(args.selected_joints)

    print(f"[visualizer] Connecting to tcp://{args.host}:{args.port}, topic='{args.topic}'")
    ctx = zmq.Context.instance()
    socket = ctx.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, args.topic.encode("utf-8"))
    socket.setsockopt(zmq.RCVTIMEO, args.timeout_ms)
    if args.conflate:
        socket.setsockopt(zmq.CONFLATE, 1)
    socket.connect(f"tcp://{args.host}:{args.port}")

    plot_ctx = None
    if args.plot:
        plot_ctx = _init_plot(selected_joints=selected_joints, window=args.window)
        if plot_ctx is None:
            print("[visualizer] matplotlib unavailable; plotting disabled")

    socket_open = True
    prev_frame = None
    packet_count = 0
    sample_count = 0
    start = time.time()

    try:
        while True:
            try:
                data = socket.recv()
            except zmq.Again:
                print("[visualizer] No packet received within timeout")
                continue

            parsed, _payload, used = _decode_message(args.topic, data)
            header = parsed["header"]
            decoded = parsed["decoded"]
            version = parsed["version"]

            if "joint_pos" not in decoded:
                raise ValueError("Missing required field 'joint_pos'")
            if "joint_vel" not in decoded:
                raise ValueError("Missing required field 'joint_vel'")
            if "frame_index" not in decoded:
                raise ValueError("Missing required field 'frame_index'")
            if "body_quat" not in decoded:
                if "body_quat_w" not in decoded:
                    raise ValueError("Missing required field 'body_quat' (or 'body_quat_w')")
                decoded["body_quat"] = decoded.pop("body_quat_w")
            root_pos = decoded.get("root_pos")
            if root_pos is None:
                root_pos = decoded.get("body_pos")

            joint_pos = np.asarray(decoded["joint_pos"])
            joint_vel = np.asarray(decoded["joint_vel"])
            body_quat = np.asarray(decoded["body_quat"])
            if root_pos is not None:
                root_pos = np.asarray(root_pos)
            frame_index = np.asarray(decoded["frame_index"]).reshape(-1)
            catch_up = decoded.get("catch_up")
            if catch_up is not None and catch_up.size > 0:
                catch_up_val = int(catch_up.reshape(-1)[0])
            else:
                catch_up_val = 1

            packet_count += 1
            sample_count += int(frame_index.size)

            # Frame sequence checks
            bad_jump = None
            gap_info = None
            if prev_frame is not None and frame_index.size > 0:
                diffs = np.diff(frame_index)
                if diffs.size == 0:
                    gap_info = (0, 0)
                else:
                    if np.any(diffs < 0):
                        bad_jump = "negative"
                    max_gap = int(np.max(diffs))
                    gap_info = (int(np.min(diffs)), max_gap)
                    if max_gap > args.frame_jump_limit:
                        bad_jump = "large_gap"
                prev_frame = int(frame_index[-1])
            else:
                prev_frame = int(frame_index[0]) if frame_index.size else prev_frame

            if packet_count % args.print_every == 0:
                print(
                    f"[packet #{packet_count}] v{version} "
                    f"frames={frame_index.size} idx={int(frame_index[0]) if frame_index.size else -1}"
                    f"..{int(frame_index[-1]) if frame_index.size else -1} "
                    f"used_payload={used}/{parsed['payload_size']} bytes "
                    f"header={parsed['header_size']}B catch_up={bool(catch_up_val)}"
                )
                print(f"  joint_pos: {_format_stats(joint_pos)}")
                print(f"  joint_vel: {_format_stats(joint_vel)}")
                print(f"  body_quat: {_format_stats(body_quat)}")
                if root_pos is not None:
                    print(f"  root_pos: {_format_stats(root_pos)}")
                if gap_info is not None:
                    print(
                        f"  frame_diff: min={gap_info[0]}, max={gap_info[1]}, "
                        f"status={'OK' if bad_jump is None else bad_jump}"
                    )
                print(f"  fields: {[f['name'] for f in header.get('fields', [])]}")

            if plot_ctx is not None and frame_index.size > 0:
                # Clip to valid selected_joint range for safety.
                valid_joints = [j for j in selected_joints if 0 <= j < joint_pos.shape[1]]
                if not valid_joints:
                    raise ValueError(f"selected_joints {selected_joints} out of range (njoints={joint_pos.shape[1]})")
                _update_plot_state(plot_ctx, frame_index, joint_pos[:, valid_joints], body_quat.reshape(len(frame_index), -1, 4))

            if args.duration_sec > 0 and (time.time() - start) >= args.duration_sec:
                break
            if args.max_packets > 0 and packet_count >= args.max_packets:
                break

    except KeyboardInterrupt:
        pass
    finally:
        if socket_open:
            socket.close(0)
            if args.print_every:
                elapsed = max(time.time() - start, 1e-6)
                print(
                    f"[visualizer] received packets={packet_count}, frames={sample_count}, "
                    f"avg_fps={sample_count/elapsed:.2f}"
                )
            print("[visualizer] Bye")


if __name__ == "__main__":
    main()
