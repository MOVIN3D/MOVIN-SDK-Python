#!/usr/bin/env python3
"""View raw MOVIN Studio mocap as a MuJoCo stick figure.

This example requires the viewer extra but does not use Retargeter:

    pip install -e ".[viewer]"
    python3 examples/view_mocap.py --port 11235
    python3 examples/view_mocap.py --host 192.168.0.25 --port 12000
"""

import argparse
import os
import sys
import time


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from movin_sdk_python import MocapViewer, MovinSession


def main():
    parser = argparse.ArgumentParser(
        description="View raw MOVIN mocap as a MuJoCo stick figure"
    )
    parser.add_argument(
        "--host",
        "--ip",
        dest="host",
        default="0.0.0.0",
        help="Local IPv4 address or hostname to bind (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=11235,
        help="UDP port to listen on (default: 11235)",
    )
    parser.add_argument(
        "--motion_fps",
        type=int,
        default=60,
        help="Viewer frame-rate limit (default: 60)",
    )
    parser.add_argument(
        "--record",
        metavar="PATH",
        help="Also record the original OSC stream to PATH",
    )
    args = parser.parse_args()

    viewer = MocapViewer(motion_fps=args.motion_fps)
    session = MovinSession(host=args.host, port=args.port, sinks=[viewer])
    if args.record:
        session.start_recording(args.record)

    print(f"[Main] Waiting for raw mocap data on UDP {args.host}:{args.port}...")
    print("[Main] Close the viewer or press Ctrl+C to stop")

    try:
        session.start()
        while viewer.is_running():
            if session.get_latest_frame() is None:
                time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        session.stop()
        viewer.close()


if __name__ == "__main__":
    main()
