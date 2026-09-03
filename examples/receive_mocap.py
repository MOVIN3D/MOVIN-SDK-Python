#!/usr/bin/env python3
"""
Example: Receive and print motion capture data via OSC protocol.

This script demonstrates the primary MovinSession workflow for receiving and
optionally recording mocap data from MOVIN Studio.

Usage:
    python receive_mocap.py --port 11235
    python receive_mocap.py --host 192.168.0.25 --port 12000
    python receive_mocap.py --port 11235 --verbose
    python receive_mocap.py --port 11235 --record session.pkl
"""

import argparse
import os
import sys
import time

# Allow running without installing the package (add project root to path)
_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _root not in sys.path:
    sys.path.insert(0, _root)

from movin_sdk_python import MovinSession


def main():
    parser = argparse.ArgumentParser(description="Receive and print mocap data via OSC")

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
        help="UDP port to listen for OSC data (default: 11235)",
    )
    
    parser.add_argument(
        "--verbose",
        action="store_true",
        default=False,
        help="Print detailed bone information",
    )
    
    parser.add_argument(
        "--print_rate",
        action="store_true",
        default=False,
        help="Print receive rate statistics",
    )

    parser.add_argument(
        "--record",
        metavar="PATH",
        help="Record the original OSC stream to PATH",
    )
    
    args = parser.parse_args()
    
    # Initialize the high-level receive/record session.
    print(f"[Main] Initializing MovinSession on {args.host}:{args.port}...")
    session = MovinSession(host=args.host, port=args.port)
    
    if args.record:
        session.start_recording(args.record)
        print(f"[Main] Recording raw OSC messages to {args.record}")
    # Start after the optional recorder is attached so the first packet is retained.
    session.start()
    
    # Frame counter for rate display
    frame_count = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0
    
    print(f"[Main] Waiting for mocap data on {args.host}:{args.port}...")
    print("[Main] Press Ctrl+C to stop")
    
    try:
        while True:
            # Get latest mocap frame
            frame = session.get_latest_frame()
            
            if frame is None:
                time.sleep(0.001)
                continue
            
            # Print frame info
            print(f"\n[Frame {frame['frame_idx']}] Actor: {frame['actor']}, "
                  f"Timestamp: {frame['timestamp']}, Bones: {len(frame['bones'])}")
            
            # Print bone details if verbose
            if args.verbose:
                for bone in frame['bones']:
                    pos = bone['p']
                    rot = bone['q']
                    print(f"  [{bone['bone_index']:2d}] {bone['bone_name']:20s} "
                          f"pos=({pos[0]:7.3f}, {pos[1]:7.3f}, {pos[2]:7.3f}) "
                          f"rot=({rot[0]:6.3f}, {rot[1]:6.3f}, {rot[2]:6.3f}, {rot[3]:6.3f})")
            else:
                # Just print bone names
                bone_names = [b['bone_name'] for b in frame['bones']]
                print(f"  Bones: {', '.join(bone_names[:5])}{'...' if len(bone_names) > 5 else ''}")
            
            # Update frame counter
            frame_count += 1
            
            # Print rate statistics
            if args.print_rate:
                current_time = time.time()
                if current_time - fps_start_time >= fps_display_interval:
                    fps = frame_count / (current_time - fps_start_time)
                    recv_rate = session.get_receive_rate()
                    print(f"[Main] Frame rate: {fps:.1f} fps, OSC receive rate: {recv_rate:.1f} Hz")
                    frame_count = 0
                    fps_start_time = current_time
                    
    except KeyboardInterrupt:
        print("\n[Main] Stopping...")
    finally:
        session.stop()
        print("[Main] Done")


if __name__ == "__main__":
    main()
