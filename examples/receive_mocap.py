#!/usr/bin/env python3
"""
Example: Receive and print motion capture data via OSC protocol.

This script demonstrates how to use the MocapReceiver class to receive
mocap data from MOVIN STUDIO and print the received frame information.

Usage:
    python receive_mocap.py --port 11235
    python receive_mocap.py --port 11235 --verbose
"""

import argparse
import os
import sys
import time

# Allow running without installing the package (add project root to path)
_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _root not in sys.path:
    sys.path.insert(0, _root)

from movin_sdk_python import MocapReceiver


def main():
    parser = argparse.ArgumentParser(description="Receive and print mocap data via OSC")
    
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
    
    args = parser.parse_args()
    
    # Initialize the receiver
    print(f"[Main] Initializing MocapReceiver on port {args.port}...")
    receiver = MocapReceiver(port=args.port)
    
    # Start receiving
    receiver.start()
    
    # Frame counter for rate display
    frame_count = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0
    
    print(f"[Main] Waiting for mocap data on port {args.port}...")
    print("[Main] Press Ctrl+C to stop")
    
    try:
        while True:
            # Get latest mocap frame
            frame = receiver.get_latest_frame()
            
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
                    recv_rate = receiver.get_receive_rate()
                    print(f"[Main] Frame rate: {fps:.1f} fps, OSC receive rate: {recv_rate:.1f} Hz")
                    frame_count = 0
                    fps_start_time = current_time
                    
    except KeyboardInterrupt:
        print("\n[Main] Stopping...")
    finally:
        receiver.stop()
        print("[Main] Done")


if __name__ == "__main__":
    main()
