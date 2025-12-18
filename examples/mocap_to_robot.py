#!/usr/bin/env python3
"""
Example: Real-time mocap to robot retargeting via OSC protocol.

This script receives mocap data from MOVIN (via OSC over UDP), processes the
bone transforms, and retargets the motion to a robot in real-time.

The data flow:
1. Receive local bone transforms from Unity via OSC (/MOVIN/Frame)
2. Process with Retargeter.process_mocap_frame()
3. Retarget to robot joint positions

Usage:
    python mocap_to_robot.py --port 11235 --robot unitree_g1 --human_height 1.75
"""

import argparse
import time

from movin_sdk_python import Retargeter, MocapReceiver


def main():
    parser = argparse.ArgumentParser(description="Real-time mocap to robot retargeting via OSC")
    
    parser.add_argument(
        "--port",
        type=int,
        default=11235,
        help="UDP port to listen for OSC data (default: 11235)",
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
        "--print_fps",
        action="store_true",
        default=False,
        help="Print FPS information",
    )
    
    parser.add_argument(
        "--debug",
        action="store_true",
        default=False,
        help="Print debug information",
    )
    
    args = parser.parse_args()
    
    # Initialize the retargeter
    print(f"[Main] Initializing Retargeter for {args.robot}...")
    retargeter = Retargeter(
        robot_type=args.robot,
        human_height=args.human_height,
        verbose=args.debug,
    )
    
    # Get required bones for validation
    required_bones = retargeter.get_required_bones()
    
    # Start the mocap receiver
    receiver = MocapReceiver(port=args.port)
    receiver.start()
    
    # FPS measurement
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0
    
    # Warning tracking
    warned_missing = 0
    
    print(f"[Main] Waiting for mocap data on port {args.port}...")
    print("[Main] Press Ctrl+C to stop")
    
    try:
        while True:
            # Get latest mocap frame
            frame = receiver.get_latest_frame()
            
            if frame is None:
                time.sleep(0.001)
                continue
            
            # Debug: print bone names on first frame
            if args.debug and warned_missing == 0:
                bone_names = [b["bone_name"] for b in frame["bones"]]
                print(f"[Debug] Received {len(bone_names)} bones: {bone_names}")
            
            # Process mocap frame
            mocap_data = retargeter.process_mocap_frame(frame["bones"])
            
            # Check for required bones
            missing = sorted(required_bones.difference(mocap_data.keys()))
            if missing:
                if warned_missing < 5:
                    warned_missing += 1
                    print(f"[Main] Warning: missing {len(missing)} required bones: {missing}")
                continue
            
            # Filter to only required bones
            mocap_data = {k: mocap_data[k] for k in required_bones if k in mocap_data}
            
            # Retarget
            qpos = retargeter.retarget(mocap_data)
            
            # Print output (for demonstration)
            if args.debug:
                print(f"[Frame {frame['frame_idx']}] root_pos={qpos[:3]}, dof_pos={qpos[7:12]}...")
            
            # FPS measurement
            if args.print_fps:
                fps_counter += 1
                current_time = time.time()
                if current_time - fps_start_time >= fps_display_interval:
                    actual_fps = fps_counter / (current_time - fps_start_time)
                    recv_rate = receiver.get_receive_rate()
                    print(f"[Main] Retargeting FPS: {actual_fps:.1f}, OSC receive rate: {recv_rate:.1f} Hz")
                    fps_counter = 0
                    fps_start_time = current_time
                    
    except KeyboardInterrupt:
        print("\n[Main] Stopping...")
    finally:
        receiver.stop()
        print("[Main] Done")


if __name__ == "__main__":
    main()
