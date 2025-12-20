#!/usr/bin/env python3
"""
Example: Real-time mocap to robot retargeting with MuJoCo visualization.

This script receives mocap data from MOVIN (via OSC over UDP), retargets
the motion to a robot, and visualizes the result in real-time using MuJoCo.

The data flow:
1. Receive local bone transforms from Unity via OSC (/MOVIN/Frame)
2. Process with Retargeter.process_mocap_frame()
3. Retarget to robot joint positions
4. Visualize in MuJoCo viewer

Usage:
    python3 mocap_to_robot_mujoco.py --port 11235 --robot unitree_g1 --human_height 1.75
    python3 mocap_to_robot_mujoco.py --port 11235 --robot unitree_g1 --print_fps --no_rate_limit
"""

import argparse
import time

from movin_sdk_python import Retargeter, MocapReceiver, MujocoViewer


def main():
    parser = argparse.ArgumentParser(description="Real-time mocap to robot with MuJoCo visualization")
    
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
        "--motion_fps",
        type=int,
        default=60,
        help="Target motion FPS for visualization (default: 60)",
    )
    
    parser.add_argument(
        "--no_rate_limit",
        action="store_true",
        default=False,
        help="Disable FPS rate limiting (run as fast as possible)",
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
    
    # Initialize the MuJoCo viewer
    print(f"[Main] Initializing MujocoViewer (target FPS: {args.motion_fps})...")
    viewer = MujocoViewer(
        robot_type=args.robot,
        motion_fps=args.motion_fps,
    )
    
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
    print("[Main] Close the viewer window or press Ctrl+C to stop")
    
    rate_limit = not args.no_rate_limit
    
    try:
        while viewer.is_running():
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
            
            # Visualize in MuJoCo
            viewer.step(qpos, rate_limit=rate_limit)
            
            # Debug output
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
        viewer.close()
        print("[Main] Done")


if __name__ == "__main__":
    main()
