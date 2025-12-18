#!/usr/bin/env python3
"""
Example: BVH file to robot retargeting.

This script demonstrates how to use the Retargeter class to convert
BVH motion capture files to robot joint positions.

Usage:
    python bvh_to_robot.py --bvh_file path/to/motion.bvh --human_height 1.75

Output:
    - Prints retargeted joint positions for each frame
    - Optionally saves to pickle file
"""

import argparse
import os
import time
import numpy as np

from movin_sdk_python import Retargeter


def main():
    parser = argparse.ArgumentParser(description="BVH to robot retargeting")
    
    parser.add_argument(
        "--bvh_file",
        type=str,
        required=True,
        help="Path to BVH motion file",
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
        "--save_path",
        type=str,
        default=None,
        help="Path to save retargeted motion (pickle format)",
    )
    
    parser.add_argument(
        "--motion_fps",
        type=int,
        default=30,
        help="Motion FPS for saved data (default: 30)",
    )
    
    parser.add_argument(
        "--verbose",
        action="store_true",
        default=False,
        help="Print verbose output",
    )
    
    args = parser.parse_args()
    
    # Initialize the retargeter
    print(f"Initializing Retargeter for {args.robot}...")
    retargeter = Retargeter(
        robot_type=args.robot,
        human_height=args.human_height,
        verbose=args.verbose,
    )
    
    # Load BVH file
    print(f"Loading BVH file: {args.bvh_file}")
    frames, human_height = retargeter.load_bvh(args.bvh_file, human_height=args.human_height)
    print(f"Loaded {len(frames)} frames")
    
    # Retarget each frame
    qpos_list = []
    
    print("Retargeting...")
    start_time = time.time()
    
    for i, frame in enumerate(frames):
        qpos = retargeter.retarget(frame)
        qpos_list.append(qpos)
        
        if args.verbose and i % 100 == 0:
            print(f"  Frame {i}/{len(frames)}")
    
    elapsed = time.time() - start_time
    fps = len(frames) / elapsed
    print(f"Retargeting completed: {len(frames)} frames in {elapsed:.2f}s ({fps:.1f} fps)")
    
    # Print sample output
    print("\nSample output (first frame):")
    qpos = qpos_list[0]
    print(f"  Root position: {qpos[:3]}")
    print(f"  Root rotation (wxyz): {qpos[3:7]}")
    print(f"  Joint angles ({len(qpos)-7} DoFs): {qpos[7:]}")
    
    # Save to file if requested
    if args.save_path:
        import pickle
        
        # Create directory if needed
        save_dir = os.path.dirname(args.save_path)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        
        # Prepare data in standard format
        root_pos = np.array([qpos[:3] for qpos in qpos_list])
        # Convert quaternion from wxyz to xyzw format
        root_rot = np.array([qpos[3:7][[1, 2, 3, 0]] for qpos in qpos_list])
        dof_pos = np.array([qpos[7:] for qpos in qpos_list])
        
        motion_data = {
            "fps": args.motion_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,  # xyzw format
            "dof_pos": dof_pos,
            "local_body_pos": None,
            "link_body_list": None,
        }
        
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"\nSaved to {args.save_path}")
    
    return qpos_list


if __name__ == "__main__":
    main()
