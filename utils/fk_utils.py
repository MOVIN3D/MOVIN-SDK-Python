"""
Forward kinematics utilities for real-time motion capture data.

This module handles the conversion of local bone transforms (received from OSC)
to global transforms suitable for motion retargeting.
"""

import numpy as np

from .quat_utils import (
    quat_mul,
    quat_conj,
    quat_normalize,
    rotate_vec_by_quat,
    unity_to_opengl_vec,
    unity_to_opengl_quat,
)


def compute_forward_kinematics(bones):
    """
    Compute global transforms from local bone data using forward kinematics.
    
    This function:
    1. Removes rest pose offset: q = quat_mul(quat_conj(rq), q)
    2. Applies Unity->OpenGL coordinate conversion
    3. Computes FK to get global transforms
    4. Applies BVH-like axis conversion (90 deg around X)
    
    Args:
        bones: List of bone dicts with keys:
            - bone_index: int
            - parent_index: int (-1 for root)
            - bone_name: str
            - p: (px, py, pz) local position in Unity coords
            - q: (w, x, y, z) local rotation quaternion
            - rq: (w, x, y, z) rest pose quaternion
            - s: (sx, sy, sz) scale (not used)
        
    Returns:
        Dict mapping bone_name to [global_position (np.array), global_rotation_quat (np.array)]
    """
    # Build index map
    bone_by_index = {}
    for bone in bones:
        bone_by_index[bone["bone_index"]] = bone
    
    # Sort bones by index to process parents first
    sorted_bones = sorted(bones, key=lambda b: b["bone_index"])
    
    # Compute local transforms with rest pose removal and coordinate conversion
    local_transforms = {}  # bone_index -> (local_pos, local_rot)
    
    for bone in sorted_bones:
        idx = bone["bone_index"]
        
        # Get raw values
        p_unity = bone["p"]
        q_unity = bone["q"]   # (w, x, y, z)
        rq_unity = bone["rq"]  # rest pose (w, x, y, z)
        
        # Step 1: Apply Unity->OpenGL coordinate conversion
        p_opengl = unity_to_opengl_vec(p_unity)
        rq_opengl = unity_to_opengl_quat(rq_unity)
        q_opengl = unity_to_opengl_quat(q_unity)
        
        # Step 2: Remove rest pose offset
        # q_final = rq_inv * q
        rq_inv = quat_conj(rq_opengl)
        q_final = quat_mul(rq_inv, q_opengl)
        q_final = quat_normalize(q_final)
        
        local_transforms[idx] = (p_opengl, q_final)
    
    # Compute FK (global transforms)
    global_transforms = {}  # bone_index -> (global_pos, global_rot)
    
    for bone in sorted_bones:
        idx = bone["bone_index"]
        parent_idx = bone["parent_index"]
        local_pos, local_rot = local_transforms[idx]
        
        if parent_idx < 0 or parent_idx not in global_transforms:
            # Root bone - use local transform as global
            global_pos = local_pos
            global_rot = local_rot
        else:
            # Get parent's global transform
            parent_global_pos, parent_global_rot = global_transforms[parent_idx]
            
            # Global position = parent_global_pos + parent_global_rot * local_pos
            rotated_local_pos = rotate_vec_by_quat(local_pos, parent_global_rot)
            global_pos = parent_global_pos + rotated_local_pos
            
            # Global rotation = parent_global_rot * local_rot
            global_rot = quat_mul(parent_global_rot, local_rot)
            global_rot = quat_normalize(global_rot)
        
        global_transforms[idx] = (global_pos, global_rot)
    
    # Step 3: Apply BVH-like axis conversion (90 deg around X for Y-up to Z-up)
    # Rotation matrix: [[1,0,0], [0,0,-1], [0,1,0]]
    # As quaternion (w,x,y,z): (cos(45°), sin(45°), 0, 0) = (0.7071, 0.7071, 0, 0)
    rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
    rotation_quat = np.array([0.7071067811865476, 0.7071067811865476, 0.0, 0.0])  # 90 deg around X
    
    # Build result dict by bone name
    result = {}
    for bone in bones:
        idx = bone["bone_index"]
        name = bone["bone_name"]
        if idx in global_transforms:
            global_pos, global_rot = global_transforms[idx]
            
            # Apply axis conversion to position
            converted_pos = global_pos @ rotation_matrix.T
            
            # Apply axis conversion to rotation
            converted_rot = quat_mul(rotation_quat, global_rot)
            converted_rot = quat_normalize(converted_rot)
            
            result[name] = [converted_pos, converted_rot]
    
    return result


def add_foot_mod_bones(result):
    """
    Add FootMod entries for optitrack format.
    FootMod uses foot position with toe rotation.
    
    Args:
        result: Dict from compute_forward_kinematics
        
    Returns:
        Updated dict with FootMod entries added
    """
    if "LeftFoot" in result and "LeftToeBase" in result:
        result["LeftFootMod"] = [result["LeftFoot"][0].copy(), result["LeftToeBase"][1].copy()]
    if "RightFoot" in result and "RightToeBase" in result:
        result["RightFootMod"] = [result["RightFoot"][0].copy(), result["RightToeBase"][1].copy()]
    
    return result


def process_mocap_frame(bones):
    """
    Process a single mocap frame from OSC data.
    
    Args:
        bones: List of bone dicts from OSC receiver
        
    Returns:
        Dict mapping bone_name to [position, rotation] suitable for Retargeter
    """
    result = compute_forward_kinematics(bones)
    result = add_foot_mod_bones(result)
    return result
