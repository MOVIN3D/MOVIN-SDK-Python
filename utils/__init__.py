"""
Utility functions for motion capture processing and retargeting.

This module provides:
    - bvh_loader: BVH file parsing and loading
    - fk_utils: Forward kinematics utilities for mocap data
    - quat_utils: Quaternion math utilities
"""

from .bvh_loader import load_bvh_file
from .fk_utils import process_mocap_frame, compute_forward_kinematics, add_foot_mod_bones
from .quat_utils import quat_mul, rotate_vec_by_quat

__all__ = [
    "load_bvh_file",
    "process_mocap_frame",
    "compute_forward_kinematics",
    "add_foot_mod_bones",
    "quat_mul",
    "rotate_vec_by_quat",
]
