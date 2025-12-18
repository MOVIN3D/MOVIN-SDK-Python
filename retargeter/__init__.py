"""
Retargeter - Motion retargeting from optitrack mocap to Unitree robots.

This package provides a simple interface for retargeting human motion capture data
to Unitree G1 robots. It supports both BVH file playback and real-time OSC streaming.

Example usage:
    from retargeter import Retargeter
    
    # Initialize
    retargeter = Retargeter(robot_type="unitree_g1", human_height=1.75)
    
    # From BVH file
    frames, height = retargeter.load_bvh("motion.bvh")
    for frame in frames:
        qpos = retargeter.retarget(frame)
        # qpos[:3] = root position
        # qpos[3:7] = root orientation (wxyz quaternion)
        # qpos[7:] = joint angles
    
    # From real-time mocap
    mocap_data = retargeter.process_mocap_frame(bones)
    qpos = retargeter.retarget(mocap_data)
"""

from .retargeter import Retargeter
from ..utils.bvh_loader import load_bvh_file
from ..utils.fk_utils import process_mocap_frame, compute_forward_kinematics, add_foot_mod_bones

__all__ = [
    "Retargeter",
    "load_bvh_file",
    "process_mocap_frame",
    "compute_forward_kinematics",
    "add_foot_mod_bones",
]
