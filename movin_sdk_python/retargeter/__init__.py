"""
Retargeter - Motion retargeting from MOVIN mocap to Unitree robots.

This package provides a simple interface for retargeting live or replayed MOVIN
motion capture data to Unitree G1 robots.

Example usage:
    from retargeter import Retargeter
    
    # Initialize (MOVINManV3 is the default source skeleton)
    retargeter = Retargeter(robot_type="unitree_g1", human_height=1.75)
    
    # From a live or replayed mocap frame
    mocap_data = retargeter.process_mocap_frame(bones)
    qpos = retargeter.retarget(mocap_data)
"""

from ..exceptions import MissingOptionalDependencyError

try:
    from .retargeter import Retargeter
    from ..utils.fk_utils import (
        add_foot_mod_bones,
        compute_forward_kinematics,
        process_mocap_frame,
    )
except ModuleNotFoundError as exc:
    if exc.name and exc.name.startswith("movin_sdk_python"):
        raise
    raise MissingOptionalDependencyError(
        "Retargeting", "retargeting", exc.name
    ) from exc

__all__ = [
    "Retargeter",
    "process_mocap_frame",
    "compute_forward_kinematics",
    "add_foot_mod_bones",
]
