"""
MOVIN SDK Python - Motion capture to robot retargeting SDK.

This package provides tools for receiving motion capture data from MOVIN
(via Unity) and retargeting it to robots in real-time.

Main classes:
    - MocapReceiver: Receives mocap data via OSC over UDP
    - Retargeter: Retargets human motion to robot joint positions
    - MujocoViewer: Real-time visualization in MuJoCo

Example usage:
    from movin_sdk_python import MocapReceiver, Retargeter, MujocoViewer
    
    # Initialize
    receiver = MocapReceiver(port=11235)
    retargeter = Retargeter(robot_type="unitree_g1", human_height=1.75)
    viewer = MujocoViewer(robot_type="unitree_g1", motion_fps=60)
    
    # Start receiving
    receiver.start()
    
    # Main loop
    while viewer.is_running():
        frame = receiver.get_latest_frame()
        if frame:
            mocap_data = retargeter.process_mocap_frame(frame["bones"])
            qpos = retargeter.retarget(mocap_data)
            # qpos[:3] = root position
            # qpos[3:7] = root orientation (wxyz quaternion)
            # qpos[7:] = joint angles
            viewer.step(qpos)
    
    # Cleanup
    receiver.stop()
    viewer.close()
"""

# Import from subpackages
from .mocap_receiver import MocapReceiver, OscReader
from .retargeter import Retargeter
from .utils import load_bvh_file, process_mocap_frame
from .viewer import MujocoViewer

__version__ = "0.1.0"
__all__ = [
    "MocapReceiver",
    "OscReader",
    "Retargeter",
    "MujocoViewer",
    "load_bvh_file",
    "process_mocap_frame",
]
