"""
Real-time optimized MuJoCo viewer for streaming mocap visualization.

This viewer is designed for high-frequency updates (60+ Hz) with minimal latency.
"""

import pathlib

import mujoco as mj
import mujoco.viewer as mjv
import numpy as np
from loop_rate_limiters import RateLimiter


# Package paths - reuse assets from retargeter
HERE = pathlib.Path(__file__).parent
ASSETS_ROOT = HERE.parent / "retargeter" / "assets"

# Robot XML paths
ROBOT_XML_DICT = {
    "unitree_g1": ASSETS_ROOT / "unitree_g1" / "g1_mocap_29dof.xml",
    "unitree_g1_with_hands": ASSETS_ROOT / "unitree_g1" / "g1_mocap_29dof_with_hands.xml",
}

# Robot base body name for camera tracking
ROBOT_BASE_DICT = {
    "unitree_g1": "pelvis",
    "unitree_g1_with_hands": "pelvis",
}

# Default camera distance for each robot
VIEWER_CAM_DISTANCE_DICT = {
    "unitree_g1": 3.0,
    "unitree_g1_with_hands": 3.0,
}


class MujocoViewer:
    """
    Real-time optimized MuJoCo viewer for streaming mocap visualization.
    
    Designed for high-frequency updates (60+ Hz) with minimal latency.
    Pre-caches body IDs and uses direct array operations in the hot path.
    
    Example:
        viewer = MujocoViewer(robot_type="unitree_g1", motion_fps=60)
        
        while viewer.is_running():
            qpos = retargeter.retarget(mocap_data)
            viewer.step(qpos)
        
        viewer.close()
    """
    
    def __init__(
        self,
        robot_type: str = "unitree_g1",
        motion_fps: int = 60,
        camera_distance: float = None,
        camera_elevation: float = -10.0,
        show_left_ui: bool = False,
        show_right_ui: bool = False,
        keyboard_callback=None,
    ):
        """
        Initialize the real-time MuJoCo viewer.
        
        Args:
            robot_type: Robot type ("unitree_g1" or "unitree_g1_with_hands")
            motion_fps: Target FPS for rate limiting (default: 60)
            camera_distance: Camera distance from robot (default: auto from robot type)
            camera_elevation: Camera elevation angle in degrees (default: -10)
            show_left_ui: Show MuJoCo left UI panel
            show_right_ui: Show MuJoCo right UI panel
            keyboard_callback: Optional keyboard callback function
        """
        if robot_type not in ROBOT_XML_DICT:
            raise ValueError(f"Unknown robot type: {robot_type}. "
                           f"Supported: {list(ROBOT_XML_DICT.keys())}")
        
        self.robot_type = robot_type
        self.motion_fps = motion_fps
        
        # Load model and create data
        xml_path = str(ROBOT_XML_DICT[robot_type])
        self.model = mj.MjModel.from_xml_path(xml_path)
        self.data = mj.MjData(self.model)
        
        # Initial forward pass
        mj.mj_step(self.model, self.data)
        
        # Pre-cache body ID for camera tracking (avoid name lookup in hot path)
        base_name = ROBOT_BASE_DICT[robot_type]
        self._base_body_id = self.model.body(base_name).id
        
        # Camera settings
        self._camera_distance = camera_distance or VIEWER_CAM_DISTANCE_DICT[robot_type]
        self._camera_elevation = camera_elevation
        
        # Rate limiter for FPS control
        self.rate_limiter = RateLimiter(frequency=motion_fps, warn=False)
        
        # Launch passive viewer (non-blocking)
        self.viewer = mjv.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=show_left_ui,
            show_right_ui=show_right_ui,
            key_callback=keyboard_callback,
        )
        
        # Initial camera setup
        self.viewer.cam.distance = self._camera_distance
        self.viewer.cam.elevation = self._camera_elevation
    
    def step(self, qpos, rate_limit: bool = True, follow_camera: bool = True) -> bool:
        """
        Update viewer with new robot state.
        
        This is the hot path - optimized for minimal latency.
        
        Args:
            qpos: Full qpos array [root_pos(3), root_rot(4), dof_pos(N)]
                  - qpos[:3] = root position (x, y, z)
                  - qpos[3:7] = root orientation (w, x, y, z quaternion)
                  - qpos[7:] = joint angles
            rate_limit: If True, sleep to maintain target FPS
            follow_camera: If True, camera follows the robot
            
        Returns:
            True if viewer is still running, False if closed
        """
        # Direct array copy (fastest)
        self.data.qpos[:] = qpos
        
        # Forward kinematics
        mj.mj_forward(self.model, self.data)
        
        # Camera follow (using pre-cached body ID)
        if follow_camera:
            self.viewer.cam.lookat[:] = self.data.xpos[self._base_body_id]
        
        # Sync viewer
        self.viewer.sync()
        
        # Optional rate limiting
        if rate_limit:
            self.rate_limiter.sleep()
        
        return self.viewer.is_running()
    
    def step_decomposed(
        self,
        root_pos,
        root_rot,
        dof_pos,
        rate_limit: bool = True,
        follow_camera: bool = True,
    ) -> bool:
        """
        Update viewer with decomposed robot state.
        
        Alternative to step() when qpos is not pre-assembled.
        
        Args:
            root_pos: Root position (3,)
            root_rot: Root orientation quaternion (w, x, y, z) (4,)
            dof_pos: Joint angles (N,)
            rate_limit: If True, sleep to maintain target FPS
            follow_camera: If True, camera follows the robot
            
        Returns:
            True if viewer is still running, False if closed
        """
        # Assign decomposed state
        self.data.qpos[:3] = root_pos
        self.data.qpos[3:7] = root_rot
        self.data.qpos[7:] = dof_pos
        
        # Forward kinematics
        mj.mj_forward(self.model, self.data)
        
        # Camera follow
        if follow_camera:
            self.viewer.cam.lookat[:] = self.data.xpos[self._base_body_id]
        
        # Sync viewer
        self.viewer.sync()
        
        # Optional rate limiting
        if rate_limit:
            self.rate_limiter.sleep()
        
        return self.viewer.is_running()
    
    def is_running(self) -> bool:
        """Check if viewer window is still open."""
        return self.viewer.is_running()
    
    def close(self):
        """Close the viewer window."""
        self.viewer.close()
        # Give the viewer thread time to properly shutdown
        import time
        time.sleep(0.5)
