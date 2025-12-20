# API Reference

## MocapReceiver Class

```python3
MocapReceiver(port=11235)
```

Receives mocap data from MOVIN via OSC over UDP.

**Methods:**

- `start()` - Start the background receiver thread
- `stop()` - Stop the receiver and cleanup
- `get_latest_frame()` - Get the most recent complete frame (or None)
- `get_receive_rate()` - Get current packet receive rate in Hz
- `reset()` - Reset all internal buffers

**Frame Format:**

```python
{
    "timestamp": str,        # Timestamp from mocap system
    "actor": str,            # Actor name
    "frame_idx": int,        # Frame index
    "bones": [               # List of bone data
        {
            "bone_index": int,
            "parent_index": int,
            "bone_name": str,
            "p": (px, py, pz),           # Local position
            "rq": (w, x, y, z),          # Rest pose quaternion
            "q": (w, x, y, z),           # Local rotation quaternion
            "s": (sx, sy, sz),           # Scale
        },
        ...
    ]
}
```

## Retargeter Class

```python
Retargeter(
    robot_type="unitree_g1",     # "unitree_g1" or "unitree_g1_with_hands"
    human_height=1.75,           # Human height in meters
    solver="daqp",               # IK solver
    damping=0.5,                 # IK damping
    verbose=False,               # Debug output
    use_velocity_limit=False,    # Enable velocity limits
)
```

**Methods:**

- `load_bvh(bvh_file, human_height=None)` - Load BVH file
  - Returns: `(frames, human_height, parents, bones)`
    - `frames`: list of frame data dictionaries
    - `human_height`: human height in meters
    - `parents`: numpy array of parent indices for skeleton hierarchy
    - `bones`: list of bone names
- `process_mocap_frame(bones)` - Process real-time mocap frame
- `retarget(human_data, offset_to_ground=False)` - Retarget to robot
- `get_required_bones()` - Get set of required bone names
- `set_ground_offset(offset)` - Set ground height offset

## MujocoViewer Class

```python
MujocoViewer(
    robot_type="unitree_g1",     # "unitree_g1" or "unitree_g1_with_hands"
    motion_fps=60,               # Target FPS for rate limiting
    camera_distance=3.0,         # Camera distance from robot
    camera_elevation=-10.0,      # Camera elevation angle
    show_left_ui=False,          # Show MuJoCo left UI panel
    show_right_ui=False,         # Show MuJoCo right UI panel
)
```

Real-time optimized MuJoCo viewer for streaming mocap visualization. Designed for high-frequency updates (60+ Hz) with minimal latency.

**Methods:**

- `step(qpos, rate_limit=True, follow_camera=True)` - Update viewer with new robot state, returns `False` if viewer closed
- `step_decomposed(root_pos, root_rot, dof_pos, ...)` - Alternative API with separate position/rotation/joints
- `is_running()` - Check if viewer window is still open
- `close()` - Close the viewer window

## Utility Functions

### load_bvh_file

```python
from movin_sdk_python import load_bvh_file

frames, human_height, parents, bones = load_bvh_file(bvh_file, human_height=1.75)
```

Load a BVH file and return frame data with skeleton hierarchy information.

**Arguments:**
- `bvh_file`: Path to BVH file
- `human_height`: Assumed human height in meters (default: 1.75)

**Returns:**
- `frames`: List of dictionaries with bone names as keys and `[position, orientation]` as values
- `human_height`: Assumed human height in meters
- `parents`: Numpy array of parent indices for each joint (skeleton hierarchy)
- `bones`: List of bone names

This function is similar to `BVHAnimation` from `read_bvh()` but returns processed frame data with coordinate transformations applied (Y-up to Z-up).

## Output Format

The `retarget()` method returns a numpy array `qpos`:

- `qpos[:3]` - Root position (x, y, z) in meters
- `qpos[3:7]` - Root orientation as quaternion (w, x, y, z)
- `qpos[7:]` - Joint angles in radians
