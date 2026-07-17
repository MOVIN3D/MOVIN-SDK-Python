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
    use_velocity_limit=True,     # Enable velocity limits (default)
    source_preset="movinman",    # Source skeleton preset ("movinman" or "movinman_v3")
)
```

`source_preset` selects the source skeleton layout: `"movinman"` (legacy 51-body MOVINMan) or `"movinman_v3"` (54-joint MOVINManV3, adding `Spine2`/`Spine3`/`Neck1` and full finger chains). It selects which IK config is loaded — `"movinman_v3"` maps the G1 torso from `Spine3` instead of `Spine1` — and the constructor raises `ValueError` for an unrecognized preset. See [Skeleton Presets](#skeleton-presets) below for auto-detecting the preset from bone names.

**Methods:**

- `load_bvh(bvh_file, human_height=None)` - Load BVH file
  - Returns: `(frames, human_height, parents, bones)`
    - `frames`: list of frame data dictionaries
    - `human_height`: human height in meters
    - `parents`: numpy array of parent indices for skeleton hierarchy
    - `bones`: list of bone names
- `process_mocap_frame(bones)` - Process real-time mocap frame
- `retarget(human_data, offset_to_ground=False)` - Retarget to robot
- `get_required_bones()` - Get set of required bone names (depends on `source_preset`)
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

### Skeleton Presets

```python
from movin_sdk_python.utils.skeleton_presets import get_preset, detect_preset_from_bone_names

preset = get_preset("movinman_v3")
preset = detect_preset_from_bone_names(bone_names)
```

`movin_sdk_python.utils.skeleton_presets` is the single source of truth for MOVINMan skeleton layouts. Two presets are defined:

| Preset | Body count | Notes |
|--------|------------|-------|
| `movinman` | 51 (`MOVINMan.fbx`, legacy) | Has a mesh overlay asset (`movinman_mesh.npz`) |
| `movinman_v3` | 54 (`MOVINManV3`) | Adds `Spine2`, `Spine3`, `Neck1`, and full finger chains; no mesh overlay asset |

**`SkeletonPreset`** (frozen dataclass):
- `name`: preset identifier (`"movinman"` or `"movinman_v3"`)
- `body_names`: DFS body order including the root `"Hips"`
- `default_hips_height`: rest-pose hips height in meters (Z-up)
- `mjcf_filename`: MJCF asset filename
- `mesh_npz_filename`: LBS mesh asset filename, or `None` if the preset has no mesh overlay
- `joint_bones` (property): `body_names` minus the root — the bones that each drive 3 DOFs

**Functions:**
- `get_preset(name)` - Look up a preset by name; raises `KeyError` if unknown
- `detect_preset_from_bone_names(names)` - Return the V3 preset if any V3-only bone (`Spine3`, `Neck1`) is present in `names`, else the legacy preset

Several utilities in `isaac_lab_utils.py` and `movinman_mesh_utils.py` accept an optional bone-list parameter to target a non-default preset, defaulting to the legacy `movinman` layout: `extract_movin_local_quats_yup` / `extract_bvh_local_quats_yup` take `skeleton_body_names=` (full body list incl. `Hips`), `MOVINMeshModel(...)` takes `expected_bone_names=` (also a full body list incl. `Hips`), and `process_movin_bones_for_isaaclab` / `process_bvh_frame_for_isaaclab` / `build_dof_reorder_map` take `skeleton_bone_names=` (non-root list). `build_dof_reorder_map` raises `ValueError` if any expected joint is missing from the Isaac Lab articulation.

## Output Format

The `retarget()` method returns a numpy array `qpos`:

- `qpos[:3]` - Root position (x, y, z) in meters
- `qpos[3:7]` - Root orientation as quaternion (w, x, y, z)
- `qpos[7:]` - Joint angles in radians
