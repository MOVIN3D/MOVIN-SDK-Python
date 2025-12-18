# MOVIN SDK Python

A self-contained Python SDK for receiving motion capture data from MOVIN and retargeting it to robots in real-time.

## Features

- **Real-time Mocap Receiver**: Receive mocap data via OSC protocol from MOVIN (Unity)
- **BVH File Support**: Load and retarget BVH motion capture files
- **Motion Retargeting**: Retarget human motion to robot joint positions using IK
- **Unitree G1 Support**: Both standard G1 (29 DoF) and G1 with hands variants
- **Self-contained**: All assets (robot models, meshes, IK configs) included

## Installation

### Option 1: Install via pip (Recommended)

```bash
# Install from local path
pip install /path/to/movin_sdk_python

# Or install in editable mode for development
pip install -e /path/to/movin_sdk_python
```

### Option 2: Copy to your project

1. Copy this folder to your project
2. Install dependencies:

```bash
pip install -r requirements.txt
```

## Quick Start

### Real-time Mocap to Robot

```python
from movin_sdk_python import MocapReceiver, Retargeter

# Initialize
receiver = MocapReceiver(port=11235)
retargeter = Retargeter(robot_type="unitree_g1", human_height=1.75)

# Start receiving mocap data
receiver.start()

# Main loop
while True:
    frame = receiver.get_latest_frame()
    if frame:
        mocap_data = retargeter.process_mocap_frame(frame["bones"])
        qpos = retargeter.retarget(mocap_data)
        # qpos[:3] = root position (x, y, z)
        # qpos[3:7] = root rotation (w, x, y, z quaternion)
        # qpos[7:] = joint angles

# Cleanup
receiver.stop()
```

### Receive Mocap Data Only

```python
from movin_sdk_python import MocapReceiver

# Initialize receiver
receiver = MocapReceiver(port=11235)
receiver.start()

# Main loop
while True:
    frame = receiver.get_latest_frame()
    if frame:
        print(f"Frame {frame['frame_idx']}: {len(frame['bones'])} bones")
        for bone in frame['bones']:
            print(f"  {bone['bone_name']}: pos={bone['p']}, rot={bone['q']}")

receiver.stop()
```

### BVH File Retargeting

```python
from movin_sdk_python import Retargeter

# Initialize
retargeter = Retargeter(robot_type="unitree_g1", human_height=1.75)

# Load BVH file
frames, height = retargeter.load_bvh("motion.bvh")

# Retarget each frame
for frame in frames:
    qpos = retargeter.retarget(frame)
    # Use qpos...
```

## Example Scripts

> **Note:** Install the package first before running examples: `pip install -e /path/to/movin_sdk_python`

### Receive and Print Mocap Data

```bash
python -m movin_sdk_python.examples.receive_mocap --port 11235
python -m movin_sdk_python.examples.receive_mocap --port 11235 --verbose
```

### Real-time Mocap to Robot

```bash
python -m movin_sdk_python.examples.mocap_to_robot --port 11235 --robot unitree_g1 --human_height 1.75
```

### BVH to Robot

```bash
python -m movin_sdk_python.examples.bvh_to_robot --bvh_file path/to/motion.bvh --human_height 1.75
```

## API Reference

### MocapReceiver Class

```python
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

### Retargeter Class

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
- `process_mocap_frame(bones)` - Process real-time mocap frame
- `retarget(human_data, offset_to_ground=False)` - Retarget to robot
- `get_required_bones()` - Get set of required bone names
- `set_ground_offset(offset)` - Set ground height offset

### Output Format

The `retarget()` method returns a numpy array `qpos`:

- `qpos[:3]` - Root position (x, y, z) in meters
- `qpos[3:7]` - Root orientation as quaternion (w, x, y, z)
- `qpos[7:]` - Joint angles in radians

## Package Structure

```
movin_sdk_python/
├── pyproject.toml           # Package configuration
├── __init__.py              # Main package exports
├── examples/                # Example scripts
│   ├── bvh_to_robot.py      # BVH file to robot retargeting
│   ├── mocap_to_robot.py    # Real-time mocap to robot
│   └── receive_mocap.py     # Simple mocap data receiver
├── mocap_receiver/          # MocapReceiver subpackage
│   ├── mocap_receiver.py    # UDP/OSC receiver with frame assembly
│   └── osc_reader.py        # OSC protocol parser
├── retargeter/              # Retargeter subpackage
│   ├── retargeter.py        # IK-based motion retargeting
│   ├── assets/              # Robot models and meshes
│   └── ik_configs/          # IK configuration files
└── utils/                   # Utility functions
    ├── bvh_loader.py        # BVH file parsing
    ├── fk_utils.py          # Forward kinematics utilities
    └── quat_utils.py        # Quaternion math utilities
```

## Supported Robots

| Robot Type | Description | DoFs |
|------------|-------------|------|
| `unitree_g1` | Unitree G1 (standard) | 29 |
| `unitree_g1_with_hands` | Unitree G1 with hands | 43 |

## Dependencies

- numpy>=1.21.0
- scipy>=1.7.0
- mujoco>=3.0.0
- mink>=0.1.0

## License

See the original GMR project for license information.
