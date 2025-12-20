# MOVIN SDK Python

A MOVIN Python SDK for receiving motion capture data from MOVIN and retargeting it to robots in real-time.

## Features

- **Real-time Mocap Receiver**: Receive mocap data via OSC protocol from MOVIN (Unity)
- **BVH File Support**: Load and retarget BVH motion capture files
- **Motion Retargeting**: Retarget human motion to robot joint positions using IK
- **MuJoCo Visualization**: Real-time visualization of retargeted motion in MuJoCo viewer
- **Unitree G1 Support**: Both standard G1 (29 DoF) and G1 with hands variants
- **Self-contained**: All assets (robot models, meshes, IK configs) included

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
- loop-rate-limiters>=0.1.0

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

## Package Structure

```
movin_sdk_python/
├── pyproject.toml                  # Package configuration
├── __init__.py                     # Main package exports
├── examples/                       # Example scripts
│   ├── bvh_to_robot.py             # BVH file to robot retargeting
│   ├── mocap_to_robot.py           # Real-time mocap to robot (console)
│   ├── mocap_to_robot_mujoco.py    # Real-time mocap with MuJoCo viewer
│   └── receive_mocap.py            # Simple mocap data receiver
├── mocap_receiver/                 # MocapReceiver subpackage
│   ├── mocap_receiver.py           # UDP/OSC receiver with frame assembly
│   └── osc_reader.py               # OSC protocol parser
├── retargeter/                     # Retargeter subpackage
│   ├── retargeter.py               # IK-based motion retargeting
│   ├── assets/                     # Robot models and meshes
│   └── ik_configs/                 # IK configuration files
├── viewer/                         # MujocoViewer subpackage
│   └── mujoco_viewer.py            # Real-time MuJoCo visualization
└── utils/                          # Utility functions
    ├── bvh_loader.py               # BVH file parsing
    ├── fk_utils.py                 # Forward kinematics utilities
    └── quat_utils.py               # Quaternion math utilities
```


## Quick Start

### Real-time Mocap to Robot with Visualization (mujoco)

```python
from movin_sdk_python import MocapReceiver, Retargeter, MujocoViewer

# Initialize
receiver = MocapReceiver(port=11235)
retargeter = Retargeter(robot_type="unitree_g1", human_height=1.75)
viewer = MujocoViewer(robot_type="unitree_g1", motion_fps=60)

# Start receiving mocap data
receiver.start()

# Main loop - runs until viewer window is closed
while viewer.is_running():
    frame = receiver.get_latest_frame()
    if frame:
        mocap_data = retargeter.process_mocap_frame(frame["bones"])
        qpos = retargeter.retarget(mocap_data)
        # qpos[:3] = root position (x, y, z)
        # qpos[3:7] = root rotation (w, x, y, z quaternion)
        # qpos[7:] = joint angles
        viewer.step(qpos)

# Cleanup
receiver.stop()
viewer.close()
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

## Examples

### Receive and Print Mocap Data

```bash
python3 -m movin_sdk_python.examples.receive_mocap --port 11235
python3 -m movin_sdk_python.examples.receive_mocap --port 11235 --verbose
```

### Real-time Mocap to Robot (Console Output)

```bash
python3 -m movin_sdk_python.examples.mocap_to_robot --port 11235 --robot unitree_g1 --human_height 1.75
```

### Real-time Mocap to Robot with MuJoCo Visualization

```bash
python3 -m movin_sdk_python.examples.mocap_to_robot_mujoco --port 11235 --robot unitree_g1 --human_height 1.75
python3 -m movin_sdk_python.examples.mocap_to_robot_mujoco --port 11235 --robot unitree_g1 --print_fps --no_rate_limit
```

### BVH to Robot

```bash
python3 -m movin_sdk_python.examples.bvh_to_robot --bvh_file path/to/motion.bvh --human_height 1.75
```

## API Reference

For detailed API documentation, see [API.md](doc/API.md).



## Acknowledgments

The URDF and STL mesh files for the Unitree G1 robot are sourced from [Unitree Robotics](https://github.com/unitreerobotics). Please refer to their repositories for the original robot models and licensing information:

- [unitree_ros](https://github.com/unitreerobotics/unitree_ros) - ROS packages with URDF files for Unitree robots

The motion retargeting approach in this SDK is based on [GMR: General Motion Retargeting](https://github.com/YanjieZe/GMR) :

- [GMR GitHub Repository](https://github.com/YanjieZe/GMR) - General Motion Retargeting for humanoid robots

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


