# MOVIN SDK Python

A Python SDK for receiving real-time motion capture data from MOVIN Studio.

MOVIN Studio streams motion capture data over UDP using the OSC protocol. This SDK provides a `MocapReceiver` class as its core component for receiving and parsing that data. Robot retargeting (`Retargeter`) and MuJoCo visualization (`MujocoViewer`) are included as usage examples.

## Core Features

- **MocapReceiver**: Receive real-time motion capture data from MOVIN Studio via OSC over UDP
- **OscReader**: Lightweight OSC binary packet parser

## Included Examples

- **Retargeter**: Retarget received motion data to robot joint angles using IK
- **MujocoViewer**: Real-time visualization with MuJoCo
- **BVH File Support**: Load and retarget BVH motion capture files

## Installation

### Install from GitHub (Recommended)

```bash
pip install git+https://github.com/MOVIN3D/MOVIN-SDK-Python.git
```

### Install from Local Path

```bash
pip install /path/to/MOVIN-SDK-Python

# Editable mode for development
pip install -e /path/to/MOVIN-SDK-Python
```

### Dependencies

The core `MocapReceiver` only uses the Python standard library (`socket`, `threading`, `struct`).

The robot retargeting and MuJoCo visualization examples require additional packages:

```
numpy>=1.21.0
scipy>=1.7.0
mujoco>=3.0.0
mink>=0.1.0
loop-rate-limiters>=0.1.0
```

## Package Structure

```
MOVIN-SDK-Python/
├── pyproject.toml
├── examples/
│   ├── receive_mocap.py            # Receive and print mocap data
│   ├── mocap_to_robot.py           # Real-time mocap → robot retargeting (console)
│   ├── mocap_to_robot_mujoco.py    # Real-time mocap → robot retargeting + MuJoCo viewer
│   └── bvh_to_robot.py             # BVH file → robot retargeting
└── movin_sdk_python/
    ├── __init__.py
    ├── mocap_receiver/             # Core: OSC/UDP reception
    │   ├── mocap_receiver.py       # MocapReceiver class
    │   └── osc_reader.py           # OSC protocol parser
    ├── retargeter/                 # Example: robot retargeting
    │   ├── retargeter.py
    │   ├── assets/                 # Robot models and meshes
    │   └── ik_configs/             # IK configuration files
    ├── viewer/                     # Example: MuJoCo visualization
    │   └── mujoco_viewer.py
    └── utils/                      # Utility functions
        ├── bvh_loader.py
        ├── fk_utils.py
        └── quat_utils.py
```

## Quick Start: Receiving Motion Capture Data

Enable OSC output in MOVIN Studio, configure the IP and port, then receive data with the following code.

```python
from movin_sdk_python import MocapReceiver
import time

receiver = MocapReceiver(port=11235)
receiver.start()

try:
    while True:
        frame = receiver.get_latest_frame()
        if frame:
            print(f"Frame {frame['frame_idx']}: actor={frame['actor']}, bones={len(frame['bones'])}")
            for bone in frame['bones']:
                print(f"  {bone['bone_name']}: pos={bone['p']}, rot={bone['q']}")
        time.sleep(0.001)
except KeyboardInterrupt:
    receiver.stop()
```

## Frame Data Format

`get_latest_frame()` returns a dictionary with the following structure:

```python
{
    "timestamp": str,        # Timestamp from the mocap system
    "actor": str,            # Actor name
    "frame_idx": int,        # Frame index
    "bones": [
        {
            "bone_index": int,
            "parent_index": int,
            "bone_name": str,
            "p": (px, py, pz),       # Local position
            "rq": (w, x, y, z),      # Rest pose quaternion
            "q": (w, x, y, z),       # Local rotation quaternion
            "s": (sx, sy, sz),       # Scale
        },
        ...
    ]
}
```

> Quaternions are provided in `(w, x, y, z)` order. The SDK internally converts from the `(x, y, z, w)` order used by MOVIN Studio (Unity).

## API Reference

### MocapReceiver

```python
receiver = MocapReceiver(port=11235)
```

| Method | Description |
|--------|-------------|
| `start()` | Start the background receiver thread |
| `stop()` | Stop receiving and clean up resources |
| `get_latest_frame()` | Return the most recent complete frame, or `None` |
| `get_receive_rate()` | Get the current packet receive rate in Hz |
| `reset()` | Reset all internal buffers |

### OscReader

Use this to parse raw OSC binary packets sent by MOVIN Studio directly.

```python
from movin_sdk_python import OscReader

reader = OscReader(raw_bytes)
address, args = reader.read_message()
# address == "/MOVIN/Frame"
```

## Example Scripts

### Receive and Print Mocap Data

```bash
python examples/receive_mocap.py --port 11235
python examples/receive_mocap.py --port 11235 --verbose
```

### Real-time Mocap → Robot Retargeting (Console)

```bash
python examples/mocap_to_robot.py --port 11235 --robot unitree_g1 --human_height 1.75
```

### Real-time Mocap → Robot Retargeting + MuJoCo Viewer

> **macOS**: The MuJoCo viewer must be launched with `mjpython` (included with the mujoco package).

```bash
python examples/mocap_to_robot_mujoco.py --port 11235 --robot unitree_g1 --human_height 1.75

# macOS
mjpython examples/mocap_to_robot_mujoco.py --port 11235 --robot unitree_g1 --human_height 1.75
```

### BVH File → Robot Retargeting

```bash
python examples/bvh_to_robot.py --bvh_file path/to/motion.bvh --human_height 1.75
```

## Retargeting Example API

For detailed documentation on the robot retargeting features, see [API.md](doc/API.md).

### Supported Robots

| Robot Type | Description | DoFs |
|------------|-------------|------|
| `unitree_g1` | Unitree G1 (standard) | 29 |
| `unitree_g1_with_hands` | Unitree G1 with hands | 43 |

### Retargeting Output Format

The `retarget()` method returns a numpy array `qpos`:

- `qpos[:3]` — Root position (x, y, z) in meters
- `qpos[3:7]` — Root orientation quaternion (w, x, y, z)
- `qpos[7:]` — Joint angles in radians

## Acknowledgments

The URDF and STL mesh files for the Unitree G1 robot are sourced from [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros).

The motion retargeting approach is based on [GMR: General Motion Retargeting](https://github.com/YanjieZe/GMR).

## License

MIT License — see the [LICENSE](LICENSE) file for details.
