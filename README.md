# MOVIN SDK Python

A Python SDK for receiving real-time motion capture data from MOVIN Studio.

MOVIN Studio streams motion capture data over UDP using the OSC protocol. This SDK provides `MocapReceiver` as its core component for receiving and parsing that data. Robot retargeting (`Retargeter`), MuJoCo visualization (`MujocoViewer`), and Isaac Lab coordinate utilities are included.

## Features

- **Mocap reception**: receive and assemble `/MOVIN/Frame` OSC packets over UDP
- **Recording & replay**: record live OSC streams to file and replay offline without MOVIN Studio
- **Motion retargeting**: retarget human motion to Unitree G1 robots via IK, supporting both the legacy MOVINMan and MOVINManV3 skeleton presets
- **Isaac Lab utilities**: coordinate conversion (Unity LH Y-up to Isaac RH Z-up) and MOVINMan mesh overlay via LBS
- **MuJoCo viewer**: lightweight real-time viewer for retargeted robot motion

## Supported Robots

| Robot Type | Description | DoFs |
|------------|-------------|------|
| `unitree_g1` | Unitree G1 (standard) | 29 |
| `unitree_g1_with_hands` | Unitree G1 with hands | 43 |

## Installation

```bash
pip install git+https://github.com/MOVIN3D/MOVIN-SDK-Python.git
```

Or from a local checkout:

```bash
pip install -e /path/to/MOVIN-SDK-Python
```

### Dependencies

The core `MocapReceiver` only uses the Python standard library (`socket`, `threading`, `struct`).

Retargeting, visualization, and Isaac Lab utilities require:

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
    ├── mocap_receiver/              # Core: OSC/UDP reception
    │   ├── mocap_receiver.py        # MocapReceiver class
    │   ├── movin_frame_assembler.py # Chunked frame assembly logic
    │   └── osc_reader.py            # OSC protocol parser
    ├── recording/                   # Recording & replay
    │   ├── osc_recorder.py          # OscRecorder — record OSC messages
    │   ├── osc_player.py            # OscPlayer — play back recordings
    │   ├── replay_receiver.py       # ReplayMocapReceiver — drop-in offline replacement
    │   └── peek.py                  # peek_first_frame — read a recording's first frame
    ├── retargeter/                  # Robot retargeting
    │   ├── retargeter.py
    │   ├── assets/                  # Robot models and meshes
    │   └── ik_configs/              # IK configuration files
    ├── viewer/                      # MuJoCo visualization
    │   └── mujoco_viewer.py
    └── utils/
        ├── bvh_loader.py            # BVH file parsing
        ├── fk_utils.py              # Forward kinematics, coordinate conversion
        ├── quat_utils.py            # Quaternion math (wxyz format)
        ├── skeleton_presets.py      # MOVINMan / MOVINManV3 skeleton preset definitions
        ├── isaac_lab_utils.py       # MOVIN-to-Isaac Lab coordinate helpers
        └── movinman_mesh_utils.py   # MOVINMan mesh model and LBS skinning
```

## Quick Start

### Receiving Motion Capture Data

Enable OSC output in MOVIN Studio, configure the IP and port, then:

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

### Recording & Replay

```python
from movin_sdk_python.recording import OscRecorder, ReplayMocapReceiver

# Record a live session
recorder = OscRecorder("session.pkl")
receiver = MocapReceiver(port=11235, recorder=recorder)
receiver.start()
# ... run your pipeline ...
receiver.stop()
recorder.save()

# Replay offline (same API as MocapReceiver)
replay = ReplayMocapReceiver("session.pkl", realtime=True, loop=True)
replay.start()
frame = replay.get_latest_frame()
replay.stop()
```

## Frame Data Format

`get_latest_frame()` returns a dictionary:

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

> Quaternions are in `(w, x, y, z)` order. The SDK converts from `(x, y, z, w)` used by MOVIN Studio (Unity).

## API Reference

### MocapReceiver

```python
receiver = MocapReceiver(port=11235, recorder=None)
```

| Method | Description |
|--------|-------------|
| `start()` | Start the background receiver thread |
| `stop()` | Stop receiving and clean up resources |
| `get_latest_frame()` | Return the most recent complete frame, or `None` |
| `get_receive_rate()` | Get the current packet receive rate in Hz |
| `reset()` | Reset all internal buffers |

Pass an `OscRecorder` instance as `recorder` to record incoming OSC messages for later replay.

### OscReader

Parse raw OSC binary packets from MOVIN Studio directly:

```python
from movin_sdk_python import OscReader

reader = OscReader(raw_bytes)
address, args = reader.read_message()
```

### Recording & Replay

| Class | Description |
|-------|-------------|
| `OscRecorder(path)` | Record OSC messages to a pickle file |
| `OscPlayer(path)` | Load and iterate over recorded messages |
| `ReplayMocapReceiver(path)` | Drop-in replacement for `MocapReceiver` using recorded data |

`peek_first_frame(recording_path)` reads the first assembled frame from a recording (same shape as `get_latest_frame()`, or `None`) without starting a full `ReplayMocapReceiver` — handy for detecting the skeleton preset before replaying.

## Example Scripts

```bash
# Receive and print mocap data
python examples/receive_mocap.py --port 11235

# Retarget live mocap to robot (console)
python examples/mocap_to_robot.py --port 11235 --robot unitree_g1 --human_height 1.75

# Retarget live mocap + MuJoCo viewer
python examples/mocap_to_robot_mujoco.py --port 11235 --robot unitree_g1 --human_height 1.75

# macOS: use mjpython instead of python
mjpython examples/mocap_to_robot_mujoco.py --port 11235 --robot unitree_g1 --human_height 1.75

# BVH file retargeting (auto-detects the source skeleton preset from the BVH)
python examples/bvh_to_robot.py --bvh_file path/to/motion.bvh --human_height 1.75
```

## Retargeting

For detailed documentation see [API.md](doc/API.md).

`Retargeter(..., source_preset="movinman")` selects the source skeleton layout — `"movinman"` (legacy) or `"movinman_v3"` (MOVINManV3) — which determines the IK config loaded and the bones `get_required_bones()` expects. Use `detect_preset_from_bone_names()` from `movin_sdk_python.utils.skeleton_presets` to auto-detect the preset from a BVH file's or recording's bone names, as `examples/bvh_to_robot.py` does.

The `retarget()` method returns a numpy array `qpos`:

- `qpos[:3]` -- Root position (x, y, z) in meters
- `qpos[3:7]` -- Root orientation quaternion (w, x, y, z)
- `qpos[7:]` -- Joint angles in radians

## Acknowledgments

- Unitree G1 URDF/STL assets: [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
- Motion retargeting approach: [GMR: General Motion Retargeting](https://github.com/YanjieZe/GMR)

## License

MIT License -- see the [LICENSE](LICENSE) file for details.
