# MOVIN SDK Python

Receive and record real-time motion capture from MOVIN Studio.

MOVIN SDK Python is centered on one workflow: connect to MOVIN Studio over
OSC/UDP, receive assembled mocap frames, and preserve the original stream for
recording and replay. The default installation has no third-party runtime
dependencies.

Robot retargeting, numerical motion utilities, and MuJoCo visualization are
separate optional extensions. They are not installed or imported by the core
SDK unless explicitly requested.

## What Is Core and What Is Optional?

```text
Default installation

MOVIN Studio ──OSC/UDP──> MocapReceiver ──> mocap frames
                              │
                              └──> OscRecorder ──> recording ──> replay

Optional extensions

mocap frame ────────────────────────────────> MocapViewer (stick figure)
     │                                            [viewer]
     └──> Retargeter ──> robot state ───────> MujocoViewer
           [retargeting]                          [viewer]
```

| Area | Default install | Purpose |
|------|:---------------:|---------|
| `MovinSession`, `MocapReceiver` | Yes | Receive mocap frames from MOVIN Studio |
| `OscRecorder`, `OscPlayer`, `ReplayMocapReceiver` | Yes | Record and replay the original OSC stream |
| `Retargeter` and numeric motion utilities | No — `[retargeting]` | Convert human motion to robot joint states |
| `MocapViewer`, `MujocoViewer` | No — `[viewer]` | Visualize raw mocap or robot states in MuJoCo |

## Install the Core SDK

From GitHub:

```bash
pip install git+https://github.com/MOVIN3D/MOVIN-SDK-Python.git
```

From a local checkout:

```bash
pip install -e /path/to/MOVIN-SDK-Python
```

This installs only the receiver, recorder, and replay APIs. NumPy, SciPy,
MuJoCo, Mink, and viewer dependencies are not part of the default installation.

## Quick Start: Receive and Record

Enable OSC output in MOVIN Studio and configure it to send to the machine and
port used below. `MovinSession` is the main high-level API.

```python
import time

from movin_sdk_python import MovinSession

session = MovinSession(host="0.0.0.0", port=11235)
session.start_recording("session.pkl")

try:
    with session:
        while True:
            frame = session.get_latest_frame()
            if frame is None:
                time.sleep(0.001)
                continue

            print(
                f"frame={frame['frame_idx']} "
                f"actor={frame['actor']} "
                f"bones={len(frame['bones'])}"
            )
except KeyboardInterrupt:
    pass
```

The context manager stops the receiver and saves the active recording. Raw OSC
messages are recorded before any optional frame processing, so extension errors
cannot corrupt the source recording.

Run the included example:

```bash
python3 examples/receive_mocap.py --port 11235 --record session.pkl
```

The default bind address is `0.0.0.0:11235`. To listen only on another local
interface and port, pass `--host` (or its `--ip` alias) and `--port`:

```bash
python3 examples/receive_mocap.py \
    --host 192.168.0.25 \
    --port 12000 \
    --record session.pkl
```

Set MOVIN Studio's destination to `192.168.0.25:12000` in this example.
`0.0.0.0` is a wildcard bind address and must not be used as the Studio
destination.

## Replay a Recording

`ReplayMocapReceiver` follows the same polling API as `MocapReceiver`:

```python
import time

from movin_sdk_python import ReplayMocapReceiver

receiver = ReplayMocapReceiver(
    "session.pkl",
    realtime=True,
    loop=True,
)
receiver.start()

try:
    while True:
        frame = receiver.get_latest_frame()
        if frame is not None:
            print(frame["frame_idx"])
        time.sleep(0.001)
except KeyboardInterrupt:
    pass
finally:
    receiver.stop()
```

Use `OscPlayer` for message-level iteration or `peek_first_frame()` when only
the first assembled frame is needed.

## Low-Level Receiver API

Use `MocapReceiver` directly when session-level recording and extension
management are unnecessary:

```python
import time

from movin_sdk_python import MocapReceiver

receiver = MocapReceiver(host="0.0.0.0", port=11235)
receiver.start()

try:
    while True:
        frame = receiver.get_latest_frame()
        if frame is not None:
            print(frame["frame_idx"], frame["actor"])
        time.sleep(0.001)
finally:
    receiver.stop()
```

## Frame Format

The core receiver returns one dictionary per complete MOVIN frame:

```python
{
    "timestamp": str,
    "actor": str,
    "frame_idx": int,
    "bones": [
        {
            "bone_index": int,
            "parent_index": int,
            "bone_name": str,
            "p": (px, py, pz),       # local position
            "rq": (w, x, y, z),      # rest-pose quaternion
            "q": (w, x, y, z),       # local rotation quaternion
            "s": (sx, sy, sz),       # scale
        },
        ...
    ],
}
```

Quaternions exposed by the SDK use `(w, x, y, z)` order. MOVIN Studio values
arrive in `(x, y, z, w)` order and are converted during frame assembly.

## Core API

| API | Role |
|-----|------|
| `MovinSession` | High-level receive, record, and extension lifecycle |
| `MocapReceiver` | Background OSC/UDP receiver and frame assembler |
| `MovinFrameAssembler` | Assemble chunked `/MOVIN/Frame` messages |
| `OscReader` | Parse one OSC packet |
| `OscRecorder` | Save parsed OSC messages to a recording |
| `OscPlayer` | Iterate through recorded OSC messages |
| `ReplayMocapReceiver` | Replay a recording through the receiver-style API |
| `peek_first_frame` | Read the first assembled frame from a recording |

See [Core API Reference](doc/API.md) for constructors, lifecycle behavior, and
extension interfaces.

## Optional Extensions

Optional features must be installed explicitly:

```bash
pip install -e ".[retargeting]"  # Retargeter plus numeric motion utilities
pip install -e ".[viewer]"       # Raw stick-figure and robot-state viewers
pip install -e ".[all]"          # Both optional feature sets
```

For a Git installation with every extension:

```bash
pip install "movin_sdk_python[all] @ git+https://github.com/MOVIN3D/MOVIN-SDK-Python.git"
```

Raw mocap can be viewed directly without Retargeter:

```python
from movin_sdk_python import MocapViewer, MovinSession

viewer = MocapViewer()
with MovinSession(host="0.0.0.0", port=11235, sinks=[viewer]) as session:
    while viewer.is_running():
        session.get_latest_frame()
```

Existing robot-viewer imports also remain supported after installing the
relevant extra:

```python
from movin_sdk_python import MujocoViewer, Retargeter
```

`Retargeter` defaults to the MOVINManV3 skeleton. Legacy streams remain
available with `Retargeter(source_preset="movinman")` or the live example
option `--source-preset movinman`.

Without the required extra, using an optional API raises
`MissingOptionalDependencyError` with the appropriate installation command. A
missing optional dependency never prevents `import movin_sdk_python` or any core
receive/record/replay workflow.

See [Optional Extensions](doc/OPTIONAL_EXTENSIONS.md) for retargeting, viewer,
skeleton preset, and session pipeline documentation.

## Examples

Core example, available with the default installation:

```bash
python3 examples/receive_mocap.py --port 11235 --record session.pkl
```

The following examples require optional extras:

```bash
# Requires [viewer], but not Retargeter
python3 examples/view_mocap.py --port 11235

# Bind to one local interface and a custom port
python3 examples/view_mocap.py --host 192.168.0.25 --port 12000

# Requires [retargeting]
python3 examples/mocap_to_robot.py --port 11235 --robot unitree_g1

# Requires [all]
python3 examples/mocap_to_robot_mujoco.py --port 11235 --robot unitree_g1
```

On macOS, MuJoCo GUI examples must be run with `mjpython`.

## Package Layout

```text
movin_sdk_python/
├── session.py           # Core: high-level receive and recording lifecycle
├── mocap_receiver/      # Core: OSC parsing and frame assembly
├── recording/           # Core: recording, playback, and replay receiver
├── pipeline.py          # Core: optional-extension interfaces
├── retargeter/          # Optional: robot retargeting implementation/assets
├── viewer/              # Optional: raw mocap and robot-state MuJoCo viewers
└── utils/               # Optional numeric utilities plus dependency-free presets
```

The dependency direction is one-way: optional modules may import core modules;
core modules do not import Retargeter, Viewer, NumPy, SciPy, MuJoCo, or Mink.

## Development

Run the dependency-free test suite with:

```bash
python3 -m unittest discover -s tests -v
```

## Acknowledgments

- Unitree G1 URDF/STL assets: [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
- Motion retargeting approach: [GMR: General Motion Retargeting](https://github.com/YanjieZe/GMR)

## License

Apache License 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE).
