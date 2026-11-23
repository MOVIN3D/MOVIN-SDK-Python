# MOVIN SDK Python

Receive and record real-time motion capture from MOVIN Studio.

MOVIN SDK Python is centered on one workflow: connect to MOVIN Studio over
OSC/UDP, receive assembled mocap frames, and preserve the original stream for
recording and replay. The default installation has no third-party runtime
dependencies.

Robot retargeting, numerical motion utilities, and MuJoCo visualization are
separate optional extensions. They are not installed or imported by the core
SDK unless explicitly requested.

MOVIN Studio calibrates every actor, and the stream carries that actor's bone
lengths. Robot retargeting and the character mesh follow the streamed actor's
body by default, and the MJCF skeleton is fitted with one call. Each can be
switched back to the fixed reference body (see
[Matching the Actor's Body](#matching-the-actors-body)).

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
| Skeleton presets, packaged MJCF/mesh assets, `fit_skeleton_mjcf` | Yes | MOVINMan skeleton assets and fitting them to the actor's body |
| `Retargeter`, `MOVINMeshModel`, numeric motion utilities | No — `[retargeting]` | Convert human motion to robot joint states; pose the character mesh |
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

`0.0.0.0` is the receiver's wildcard *bind* address, not a destination. When
MOVIN Studio runs on the same computer as the receiver, enter `127.0.0.1` as
the destination (on Linux, Studio's default `0.0.0.0` also reaches the local
receiver, because the kernel delivers packets sent to `0.0.0.0` to the local
host). When Studio runs on another computer, enter the receiver computer's
real IP address; `0.0.0.0` is not routable and never reaches another machine.

The receiver answers MOVIN Studio's status probe. While it runs, Studio's
Streaming panel shows **OSC connected** and the Motion and Viewer frame rates.
Viewer fps is the rate at which your application calls `get_latest_frame()`.
To turn the reply off, create the receiver with
`MocapReceiver(reply_status=False)`. For a session, pass that receiver as
`MovinSession(receiver=...)`. See
[Connection status in MOVIN Studio](doc/API.md#connection-status-in-movin-studio).

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
            "p": (px, py, pz),       # local position (offset in the parent's frame)
            "rq": (w, x, y, z),      # rest-pose quaternion (informational; "q" already includes it)
            "q": (w, x, y, z),       # full local rotation quaternion
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
option `--source-preset movinman`. It also sizes the robot targets from the
streamed body by default (`human_height="auto"`); see
[Matching the Actor's Body](#matching-the-actors-body).

Without the required extra, using an optional API raises
`MissingOptionalDependencyError` with the appropriate installation command. A
missing optional dependency never prevents `import movin_sdk_python` or any core
receive/record/replay workflow.

See [Optional Extensions](doc/OPTIONAL_EXTENSIONS.md) for retargeting, viewer,
skeleton preset, and session pipeline documentation.

## Matching the Actor's Body

The MOVINMan MJCF skeletons and LBS meshes of the skeleton presets ship inside
the package (`movin_sdk_python/assets`; `preset.mjcf_path`,
`preset.mesh_npz_path`) with the reference body's proportions.

| Component | Default | Turn off |
|-----------|---------|----------|
| `MocapViewer` stick figure | Always the streamed body | — |
| `Retargeter` | `human_height="auto"` sizes the targets from the streamed body (`body_scale`) | `human_height=1.75` (any number: fixed legacy scale), or `--human_height 1.75` in the examples |
| `MOVINMeshModel.pose_from_bones()` | `fit=True` fits the mesh to the streamed body | `fit=False` or `reset_fit()` |
| MJCF skeleton | Opt-in, because the scene is built by your application: call `fit_skeleton_mjcf()` | Load `preset.mjcf_path` unchanged |

```python
from movin_sdk_python import Retargeter
from movin_sdk_python.utils.movinman_mesh_utils import MOVINMeshModel
from movin_sdk_python.utils.skeleton_fitting import fit_skeleton_mjcf

bones = frame["bones"]  # any frame from MocapReceiver / ReplayMocapReceiver

retargeter = Retargeter()  # human_height="auto"
qpos = retargeter.retarget(retargeter.process_mocap_frame(bones))
print(retargeter.body_scale)  # > 1 when the actor is smaller than the reference body

mesh = MOVINMeshModel.from_preset("movinman_v3")
vertices = mesh.pose_from_bones(bones)  # fitted; fit=False for the reference body

with open("actor.xml", "w") as f:  # MJCF for MuJoCo / Isaac Lab
    f.write(fit_skeleton_mjcf(bones, "movinman_v3"))
```

The fitted mesh and MJCF reproduce the stream's joint positions exactly and
keep the feet on the ground.

Numeric `human_height` values keep the scale formula of earlier releases
(`scale_table × human_height / 1.8`), which grows with the value. It is
therefore not the actor's real height. See
[Body size](doc/OPTIONAL_EXTENSIONS.md#body-size-human_height) and
[Fitting Assets](doc/OPTIONAL_EXTENSIONS.md#fitting-assets-to-the-actors-body-proportions).

## Examples

Core example, available with the default installation:

```bash
python3 examples/receive_mocap.py --port 11235 --record session.pkl
```

The following examples require optional extras:

```bash
# Requires [viewer], but not Retargeter
python3 examples/view_mocap.py --port 11235
python3 examples/view_mocap.py --port 11235 --record session.pkl

# Bind to one local interface and a custom port
python3 examples/view_mocap.py --host 192.168.0.25 --port 12000

# Requires [retargeting]; prints the body scale once, then FPS
python3 examples/mocap_to_robot.py --port 11235 --robot unitree_g1 --print_fps
python3 examples/mocap_to_robot.py --port 11235 --human_height 1.75   # fixed legacy scale

# Requires [all]
python3 examples/mocap_to_robot_mujoco.py --port 11235 --robot unitree_g1_with_hands
```

Common options of the robot examples are `--robot unitree_g1|unitree_g1_with_hands`,
`--human_height auto|<meters>`, `--source-preset movinman_v3|movinman`,
`--print_fps`, and `--debug`. `mocap_to_robot_mujoco.py` also has
`--motion_fps` and `--no_rate_limit`.

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
├── assets/              # MOVINMan MJCF skeletons and LBS meshes for the presets
└── utils/               # Optional numeric utilities plus dependency-free presets
```

The dependency direction is one-way: optional modules may import core modules;
core modules do not import Retargeter, Viewer, NumPy, SciPy, MuJoCo, or Mink.

## Development

Run the test suite with:

```bash
python3 -m unittest discover -s tests -v
```

Without the optional extras installed, the tests that need NumPy, MuJoCo, or
Mink are skipped. The rest run with no third-party dependencies.

## Acknowledgments

- Unitree G1 URDF/STL assets: [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
- Motion retargeting approach: [GMR: General Motion Retargeting](https://github.com/YanjieZe/GMR)

## License

Apache License 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE).
