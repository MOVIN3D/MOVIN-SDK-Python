# Optional Extensions

> This document is not required for receiving, recording, or replaying MOVIN
> Studio mocap. Those workflows belong to the default SDK and are documented in
> the [Core API Reference](API.md).

Retargeting, numeric motion processing, and MuJoCo visualization are application
extensions built on top of the core mocap frame API. They are excluded from the
default dependency set and are imported only when used.

## Installation

Choose the smallest feature set needed by the application:

| Extra | Provides | Main dependencies |
|-------|----------|-------------------|
| `[retargeting]` | `Retargeter`, FK/quaternion utilities, Isaac Lab conversion helpers | NumPy, SciPy, MuJoCo, Mink |
| `[viewer]` | Raw `MocapViewer` and robot-state `MujocoViewer` | NumPy, MuJoCo, loop-rate-limiters |
| `[all]` | Both extension sets | Union of both dependency sets |

From a local checkout:

```bash
pip install -e ".[retargeting]"
pip install -e ".[viewer]"
pip install -e ".[all]"
```

From GitHub with every extension:

```bash
pip install "movin_sdk_python[all] @ git+https://github.com/MOVIN3D/MOVIN-SDK-Python.git"
```

The established top-level imports remain available after the matching extra is
installed:

```python
from movin_sdk_python import (
    MocapViewer,
    MujocoViewer,
    Retargeter,
    process_mocap_frame,
)
```

Without the required extra these symbols raise
`MissingOptionalDependencyError`. Core imports and active recording remain
unaffected.

## Raw Mocap Stick-Figure Viewer

Requires `[viewer]` only. Retargeter, Mink, and SciPy are not required.

`MocapViewer` accepts the frame dictionary returned by `MocapReceiver` and
renders its bone hierarchy as joint spheres connected by capsules:

```python
MocapViewer(
    motion_fps=60,
    camera_distance=3.0,
    camera_elevation=-10.0,
    joint_radius=0.025,
    bone_radius=0.012,
    joint_color=(1.0, 0.55, 0.1, 1.0),
    bone_color=(0.15, 0.65, 1.0, 1.0),
    show_left_ui=False,
    show_right_ui=False,
    keyboard_callback=None,
)
```

| Member | Behavior |
|--------|----------|
| `step(frame, rate_limit=True, follow_camera=True)` | Displays one raw MOVIN frame |
| `on_frame(frame)` | Displays a raw frame when attached as a `FrameSink` |
| `is_running()` | Returns whether the viewer window is open |
| `close()` | Closes the viewer |

The viewer uses the incoming `bone_index` and `parent_index` hierarchy, so it
does not require a fixed skeleton preset. Local transforms are converted to
global, right-handed Z-up coordinates before rendering.

Its sky, lighting, checker-grid floor, and default camera elevation match the
robot-state `MujocoViewer`; only the displayed subject differs.

```python
import time

from movin_sdk_python import MocapViewer, MovinSession

viewer = MocapViewer(motion_fps=60)

try:
    with MovinSession(host="0.0.0.0", port=11235, sinks=[viewer]) as session:
        while viewer.is_running():
            if session.get_latest_frame() is None:
                time.sleep(0.001)
finally:
    viewer.close()
```

Equivalent runnable example:

```bash
python3 examples/view_mocap.py --port 11235

# Or bind to a specific local interface and port
python3 examples/view_mocap.py --host 192.168.0.25 --port 12000
```

## Retargeting Extension

Requires `[retargeting]`.

`movinman_v3` is the default source preset. Pass `source_preset="movinman"`
only when receiving the legacy MOVINMan skeleton.

```python
Retargeter(
    robot_type="unitree_g1",
    human_height=1.75,
    solver="daqp",
    damping=0.5,
    verbose=False,
    use_velocity_limit=True,
    source_preset="movinman_v3",
)
```

### Supported robot models

| `robot_type` | Description | DoFs |
|--------------|-------------|------|
| `unitree_g1` | Unitree G1 | 29 |
| `unitree_g1_with_hands` | Unitree G1 with articulated hands | 43 |

### Methods

| Member | Behavior |
|--------|----------|
| `process_mocap_frame(bones)` | Converts receiver bones to the retargeter's human-motion representation |
| `retarget(human_data, offset_to_ground=False)` | Solves IK and returns robot `qpos` |
| `process(frame)` | Processes and retargets one complete receiver frame for `MovinSession` |
| `get_required_bones()` | Returns bones required by the active source preset |
| `set_ground_offset(offset)` | Sets the ground-height offset |

`process(frame)` is an additive adapter for the core `FrameProcessor` contract.
The existing `process_mocap_frame()` and `retarget()` workflow remains supported.

### Retargeting output

`retarget()` and `process()` return a NumPy `qpos` array:

- `qpos[:3]`: root position `(x, y, z)` in meters
- `qpos[3:7]`: root quaternion `(w, x, y, z)`
- `qpos[7:]`: joint angles in radians

### Live retargeting

```python
import time

from movin_sdk_python import MocapReceiver, Retargeter

receiver = MocapReceiver(host="0.0.0.0", port=11235)
retargeter = Retargeter(
    robot_type="unitree_g1",
    human_height=1.75,
    source_preset="movinman_v3",
)

receiver.start()
try:
    while True:
        frame = receiver.get_latest_frame()
        if frame is None:
            time.sleep(0.001)
            continue

        human_data = retargeter.process_mocap_frame(frame["bones"])
        qpos = retargeter.retarget(human_data)
        print(qpos)
finally:
    receiver.stop()
```

## Robot-State Viewer Extension

`MujocoViewer` requires `[viewer]` and accepts Unitree robot `qpos`, not a raw
mocap frame. Install `[all]` when it will display built-in Retargeter output.

```python
MujocoViewer(
    robot_type="unitree_g1",
    motion_fps=60,
    camera_distance=None,
    camera_elevation=-10.0,
    show_left_ui=False,
    show_right_ui=False,
    keyboard_callback=None,
)
```

### Methods

| Member | Behavior |
|--------|----------|
| `step(qpos, rate_limit=True, follow_camera=True)` | Displays one robot state and returns whether the window remains open |
| `step_decomposed(root_pos, root_rot, dof_pos, ...)` | Displays decomposed robot state arrays |
| `on_frame(qpos)` | Displays session processor output as a `FrameSink` |
| `is_running()` | Returns whether the viewer window is open |
| `close()` | Closes the viewer |

On macOS the GUI must be launched with `mjpython` rather than `python3`.

## Composing Extensions with `MovinSession`

Retargeting and viewing are never activated automatically. Create them and pass
them to the session explicitly:

```python
import time

from movin_sdk_python import MovinSession, MujocoViewer, Retargeter

retargeter = Retargeter(
    robot_type="unitree_g1",
    source_preset="movinman_v3",
)
viewer = MujocoViewer(robot_type="unitree_g1", motion_fps=60)

session = MovinSession(
    host="0.0.0.0",
    port=11235,
    processors=[retargeter],
    sinks=[viewer],
)
session.start_recording("session.pkl")

try:
    with session:
        while viewer.is_running():
            if session.get_latest_frame() is None:
                time.sleep(0.001)
finally:
    viewer.close()
```

The data paths remain separate:

```text
raw OSC ───────────────> OscRecorder
   │
   └──> mocap frame ──> Retargeter.process ──> MujocoViewer.on_frame
```

The recorder always receives the source OSC messages. A slow or failing
extension does not stop the receiver thread or recorder. Processor errors skip
downstream sinks for that frame; sink errors are isolated from other sinks.

## Motion Utilities

Requires `[retargeting]` unless a referenced module is explicitly documented as
dependency-free.

### Numeric utilities

The following established imports remain available from
`movin_sdk_python.utils`:

- `process_mocap_frame`
- `compute_forward_kinematics`
- `add_foot_mod_bones`
- `quat_mul`
- `rotate_vec_by_quat`

They load numeric dependencies lazily. Importing `movin_sdk_python` or the
`movin_sdk_python.utils` package alone does not load NumPy or SciPy.

## Skeleton Presets

Skeleton preset metadata is dependency-free even though it is primarily useful
to retargeting applications:

```python
from movin_sdk_python.utils.skeleton_presets import (
    detect_preset_from_bone_names,
    get_preset,
)

preset = get_preset("movinman_v3")
detected = detect_preset_from_bone_names(bone_names)
```

| Preset | Body count | Notes |
|--------|-----------:|-------|
| `movinman` | 51 | Legacy MOVINMan layout; includes a mesh-overlay asset |
| `movinman_v3` | 54 | Adds `Spine2`, `Spine3`, `Neck1`, and full finger chains |

`source_preset` chooses the IK mapping used by `Retargeter`. The default V3
mapping targets `Spine3` for the G1 torso rather than the legacy `Spine1`
mapping.

## Isaac Lab Helpers

Isaac Lab conversion and MOVINMan mesh helpers are optional application
utilities under `movin_sdk_python.utils` and require at least the numeric
dependencies installed by `[retargeting]`.

Key configurable functions include:

- `extract_movin_local_quats_yup(..., skeleton_body_names=...)`
- `process_movin_bones_for_isaaclab(..., skeleton_bone_names=...)`
- `build_dof_reorder_map(..., skeleton_bone_names=...)`
- `MOVINMeshModel(..., expected_bone_names=...)`
- `hips_global_transform_rh(bones)` / `find_stream_root_bone(bones)`

The Isaac Lab and mesh helpers locate the actor's global pose by walking the
streamed `parent_index` chain from `Hips` up to the stream root. The root
bone's name is not inspected, so `Root` (legacy MOVINMan), `RootBone`
(MOVINManV3), and other exporter names all work.

Use full body lists, including `Hips`, for `skeleton_body_names` and
`expected_bone_names`. Use non-root joint lists for `skeleton_bone_names`.

## Optional Examples

```bash
# [viewer] only: raw mocap stick figure
python3 examples/view_mocap.py --port 11235

# All live receiver examples accept --host/--ip and --port
python3 examples/view_mocap.py --host 192.168.0.25 --port 12000

# [retargeting]
python3 examples/mocap_to_robot.py --port 11235 --robot unitree_g1

# [all]
python3 examples/mocap_to_robot_mujoco.py --port 11235 --robot unitree_g1
```
