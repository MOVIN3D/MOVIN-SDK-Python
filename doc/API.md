# Core API Reference

This document covers the APIs included in the default MOVIN SDK Python
installation: mocap reception, recording, playback, and replay.

No Retargeter, Viewer, MuJoCo, Mink, NumPy, or SciPy installation is required
for anything in this document. See [Optional Extensions](OPTIONAL_EXTENSIONS.md)
for robot retargeting and visualization.

## Public Core Imports

```python
from movin_sdk_python import (
    FrameProcessor,
    FrameSink,
    MissingOptionalDependencyError,
    MocapReceiver,
    MovinFrameAssembler,
    MovinSession,
    OscPlayer,
    OscReader,
    OscRecorder,
    ReplayMocapReceiver,
    peek_first_frame,
)
```

## `MovinSession`

`MovinSession` is the primary high-level API. It owns the receiver lifecycle,
can attach/detach a raw OSC recorder, and provides explicit extension hooks.

```python
MovinSession(
    port=11235,
    *,
    host="0.0.0.0",
    receiver=None,
    processors=None,
    sinks=None,
    on_extension_error=None,
)
```

### Parameters

| Parameter | Description |
|-----------|-------------|
| `host` | Local IPv4 address or hostname used when the session creates its default receiver; `0.0.0.0` listens on every local IPv4 interface |
| `port` | UDP port used when the session creates its default `MocapReceiver` |
| `receiver` | Optional receiver-compatible object; defaults to `MocapReceiver(host=host, port=port)` |
| `processors` | Ordered iterable of objects implementing `process(frame)` |
| `sinks` | Iterable of objects implementing `on_frame(frame)` |
| `on_extension_error` | Optional callback receiving `(extension, exception)` |

### Lifecycle and recording

| Member | Behavior |
|--------|----------|
| `start()` | Starts the receiver once and returns the session |
| `stop()` | Stops the receiver and saves an active recording |
| `is_running` | `True` after `start()` and before `stop()` |
| `start_recording(path, stream_type="auto")` | Attaches a new `OscRecorder` to the original OSC stream |
| `stop_recording(save=True)` | Detaches and optionally saves the recorder; returns it or `None` |
| `is_recording` | `True` while a recorder is attached |

Attach recording before starting reception when the complete stream, including
the first packet, must be retained:

```python
from movin_sdk_python import MovinSession

session = MovinSession(host="0.0.0.0", port=11235)
session.start_recording("session.pkl")

with session:
    frame = session.get_latest_frame()
```

The context manager calls `start()` on entry and `stop()` on exit. An active
recording is saved even when receiver cleanup raises an exception.

### Frame access and extension dispatch

| Member | Behavior |
|--------|----------|
| `get_latest_frame()` | Polls the receiver, applies processors, notifies sinks, and returns the final value |
| `get_latest_raw_frame()` | Polls the receiver without invoking processors or sinks |
| `process_frame(frame)` | Sends an existing frame through the configured extension pipeline |
| `get_receive_rate()` | Returns the receiver's current message rate |
| `add_processor()` / `remove_processor()` | Adds or removes one processor |
| `add_sink()` / `remove_sink()` | Adds or removes one sink |

With no processors or sinks, `get_latest_frame()` returns exactly the raw frame
obtained from `MocapReceiver`. If a processor fails, remaining processors and
sinks are skipped for that frame. If a sink fails, remaining sinks still run.
All extension exceptions are logged and sent to `on_extension_error`; they do
not stop the receiver or raw recorder.

## `MocapReceiver`

```python
MocapReceiver(port=11235, recorder=None, *, host="0.0.0.0")
```

Receives OSC messages on a background UDP thread and assembles chunked
`/MOVIN/Frame` messages.

`host` is the receiver computer's local bind address. Use `0.0.0.0` to accept
packets on every local IPv4 interface or a specific local address such as
`192.168.0.25` to restrict reception to that interface. This is not the MOVIN
Studio sender address. Configure MOVIN Studio's destination to the receiver
computer's reachable IP and the same UDP port.

| Member | Behavior |
|--------|----------|
| `start()` | Resets buffers and starts the receiver thread |
| `stop()` | Stops the thread, closes the socket, and resets buffers |
| `get_latest_frame()` | Returns the newest complete frame or `None`; older queued frames are discarded |
| `get_receive_rate()` | Returns the measured OSC packet rate in Hz |
| `reset()` | Clears partial frames, ready frames, and rate state |
| `recorder` | Recorder receiving every successfully parsed OSC message |

The recorder is called before messages enter `MovinFrameAssembler`. Changing
`receiver.recorder` is serialized with active `record()` calls.

### Frame schema

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
            "p": (float, float, float),
            "rq": (float, float, float, float),
            "q": (float, float, float, float),
            "s": (float, float, float),
        },
        ...
    ],
}
```

`p` is the local position; `rq` and `q` are rest and local rotation
quaternions in `(w, x, y, z)` order; `s` is scale.

## Recording and Playback

Recordings store parsed OSC messages so replay passes through the same frame
assembly path as live reception.

### `OscRecorder`

```python
OscRecorder(output_path, stream_type="auto")
```

| Member | Behavior |
|--------|----------|
| `record(address, args, wall_time=None)` | Appends one parsed OSC message and relative timestamp |
| `save()` | Writes the versioned recording to `output_path` |

`stream_type` accepts `"movin"`, `"nova"`, or `"auto"`. In auto mode the
recorder detects the stream from OSC addresses and payload shape. `OscRecorder`
is thread-safe and can also be used as a context manager that saves on exit.

### Recording file schema

```python
{
    "version": 1,
    "stream_type": str,  # "movin", "nova", or "unknown"
    "created": str,
    "num_messages": int,
    "duration_sec": float,
    "messages": [
        {
            "t": float,
            "addr": str,
            "args": list,
        },
        ...
    ],
}
```

### `OscPlayer`

```python
OscPlayer(recording_path)
```

| Member | Behavior |
|--------|----------|
| `stream_type` | Recorded stream type |
| `num_messages` | Number of recorded messages |
| `duration_sec` | Recorded duration in seconds |
| `messages(realtime=False)` | Yields `(address, args)`; optionally preserves original timing |
| `messages_with_timing()` | Yields `(relative_time, address, args)` without sleeping |

Unsupported recording versions raise `ValueError` during construction.

### `ReplayMocapReceiver`

```python
ReplayMocapReceiver(recording_path, realtime=True, loop=True)
```

The replay receiver is a drop-in polling replacement for `MocapReceiver` and
provides `start()`, `stop()`, `get_latest_frame()`, and `get_receive_rate()`.

- `realtime=True` reproduces recorded message intervals.
- `loop=True` restarts playback after the final message.

### `peek_first_frame`

```python
frame = peek_first_frame(recording_path)
```

Returns the first complete assembled frame or `None` without starting a replay
thread.

## OSC and Frame Assembly

### `OscReader`

```python
reader = OscReader(packet_bytes)
address, args = reader.read_message()
```

Parses one OSC message from a UDP packet.

### `MovinFrameAssembler`

```python
MovinFrameAssembler(max_ready_frames=4, partial_ttl_sec=0.5)
```

| Member | Behavior |
|--------|----------|
| `ingest(address, args, now=None)` | Adds one parsed `/MOVIN/Frame` chunk |
| `pop_latest_frame()` | Returns the newest complete frame and discards older ready frames |
| `prune_stale(now=None)` | Removes incomplete frames older than `partial_ttl_sec` |

## Extension Contracts

The interfaces live in the core package so applications can create extensions
without importing any optional implementation.

```python
class MyProcessor:
    def process(self, frame):
        return transformed_frame


class MySink:
    def on_frame(self, frame):
        consume(frame)
```

`FrameProcessor` and `FrameSink` are runtime-checkable typing protocols for
these method shapes. See [Optional Extensions](OPTIONAL_EXTENSIONS.md) for the
built-in raw-frame `MocapViewer` sink, `Retargeter` processor, and robot-state
`MujocoViewer` sink.

## Optional Dependency Errors

The root package resolves legacy optional exports lazily. Therefore this always
works in a core-only environment:

```python
import movin_sdk_python
```

Using an unavailable optional symbol raises `MissingOptionalDependencyError`,
an `ImportError` subclass whose message includes the required extra. Its public
attributes are `feature`, `extra`, and `dependency`.
