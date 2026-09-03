"""MOVIN Studio motion-capture receiver and recording SDK.

The default package contains the dependency-free receive, record, and replay
workflow. Robot retargeting and MuJoCo visualization remain available through
optional installation extras and are imported lazily for API compatibility.
"""

from importlib import import_module

from .exceptions import MissingOptionalDependencyError
from .mocap_receiver import MocapReceiver, MovinFrameAssembler, OscReader
from .pipeline import FrameProcessor, FrameSink
from .recording import OscPlayer, OscRecorder, ReplayMocapReceiver, peek_first_frame
from .session import MovinSession


__version__ = "0.1.0"

# Keep the established top-level API without importing optional dependencies
# during ``import movin_sdk_python``.
_OPTIONAL_EXPORTS = {
    "Retargeter": (".retargeter", "Retargeter", "retargeting"),
    "MocapViewer": (".viewer", "MocapViewer", "viewer"),
    "MujocoViewer": (".viewer", "MujocoViewer", "viewer"),
    "process_mocap_frame": (".utils", "process_mocap_frame", "retargeting"),
}


def __getattr__(name):
    export = _OPTIONAL_EXPORTS.get(name)
    if export is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

    module_name, attribute_name, extra = export
    try:
        value = getattr(import_module(module_name, __name__), attribute_name)
    except MissingOptionalDependencyError:
        raise
    except ModuleNotFoundError as exc:
        if exc.name and exc.name.startswith(__name__):
            raise
        raise MissingOptionalDependencyError(name, extra, exc.name) from exc
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(_OPTIONAL_EXPORTS))


__all__ = [
    "MocapReceiver",
    "MovinFrameAssembler",
    "OscReader",
    "MovinSession",
    "OscRecorder",
    "OscPlayer",
    "ReplayMocapReceiver",
    "peek_first_frame",
    "FrameProcessor",
    "FrameSink",
    "MissingOptionalDependencyError",
    # Compatibility exports, resolved lazily.
    "Retargeter",
    "MocapViewer",
    "MujocoViewer",
    "process_mocap_frame",
]
