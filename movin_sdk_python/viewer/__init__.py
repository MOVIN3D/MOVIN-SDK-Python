"""
Viewer module for movin_sdk_python.

Provides MocapViewer for raw stick-figure visualization and MujocoViewer for
retargeted robot motion.
"""

from ..exceptions import MissingOptionalDependencyError

try:
    from .mocap_viewer import MocapViewer
    from .mujoco_viewer import MujocoViewer
except ModuleNotFoundError as exc:
    if exc.name and exc.name.startswith("movin_sdk_python"):
        raise
    raise MissingOptionalDependencyError("MuJoCo viewer", "viewer", exc.name) from exc

__all__ = ["MocapViewer", "MujocoViewer"]
