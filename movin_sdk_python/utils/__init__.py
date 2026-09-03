"""Optional numeric utilities for motion processing and retargeting."""

from importlib import import_module

from ..exceptions import MissingOptionalDependencyError


_EXPORTS = {
    "process_mocap_frame": (".fk_utils", "process_mocap_frame"),
    "compute_forward_kinematics": (".fk_utils", "compute_forward_kinematics"),
    "add_foot_mod_bones": (".fk_utils", "add_foot_mod_bones"),
    "quat_mul": (".quat_utils", "quat_mul"),
    "rotate_vec_by_quat": (".quat_utils", "rotate_vec_by_quat"),
}


def __getattr__(name):
    export = _EXPORTS.get(name)
    if export is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attribute_name = export
    try:
        value = getattr(import_module(module_name, __name__), attribute_name)
    except ModuleNotFoundError as exc:
        if exc.name and exc.name.startswith("movin_sdk_python"):
            raise
        raise MissingOptionalDependencyError(
            "Motion processing utilities", "retargeting", exc.name
        ) from exc
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(_EXPORTS))

__all__ = [
    "process_mocap_frame",
    "compute_forward_kinematics",
    "add_foot_mod_bones",
    "quat_mul",
    "rotate_vec_by_quat",
]
