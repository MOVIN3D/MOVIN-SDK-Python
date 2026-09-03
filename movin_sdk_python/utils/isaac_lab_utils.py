"""
Coordinate conversion utilities for MOVIN mocap -> Isaac Lab visualization.

Converts MOVIN bone data (Unity left-handed Y-up) to Isaac Lab (right-handed Z-up)
and produces DOF arrays for driving an MJCF-based articulation.

All quaternions are in (w, x, y, z) format.
"""

import numpy as np

from .skeleton_presets import MOVINMAN_PRESET
from .quat_utils import (
    quat_mul,
    quat_conj,
    quat_normalize,
    unity_to_opengl_vec,
    unity_to_opengl_quat,
    rotate_vec_by_quat,
)

# 90-degree rotation around X axis: transforms Y-up -> Z-up
Q_YUP_TO_ZUP = np.array([0.7071067811865476, 0.7071067811865476, 0.0, 0.0])
Q_YUP_TO_ZUP_CONJ = quat_conj(Q_YUP_TO_ZUP)
# 90-degree yaw around Z: rotates Isaac -Y forward to Isaac +X forward
Q_ISAAC_WORLD_FORWARD = np.array([0.7071067811865476, 0.0, 0.0, 0.7071067811865476])
FORWARD_MODE_CHOICES = ("coord_equivalent", "isaac_world")

# Legacy aliases sourced from the movinman preset (single source of truth:
# skeleton_presets.MOVINMAN_PRESET).  Kept as lists for backward compatibility.

# Default Hips height from MOVINMan.fbx skeleton (meters, Z-up)
DEFAULT_HIPS_HEIGHT = MOVINMAN_PRESET.default_hips_height

# DFS body order matching the MJCF skeleton hierarchy
SKELETON_BODY_NAMES = list(MOVINMAN_PRESET.body_names)

# Non-root bones in skeleton order (each gets 3 DOFs: _x, _y, _z)
SKELETON_JOINT_BONES = list(MOVINMAN_PRESET.joint_bones)  # everything except Hips


def yup_to_zup_vec(v):
    """Convert a Y-up position vector to Z-up: (x, y, z) -> (x, -z, y)."""
    return np.array([v[0], -v[2], v[1]])


def yup_to_zup_quat(q):
    """Convert a Y-up local rotation quaternion to Z-up via similarity transform.

    q_zup = Q_x90 * q_yup * conj(Q_x90)
    """
    return quat_normalize(quat_mul(Q_YUP_TO_ZUP, quat_mul(q, Q_YUP_TO_ZUP_CONJ)))


def quat_to_exp_map(q):
    """Convert quaternion (w,x,y,z) to exponential map (3D axis*angle).

    The exp map is the rotation axis scaled by the rotation angle.
    Returns a 3-element array suitable for MJCF hinge joint DOFs (x, y, z).
    """
    q = quat_normalize(q)
    # Ensure w >= 0 for consistent angle extraction
    if q[0] < 0:
        q = -q

    w = q[0]
    xyz = q[1:4]
    sin_half = np.linalg.norm(xyz)

    if sin_half < 1e-8:
        return np.zeros(3)

    half_angle = np.arctan2(sin_half, w)
    angle = 2.0 * half_angle
    axis = xyz / sin_half
    return axis * angle


def _validate_forward_mode(forward_mode):
    """Validate the requested root forward-alignment policy."""
    if forward_mode not in FORWARD_MODE_CHOICES:
        raise ValueError(
            f"Invalid forward_mode '{forward_mode}'. "
            f"Expected one of: {', '.join(FORWARD_MODE_CHOICES)}"
        )


def _apply_forward_mode_to_root(root_pos_zup, root_quat_zup, forward_mode):
    """Apply an optional visualization-space yaw correction to the root pose."""
    _validate_forward_mode(forward_mode)
    if forward_mode == "coord_equivalent":
        return root_pos_zup, root_quat_zup

    root_pos_zup = rotate_vec_by_quat(root_pos_zup, Q_ISAAC_WORLD_FORWARD)
    root_quat_zup = quat_normalize(quat_mul(Q_ISAAC_WORLD_FORWARD, root_quat_zup))
    return root_pos_zup, root_quat_zup


def _convert_bone(bone):
    """Convert a MOVIN bone dict from Unity to right-handed coords.

    Returns (pos_rh, q_local) where q_local has rest-pose removed.
    """
    p_rh = unity_to_opengl_vec(np.array(bone["p"], dtype=np.float64))
    q_rh = unity_to_opengl_quat(np.array(bone["q"], dtype=np.float64))
    rq_rh = unity_to_opengl_quat(np.array(bone["rq"], dtype=np.float64))
    q_local = quat_normalize(quat_mul(quat_conj(rq_rh), q_rh))
    return p_rh, q_local


def find_stream_root_bone(bones):
    """Return the bone at the top of the streamed hierarchy, or None.

    The root is the bone whose ``parent_index`` is negative (or points at a
    bone that is not part of the frame).  Its name is not inspected: legacy
    MOVINMan streams call it ``Root``, MOVINManV3 streams call it
    ``RootBone``, and other exporters may use other names.
    """
    by_index = {b["bone_index"]: b for b in bones if "bone_index" in b}
    for bone in bones:
        parent_index = bone.get("parent_index", -1)
        if parent_index is None or parent_index < 0 or parent_index not in by_index:
            return bone
    return None


def hips_global_transform_rh(bones):
    """Global right-handed Y-up transform of the ``Hips`` bone.

    MOVIN streams parent ``Hips`` under a stream root bone that carries the
    actor's global transform while ``Hips`` is expressed locally to it.  The
    chain is walked through ``parent_index`` (not bone names), so it works for
    a root called ``Root`` (legacy MOVINMan), ``RootBone`` (MOVINManV3), any
    other name, a deeper chain of ancestors, or a stream whose ``Hips`` is
    itself the root.

    Returns:
        ``(pos_rh, q_rh)`` with the rest pose removed from the rotation, or
        None when the frame has neither a ``Hips`` bone nor a root bone.
        Without a ``Hips`` bone the stream root's own transform is returned.
    """
    by_index = {b["bone_index"]: b for b in bones if "bone_index" in b}
    start = next((b for b in bones if b["bone_name"] == "Hips"), None)
    if start is None:
        start = find_stream_root_bone(bones)
    if start is None:
        return None

    pos_rh, q_rh = _convert_bone(start)
    visited = {start.get("bone_index")}
    parent_index = start.get("parent_index", -1)
    while (
        parent_index is not None
        and parent_index >= 0
        and parent_index in by_index
        and parent_index not in visited
    ):
        parent = by_index[parent_index]
        visited.add(parent_index)
        p_parent_rh, q_parent_rh = _convert_bone(parent)
        # child global = parent.pos + rotate(child.local_pos, parent.rot)
        pos_rh = p_parent_rh + rotate_vec_by_quat(pos_rh, q_parent_rh)
        # child global rot = parent.rot * child.local_rot
        q_rh = quat_normalize(quat_mul(q_parent_rh, q_rh))
        parent_index = parent.get("parent_index", -1)
    return pos_rh, q_rh


def process_movin_bones_for_isaaclab(
    bones,
    skeleton_bone_names=None,
    forward_mode="coord_equivalent",
):
    """Convert MOVIN OSC bone data to Isaac Lab DOF values.

    Pipeline per bone:
        1. Unity->right-handed: quat (w,x,-y,-z), pos (-x,y,z)
        2. Remove rest pose: q = conj(rq) * q
        3. Y-up -> Z-up similarity transform on quaternion
        4. Root: extract position + rotation
        5. Non-root: quaternion -> exp_map -> 3 DOF values

    Args:
        bones: List of bone dicts from MocapReceiver (same format as fk_utils expects)
        skeleton_bone_names: List of bone names matching MJCF joint order.
            Defaults to SKELETON_JOINT_BONES.
        forward_mode: Root-facing policy in Isaac visualization space.
            - "coord_equivalent": preserve the current Unity +Z -> Isaac -Y mapping
            - "isaac_world": add a +90 deg Isaac-Z yaw so Unity +Z faces Isaac +X

    Returns:
        (root_pos_zup, root_quat_zup, dof_array) where:
        - root_pos_zup: (3,) position in Z-up meters
        - root_quat_zup: (4,) quaternion (w,x,y,z) in Z-up
        - dof_array: (N*3,) hinge joint DOF values in skeleton order
    """
    if skeleton_bone_names is None:
        skeleton_bone_names = SKELETON_JOINT_BONES

    # Index bones by name for fast lookup
    bone_by_name = {}
    for bone in bones:
        bone_by_name[bone["bone_name"]] = bone

    # Process root: the streamed hierarchy is Root -> Hips (root carries the
    # global transform, Hips is local to it) or a lone global Hips.  Resolve it
    # through parent_index so the root bone's name ("Root", "RootBone", ...)
    # does not matter, then map the global Hips pose onto the MJCF freejoint.
    hips_transform = hips_global_transform_rh(bones)
    if hips_transform is None:
        root_pos_zup = np.array([0.0, 0.0, DEFAULT_HIPS_HEIGHT])
        root_quat_zup = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        p_hips_rh, q_hips_rh = hips_transform
        root_pos_zup = yup_to_zup_vec(p_hips_rh)
        root_quat_zup = yup_to_zup_quat(q_hips_rh)

    root_pos_zup, root_quat_zup = _apply_forward_mode_to_root(
        root_pos_zup, root_quat_zup, forward_mode
    )

    # Process non-root bones
    num_joints = len(skeleton_bone_names)
    dof_array = np.zeros(num_joints * 3)

    for i, bone_name in enumerate(skeleton_bone_names):
        bone = bone_by_name.get(bone_name)
        if bone is None:
            # Missing bone (e.g. hand joints not present) -> identity (zero DOFs)
            continue

        _, q_local = _convert_bone(bone)
        q_zup = yup_to_zup_quat(q_local)
        dof_array[i * 3: i * 3 + 3] = quat_to_exp_map(q_zup)

    return root_pos_zup, root_quat_zup, dof_array


def build_dof_reorder_map(isaac_joint_names, skeleton_bone_names=None):
    """Build a mapping from skeleton DOF order to Isaac Lab DOF order.

    Isaac Lab may reorder joints internally after MJCF->USD conversion.
    This function creates an index array so that:
        isaac_dof_values[reorder_map] = skeleton_dof_values

    Args:
        isaac_joint_names: List of joint names from Isaac Lab articulation
            (e.g., articulation.data.joint_names)
        skeleton_bone_names: Expected skeleton bone names (non-root).
            Defaults to SKELETON_JOINT_BONES.

    Returns:
        reorder_map: numpy array of indices, or None if orders already match
    """
    if skeleton_bone_names is None:
        skeleton_bone_names = SKELETON_JOINT_BONES

    # Build expected joint name list: BoneName_x, BoneName_y, BoneName_z
    expected_names = []
    for bone in skeleton_bone_names:
        expected_names.extend([f"{bone}_x", f"{bone}_y", f"{bone}_z"])

    # Build isaac name -> index map
    isaac_name_to_idx = {}
    for i, name in enumerate(isaac_joint_names):
        isaac_name_to_idx[name] = i

    # Build reorder map: skeleton_idx -> isaac_idx
    reorder_map = []
    missing = []
    for name in expected_names:
        if name in isaac_name_to_idx:
            reorder_map.append(isaac_name_to_idx[name])
        else:
            missing.append(name)

    if missing:
        raise ValueError(
            f"{len(missing)} joint(s) not found in Isaac Lab articulation: "
            f"{missing}. The MJCF and skeleton preset must match "
            f"(check the --preset selection against the loaded MJCF)."
        )

    reorder_map = np.array(reorder_map, dtype=np.int64)

    # Check if it's already identity
    if np.array_equal(reorder_map, np.arange(len(reorder_map))):
        return None

    return reorder_map
