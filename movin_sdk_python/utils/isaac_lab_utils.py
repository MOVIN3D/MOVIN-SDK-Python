"""
Coordinate conversion utilities for MOVIN mocap -> Isaac Lab visualization.

Converts MOVIN bone data (Unity left-handed Y-up) to Isaac Lab (right-handed Z-up)
and produces DOF arrays for driving an MJCF-based articulation.

All quaternions are in (w, x, y, z) format.
"""

import numpy as np

import warnings

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

# Default Hips height from MOVINMan.fbx skeleton (meters, Z-up)
DEFAULT_HIPS_HEIGHT = 0.8698

# DFS body order matching the MJCF skeleton hierarchy
SKELETON_BODY_NAMES = [
    "Hips",
    "Spine", "Spine1",
    "Neck", "Head",
    "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
    "LeftHandThumb1", "LeftHandThumb2", "LeftHandThumb3",
    "LeftHandIndex1", "LeftHandIndex2", "LeftHandIndex3",
    "LeftHandMiddle1", "LeftHandMiddle2", "LeftHandMiddle3",
    "LeftHandRing1", "LeftHandRing2", "LeftHandRing3",
    "LeftHandPinky1", "LeftHandPinky2", "LeftHandPinky3",
    "RightShoulder", "RightArm", "RightForeArm", "RightHand",
    "RightHandThumb1", "RightHandThumb2", "RightHandThumb3",
    "RightHandIndex1", "RightHandIndex2", "RightHandIndex3",
    "RightHandMiddle1", "RightHandMiddle2", "RightHandMiddle3",
    "RightHandRing1", "RightHandRing2", "RightHandRing3",
    "RightHandPinky1", "RightHandPinky2", "RightHandPinky3",
    "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase",
    "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase",
]

# Non-root bones in skeleton order (each gets 3 DOFs: _x, _y, _z)
SKELETON_JOINT_BONES = SKELETON_BODY_NAMES[1:]  # everything except Hips


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

    # Process root (Root + Hips)
    # MOVIN live stream sends 52 bones: "Root" (global transform) + "Hips" (local to Root).
    # BVH files use "Hips" directly as root with global transform.
    # We combine Root + Hips into the MJCF freejoint (global position + rotation).
    movin_root = bone_by_name.get("Root")
    hips_bone = bone_by_name.get("Hips")

    if hips_bone is None and movin_root is None:
        root_pos_zup = np.array([0.0, 0.0, DEFAULT_HIPS_HEIGHT])
        root_quat_zup = np.array([1.0, 0.0, 0.0, 0.0])
    elif movin_root is not None and hips_bone is not None:
        # Live stream: Root carries global transform, Hips is local to Root
        p_root_rh, q_root = _convert_bone(movin_root)
        p_hips_rh, q_hips = _convert_bone(hips_bone)

        # Global Hips position = Root.pos + rotate(Hips.local_pos, Root.rot)
        global_pos = p_root_rh + rotate_vec_by_quat(p_hips_rh, q_root)
        # Global Hips rotation = Root.rot * Hips.local_rot
        global_rot = quat_normalize(quat_mul(q_root, q_hips))

        root_pos_zup = yup_to_zup_vec(global_pos)
        root_quat_zup = yup_to_zup_quat(global_rot)
    else:
        # Hips only (BVH-style, no separate Root bone)
        p_rh, q_local = _convert_bone(hips_bone or movin_root)
        root_pos_zup = yup_to_zup_vec(p_rh)
        root_quat_zup = yup_to_zup_quat(q_local)

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


def process_bvh_frame_for_isaaclab(
    quats,
    positions,
    bone_names,
    skeleton_bone_names=None,
    forward_mode="coord_equivalent",
):
    """Convert a single BVH frame to Isaac Lab DOF values.

    BVH data is already in right-handed Y-up coordinates.

    Args:
        quats: (J, 4) local quaternions in (w,x,y,z) for this frame
        positions: (J, 3) local positions for this frame (used for root only)
        bone_names: List of BVH bone names
        skeleton_bone_names: List of bone names matching MJCF joint order.
            Defaults to SKELETON_JOINT_BONES.
        forward_mode: Root-facing policy in Isaac visualization space.

    Returns:
        (root_pos_zup, root_quat_zup, dof_array)
    """
    if skeleton_bone_names is None:
        skeleton_bone_names = SKELETON_JOINT_BONES

    # Build BVH name -> index map
    bvh_name_to_idx = {name: i for i, name in enumerate(bone_names)}

    # Root (index 0 = Hips)
    root_pos_yup = positions[0]
    root_quat_yup = quats[0]

    # Convert root position: BVH Y-up -> Z-up
    # Note: BVH units vary by file (cm or m) - no scaling applied here
    root_pos_zup = yup_to_zup_vec(root_pos_yup)

    # Convert root quaternion
    root_quat_zup = yup_to_zup_quat(root_quat_yup)
    root_pos_zup, root_quat_zup = _apply_forward_mode_to_root(
        root_pos_zup, root_quat_zup, forward_mode
    )

    # Process non-root joints
    num_joints = len(skeleton_bone_names)
    dof_array = np.zeros(num_joints * 3)

    for i, bone_name in enumerate(skeleton_bone_names):
        bvh_idx = bvh_name_to_idx.get(bone_name)
        if bvh_idx is None:
            # Missing bone in BVH -> identity (zero DOFs)
            continue

        q_local = quats[bvh_idx]

        # BVH local rotations are already rest-pose-relative
        # Convert Y-up -> Z-up
        q_zup = yup_to_zup_quat(q_local)

        # Quaternion -> exp map -> 3 DOF values
        exp = quat_to_exp_map(q_zup)
        dof_array[i * 3: i * 3 + 3] = exp

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
    for name in expected_names:
        if name in isaac_name_to_idx:
            reorder_map.append(isaac_name_to_idx[name])
        else:
            warnings.warn(f"Joint '{name}' not found in Isaac Lab articulation")
            reorder_map.append(i)  # map to own index as fallback

    reorder_map = np.array(reorder_map, dtype=np.int64)

    # Check if it's already identity
    if np.array_equal(reorder_map, np.arange(len(reorder_map))):
        return None

    return reorder_map
