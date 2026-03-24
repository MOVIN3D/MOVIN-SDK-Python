"""
MOVIN mesh model and Linear Blend Skinning (LBS) utilities.

Loads the pre-extracted MOVINMan mesh NPZ and implements forward kinematics +
LBS to produce posed vertices for the Isaac Lab mesh overlay.  Pure NumPy —
no PyTorch dependency.

Coordinate systems:
    Geometry space (NPZ vertices):  centimeters, Z-up
    Skeleton space (bone offsets):  meters, Y-up
    Inverse bind matrices:         geometry → bone-local (includes cm→m + Y↔Z)
    LBS output:                    meters, Y-up
    Final output:                  meters, Z-up (Isaac Lab)
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

from .isaac_lab_utils import FORWARD_MODE_CHOICES, SKELETON_BODY_NAMES
from .bvh_loader import quat_fk
from .quat_utils import (
    quat_mul,
    quat_conj,
    quat_normalize,
    unity_to_opengl_vec,
    unity_to_opengl_quat,
)

_RESOLVED_FILE = Path(__file__).resolve()
_PROJECT_ROOT = _RESOLVED_FILE.parents[2]
_DEFAULT_NPZ_PATH = _PROJECT_ROOT / "data" / "movinman_mesh.npz"

_ROT_Z_90 = np.array(
    [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float32,
)


def _validate_forward_mode(forward_mode: str) -> None:
    if forward_mode not in FORWARD_MODE_CHOICES:
        raise ValueError(
            f"Invalid forward_mode '{forward_mode}'. "
            f"Expected one of: {', '.join(FORWARD_MODE_CHOICES)}"
        )


def convert_movin_vertices_to_isaac(vertices_yup, forward_mode="coord_equivalent"):
    """Convert Y-up MOVIN mesh vertices into Isaac Z-up world space.

    Same pattern as ``convert_nova_vertices_to_isaac`` in nova_mesh_utils.py.
    """
    _validate_forward_mode(forward_mode)
    vertices_yup = np.asarray(vertices_yup, dtype=np.float32)
    if vertices_yup.ndim != 2 or vertices_yup.shape[1] != 3:
        raise ValueError(f"Expected vertices shape (N, 3), got {vertices_yup.shape}")

    vertices_zup = np.empty_like(vertices_yup, dtype=np.float32)
    vertices_zup[:, 0] = vertices_yup[:, 0]
    vertices_zup[:, 1] = -vertices_yup[:, 2]
    vertices_zup[:, 2] = vertices_yup[:, 1]

    if forward_mode == "isaac_world":
        vertices_zup = vertices_zup @ _ROT_Z_90.T
    return vertices_zup


def _quat_to_rotmat(q):
    """Convert quaternion (w, x, y, z) to 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _quat_to_rotmat_batch(quats):
    """Convert (N, 4) quaternions (w, x, y, z) to (N, 3, 3) rotation matrices."""
    w, x, y, z = quats[:, 0], quats[:, 1], quats[:, 2], quats[:, 3]
    N = len(quats)
    R = np.zeros((N, 3, 3), dtype=np.float64)
    R[:, 0, 0] = 1 - 2*(y*y + z*z)
    R[:, 0, 1] = 2*(x*y - w*z)
    R[:, 0, 2] = 2*(x*z + w*y)
    R[:, 1, 0] = 2*(x*y + w*z)
    R[:, 1, 1] = 1 - 2*(x*x + z*z)
    R[:, 1, 2] = 2*(y*z - w*x)
    R[:, 2, 0] = 2*(x*z - w*y)
    R[:, 2, 1] = 2*(y*z + w*x)
    R[:, 2, 2] = 1 - 2*(x*x + y*y)
    return R


class MOVINMeshModel:
    """Load pre-extracted mesh NPZ, implement LBS, produce posed vertices."""

    def __init__(self, npz_path: str | None = None):
        if npz_path is None:
            npz_path = str(_DEFAULT_NPZ_PATH)
        if not os.path.exists(npz_path):
            raise FileNotFoundError(
                f"Mesh NPZ not found: {npz_path}. "
                "Run scripts/extract_movinman_mesh.py first."
            )

        data = np.load(npz_path, allow_pickle=False)
        self._vertices = data["vertices"].astype(np.float32)           # (V, 3) geometry space
        self._faces = data["faces"].astype(np.int32)                   # (F, 3)
        self._bone_names = list(data["bone_names"])                    # (51,)
        self._inverse_bind_matrices = data["inverse_bind_matrices"].astype(np.float64)  # (51, 4, 4)
        self._bone_parents = list(data["bone_parents"].astype(int))    # (51,) list
        self._bone_offsets = data["bone_offsets"].astype(np.float64)    # (51, 3) Y-up meters
        self._weight_bone_indices = data["weight_bone_indices"].astype(np.int32)  # (V, 4)
        self._weight_values = data["weight_values"].astype(np.float32)           # (V, 4)

        # Validate bone names match SKELETON_BODY_NAMES
        if self._bone_names != list(SKELETON_BODY_NAMES):
            raise ValueError(
                f"NPZ bone names do not match SKELETON_BODY_NAMES:\n"
                f"  NPZ: {self._bone_names}\n  Expected: {list(SKELETON_BODY_NAMES)}"
            )

        # Pre-compute homogeneous bind vertices (V, 4) for LBS
        V = self._vertices.shape[0]
        self._bind_verts_homo = np.ones((V, 4), dtype=np.float64)
        self._bind_verts_homo[:, :3] = self._vertices

    @property
    def faces(self) -> np.ndarray:
        return self._faces

    @property
    def num_vertices(self) -> int:
        return self._vertices.shape[0]

    @property
    def identity_ready(self) -> bool:
        return True  # Fixed mesh, always ready

    def pose_vertices(
        self,
        local_quats_yup: np.ndarray,
        root_pos_yup: np.ndarray,
        forward_mode: str = "coord_equivalent",
    ) -> np.ndarray:
        """Pose the mesh using LBS and return vertices in Isaac Z-up space.

        Args:
            local_quats_yup: (51, 4) wxyz quaternions in Y-up skeleton space.
                These are delta rotations relative to the bind/rest pose.
            root_pos_yup: (3,) root position in Y-up meters.
            forward_mode: "coord_equivalent" or "isaac_world".

        Returns:
            (V, 3) float32 vertices in Isaac Z-up world space.
        """
        local_quats_yup = np.asarray(local_quats_yup, dtype=np.float64)
        root_pos_yup = np.asarray(root_pos_yup, dtype=np.float64).reshape(3)

        if local_quats_yup.shape != (51, 4):
            raise ValueError(
                f"Expected local_quats_yup shape (51, 4), got {local_quats_yup.shape}"
            )

        # 1. Build local positions for FK
        #    lpos[0] = root position, lpos[1:] = bone offsets from NPZ
        lpos = self._bone_offsets.copy()  # (51, 3)
        lpos[0] = root_pos_yup

        # 2. Forward Kinematics → global quaternions + positions
        global_quats, global_pos = quat_fk(local_quats_yup, lpos, self._bone_parents)
        # global_quats: (51, 4), global_pos: (51, 3)  — Y-up meters

        # 3. Build 4x4 world transforms from FK output
        rot_mats = _quat_to_rotmat_batch(global_quats)  # (51, 3, 3)
        world_transforms = np.zeros((51, 4, 4), dtype=np.float64)
        world_transforms[:, :3, :3] = rot_mats
        world_transforms[:, :3, 3] = global_pos
        world_transforms[:, 3, 3] = 1.0

        # 4. Skin matrices: skin_j = world_j @ ibm_j
        skin_matrices = np.einsum('jab,jbc->jac', world_transforms, self._inverse_bind_matrices)
        # (51, 4, 4)

        # 5. Per-vertex blending with up to 4 influences
        bi = self._weight_bone_indices   # (V, 4) int32
        wv = self._weight_values         # (V, 4) float32

        # Gather skin matrices for each vertex's bone influences: (V, 4, 4, 4)
        gathered = skin_matrices[bi]  # (V, 4, 4, 4)

        # Weighted blend: (V, 4, 4)
        wv_64 = wv.astype(np.float64)
        blended = np.einsum('vk,vkab->vab', wv_64, gathered)  # (V, 4, 4)

        # 6. Transform bind vertices: (V, 4, 4) @ (V, 4, 1) → (V, 4, 1)
        deformed_homo = np.einsum('vab,vb->va', blended, self._bind_verts_homo)  # (V, 4)
        deformed_yup = deformed_homo[:, :3].astype(np.float32)

        # 7. Convert Y-up → Z-up + optional forward mode
        return convert_movin_vertices_to_isaac(deformed_yup, forward_mode=forward_mode)


def extract_movin_local_quats_yup(bones):
    """Extract (51, 4) local quaternions + (3,) root pos from live OSC bones.

    The MOVIN live stream sends bones in Unity left-handed Y-up coords.
    This function converts to right-handed Y-up (rest-pose-removed).

    Args:
        bones: List of bone dicts from MocapReceiver with keys
            "bone_name", "p", "q", "rq".

    Returns:
        (local_quats_yup, root_pos_yup) where:
            local_quats_yup: (51, 4) wxyz quaternions, rest-pose-removed
            root_pos_yup: (3,) Y-up meters
    """
    bone_by_name = {b["bone_name"]: b for b in bones}

    local_quats = np.zeros((51, 4), dtype=np.float64)
    local_quats[:, 0] = 1.0  # identity default

    # Root position: combine Root + Hips
    movin_root = bone_by_name.get("Root")
    hips_bone = bone_by_name.get("Hips")

    if movin_root is not None and hips_bone is not None:
        # Live stream: Root carries global transform, Hips is local to Root
        p_root_rh = unity_to_opengl_vec(np.array(movin_root["p"], dtype=np.float64))
        q_root_rh = unity_to_opengl_quat(np.array(movin_root["q"], dtype=np.float64))
        rq_root_rh = unity_to_opengl_quat(np.array(movin_root["rq"], dtype=np.float64))
        q_root_local = quat_normalize(quat_mul(quat_conj(rq_root_rh), q_root_rh))

        p_hips_rh = unity_to_opengl_vec(np.array(hips_bone["p"], dtype=np.float64))
        q_hips_rh = unity_to_opengl_quat(np.array(hips_bone["q"], dtype=np.float64))
        rq_hips_rh = unity_to_opengl_quat(np.array(hips_bone["rq"], dtype=np.float64))
        q_hips_local = quat_normalize(quat_mul(quat_conj(rq_hips_rh), q_hips_rh))

        # Global Hips = Root * Hips
        from .quat_utils import rotate_vec_by_quat
        root_pos_yup = p_root_rh + rotate_vec_by_quat(p_hips_rh, q_root_local)
        local_quats[0] = quat_normalize(quat_mul(q_root_local, q_hips_local))
    elif hips_bone is not None:
        p_rh = unity_to_opengl_vec(np.array(hips_bone["p"], dtype=np.float64))
        q_rh = unity_to_opengl_quat(np.array(hips_bone["q"], dtype=np.float64))
        rq_rh = unity_to_opengl_quat(np.array(hips_bone["rq"], dtype=np.float64))
        root_pos_yup = p_rh
        local_quats[0] = quat_normalize(quat_mul(quat_conj(rq_rh), q_rh))
    else:
        root_pos_yup = np.array([0.0, 0.8698, 0.0])

    # Non-root bones
    for i, name in enumerate(SKELETON_BODY_NAMES[1:], start=1):
        bone = bone_by_name.get(name)
        if bone is None:
            continue
        q_rh = unity_to_opengl_quat(np.array(bone["q"], dtype=np.float64))
        rq_rh = unity_to_opengl_quat(np.array(bone["rq"], dtype=np.float64))
        local_quats[i] = quat_normalize(quat_mul(quat_conj(rq_rh), q_rh))

    return local_quats, root_pos_yup


def extract_bvh_local_quats_yup(quats, positions, bone_names):
    """Extract (51, 4) local quaternions + (3,) root pos from a BVH frame.

    BVH data is already in right-handed Y-up coordinates.

    Args:
        quats: (J, 4) local quaternions in (w, x, y, z) for this frame.
        positions: (J, 3) local positions for this frame.
        bone_names: List of BVH bone names (J entries).

    Returns:
        (local_quats_yup, root_pos_yup) where:
            local_quats_yup: (51, 4) wxyz quaternions
            root_pos_yup: (3,) Y-up meters (NOTE: caller must apply bvh_scale)
    """
    bvh_name_to_idx = {name: i for i, name in enumerate(bone_names)}

    local_quats = np.zeros((51, 4), dtype=np.float64)
    local_quats[:, 0] = 1.0  # identity default

    # Root position and rotation from BVH Hips (index 0)
    root_pos_yup = np.array(positions[0], dtype=np.float64)
    local_quats[0] = quats[0]

    # Non-root bones
    for i, name in enumerate(SKELETON_BODY_NAMES[1:], start=1):
        bvh_idx = bvh_name_to_idx.get(name)
        if bvh_idx is None:
            continue
        local_quats[i] = quats[bvh_idx]

    return local_quats, root_pos_yup
