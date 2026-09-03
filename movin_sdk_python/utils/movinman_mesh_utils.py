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

from .isaac_lab_utils import (
    FORWARD_MODE_CHOICES,
    SKELETON_BODY_NAMES,
    DEFAULT_HIPS_HEIGHT,
    hips_global_transform_rh,
)
from .quat_utils import (
    quat_mul,
    quat_conj,
    quat_fk,
    quat_normalize,
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

    def __init__(self, npz_path: str | None = None, expected_bone_names=None):
        if npz_path is None:
            npz_path = str(_DEFAULT_NPZ_PATH)
        if expected_bone_names is None:
            expected_bone_names = SKELETON_BODY_NAMES
        if not os.path.exists(npz_path):
            raise FileNotFoundError(
                f"Mesh NPZ not found: {npz_path}. "
                "Run scripts/extract_movinman_mesh.py first."
            )

        data = np.load(npz_path, allow_pickle=False)
        self._vertices = data["vertices"].astype(np.float32)           # (V, 3) geometry space
        self._faces = data["faces"].astype(np.int32)                   # (F, 3)
        self._bone_names = list(data["bone_names"])                    # (B,)
        self._inverse_bind_matrices = data["inverse_bind_matrices"].astype(np.float64)  # (B, 4, 4)
        self._bone_parents = list(data["bone_parents"].astype(int))    # (B,) list
        self._bone_offsets = data["bone_offsets"].astype(np.float64)    # (B, 3) Y-up meters
        self._weight_bone_indices = data["weight_bone_indices"].astype(np.int32)  # (V, 4)
        self._weight_values = data["weight_values"].astype(np.float32)           # (V, 4)

        # Validate bone names match the expected skeleton preset order
        if self._bone_names != list(expected_bone_names):
            raise ValueError(
                f"NPZ bone names do not match expected bone names:\n"
                f"  NPZ: {self._bone_names}\n  Expected: {list(expected_bone_names)}"
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
            local_quats_yup: (B, 4) wxyz quaternions in Y-up skeleton space,
                where B is the mesh bone count.  These are delta rotations
                relative to the bind/rest pose.
            root_pos_yup: (3,) root position in Y-up meters.
            forward_mode: "coord_equivalent" or "isaac_world".

        Returns:
            (V, 3) float32 vertices in Isaac Z-up world space.
        """
        local_quats_yup = np.asarray(local_quats_yup, dtype=np.float64)
        root_pos_yup = np.asarray(root_pos_yup, dtype=np.float64).reshape(3)

        num_bones = len(self._bone_names)
        if local_quats_yup.shape != (num_bones, 4):
            raise ValueError(
                f"Expected local_quats_yup shape ({num_bones}, 4), "
                f"got {local_quats_yup.shape}"
            )

        # 1. Build local positions for FK
        #    lpos[0] = root position, lpos[1:] = bone offsets from NPZ
        lpos = self._bone_offsets.copy()  # (B, 3)
        lpos[0] = root_pos_yup

        # 2. Forward Kinematics → global quaternions + positions
        global_quats, global_pos = quat_fk(local_quats_yup, lpos, self._bone_parents)
        # global_quats: (B, 4), global_pos: (B, 3)  — Y-up meters

        # 3. Build 4x4 world transforms from FK output
        rot_mats = _quat_to_rotmat_batch(global_quats)  # (B, 3, 3)
        world_transforms = np.zeros((num_bones, 4, 4), dtype=np.float64)
        world_transforms[:, :3, :3] = rot_mats
        world_transforms[:, :3, 3] = global_pos
        world_transforms[:, 3, 3] = 1.0

        # 4. Skin matrices: skin_j = world_j @ ibm_j
        skin_matrices = np.einsum('jab,jbc->jac', world_transforms, self._inverse_bind_matrices)
        # (B, 4, 4)

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


def extract_movin_local_quats_yup(bones, skeleton_body_names=None):
    """Extract (B, 4) local quaternions + (3,) root pos from live OSC bones.

    The MOVIN live stream sends bones in Unity left-handed Y-up coords.
    This function converts to right-handed Y-up (rest-pose-removed).

    Args:
        bones: List of bone dicts from MocapReceiver with keys
            "bone_name", "p", "q", "rq".
        skeleton_body_names: DFS body order incl. root "Hips".
            Defaults to the legacy SKELETON_BODY_NAMES.

    Returns:
        (local_quats_yup, root_pos_yup) where:
            local_quats_yup: (B, 4) wxyz quaternions, rest-pose-removed
                (B = len(skeleton_body_names))
            root_pos_yup: (3,) Y-up meters
    """
    if skeleton_body_names is None:
        skeleton_body_names = SKELETON_BODY_NAMES

    bone_by_name = {b["bone_name"]: b for b in bones}

    local_quats = np.zeros((len(skeleton_body_names), 4), dtype=np.float64)
    local_quats[:, 0] = 1.0  # identity default

    # Root: global Hips pose, resolved through the streamed parent_index chain
    # (root bone name is irrelevant: "Root", "RootBone", ...).
    hips_transform = hips_global_transform_rh(bones)
    if hips_transform is None:
        root_pos_yup = np.array([0.0, DEFAULT_HIPS_HEIGHT, 0.0])
    else:
        root_pos_yup, local_quats[0] = hips_transform

    # Non-root bones
    for i, name in enumerate(skeleton_body_names[1:], start=1):
        bone = bone_by_name.get(name)
        if bone is None:
            continue
        q_rh = unity_to_opengl_quat(np.array(bone["q"], dtype=np.float64))
        rq_rh = unity_to_opengl_quat(np.array(bone["rq"], dtype=np.float64))
        local_quats[i] = quat_normalize(quat_mul(quat_conj(rq_rh), q_rh))

    return local_quats, root_pos_yup
