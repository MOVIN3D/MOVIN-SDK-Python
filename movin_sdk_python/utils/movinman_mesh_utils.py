"""
MOVIN mesh model and Linear Blend Skinning (LBS) utilities.

Loads a pre-extracted MOVIN character mesh NPZ (``movinman_mesh.npz`` for the
legacy MOVINMan rig, ``movinman_v3_mesh.npz`` for MOVINManV3; both ship in
``movin_sdk_python/assets``, see ``SkeletonPreset.mesh_npz_path``) and implements
forward kinematics + LBS to produce posed vertices for the Isaac Lab mesh
overlay.  Pure NumPy — no PyTorch dependency.

The NPZ carries the reference character's proportions.  ``fit_to_bones`` (or
``pose_from_bones``, which refits whenever the streamed proportions change)
rebinds the mesh to the actor's streamed bone offsets so the posed mesh
follows the stream's joints exactly.

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
    quat_fk,
    quat_normalize,
    unity_to_opengl_quat,
)
from .skeleton_fitting import fitted_offsets
from .skeleton_presets import MOVINMAN_PRESET, SkeletonPreset, get_preset

# Packaged legacy mesh, matching the legacy default ``expected_bone_names``.
_DEFAULT_NPZ_PATH = Path(MOVINMAN_PRESET.mesh_npz_path)

# Fitting: per-bone stretch is regularized toward identity (relative to the
# squared child-offset lengths) and its singular values clamped to this range.
_FIT_REGULARIZATION = 1e-3
_FIT_SCALE_RANGE = (0.5, 2.0)

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
                f"Mesh NPZ not found: {npz_path}. The packaged meshes live in "
                "movin_sdk_python/assets (see SkeletonPreset.mesh_npz_path)."
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

        # Reference (unfitted) bind data, restored by reset_fit().
        self._parent_names = [None] + [
            self._bone_names[p] for p in self._bone_parents[1:]
        ]
        self._reference = (
            self._vertices,
            self._bind_verts_homo,
            self._inverse_bind_matrices,
            self._bone_offsets,
        )
        self._fit_key = None
        self._fitted_bones = ()

    @classmethod
    def from_preset(cls, preset="movinman_v3"):
        """Load the packaged mesh of a skeleton preset (name or SkeletonPreset)."""
        if not isinstance(preset, SkeletonPreset):
            preset = get_preset(preset)
        if preset.mesh_npz_path is None:
            raise ValueError(f"Skeleton preset '{preset.name}' has no mesh asset")
        return cls(preset.mesh_npz_path, expected_bone_names=preset.body_names)

    @property
    def bone_names(self):
        return list(self._bone_names)

    @property
    def is_fitted(self) -> bool:
        """Whether the mesh is currently rebound to streamed proportions."""
        return bool(self._fitted_bones)

    @property
    def fitted_bones(self):
        """Bones whose offsets came from the stream in the current fit."""
        return self._fitted_bones

    def reset_fit(self):
        """Return to the reference character's proportions."""
        (
            self._vertices,
            self._bind_verts_homo,
            self._inverse_bind_matrices,
            self._bone_offsets,
        ) = self._reference
        self._fit_key = None
        self._fitted_bones = ()

    def fit_to_bones(self, bones):
        """Rebind the mesh to the bone offsets of a streamed frame.

        Every bone takes the streamed parent-local offset (bones missing from
        the stream keep the reference one), so posing reproduces the stream's
        joint positions.  Vertices move with their bones into the actor's
        T-pose: each bone's region is stretched along its child bones by the
        streamed / reference length ratio and kept as thick as the reference
        across them (leaf bones -- head, finger tips, toes -- are only moved).
        The root is placed so the T-pose's lowest joint keeps the reference
        ground clearance.

        Args:
            bones: ``frame["bones"]`` from a MOVIN receiver.

        Returns:
            self

        Raises:
            ValueError: If the stream's hierarchy does not match the mesh, or
                the mesh's bind pose has rotated bones (fitting assumes the
                identity-local T-pose both packaged meshes are bound in).
        """
        _, _, ref_ibm, ref_offsets = self._reference
        offsets, fitted = fitted_offsets(
            bones, self._bone_names, self._parent_names, [tuple(o) for o in ref_offsets]
        )
        if not fitted:
            self.reset_fit()
            return self

        geom_to_skel = ref_ibm[0, :3, :3]
        if not np.allclose(ref_ibm[:, :3, :3], geom_to_skel, atol=1e-6):
            raise ValueError("Mesh bind pose has rotated bones; cannot fit proportions")
        parents = np.asarray(self._bone_parents)
        ref_joints = -ref_ibm[:, :3, 3]                      # bind joints, Y-up m
        ref_local = ref_joints[1:] - ref_joints[parents[1:]]
        if not np.allclose(ref_local, ref_offsets[1:], atol=1e-4):
            raise ValueError("Mesh bind joints disagree with its bone offsets")

        new_offsets = np.asarray(offsets, dtype=np.float64)
        new_offsets[0] = 0.0
        joints = np.zeros_like(ref_joints)
        for i in range(1, len(joints)):
            joints[i] = joints[parents[i]] + new_offsets[i]
        root = ref_joints[0].copy()
        root[1] = ref_joints[:, 1].min() - joints[:, 1].min()
        joints += root
        new_offsets[0] = root

        stretch = _bone_stretch(parents, ref_offsets, new_offsets)

        # Bind vertices -> skeleton space -> actor T-pose (blended per influence).
        ref_vertices = self._reference[0].astype(np.float64)
        verts_skel = ref_vertices @ geom_to_skel.T
        bi, wv = self._weight_bone_indices, self._weight_values.astype(np.float64)
        moved = np.zeros_like(verts_skel)
        for k in range(bi.shape[1]):
            j = bi[:, k]
            local = verts_skel - ref_joints[j]
            moved += wv[:, k:k + 1] * (joints[j] + np.einsum("vab,vb->va", stretch[j], local))
        moved /= np.maximum(wv.sum(axis=1, keepdims=True), 1e-12)
        vertices = moved @ np.linalg.inv(geom_to_skel).T

        ibm = ref_ibm.copy()
        ibm[:, :3, 3] = -joints
        self._vertices = vertices.astype(np.float32)
        self._bind_verts_homo = np.ones((len(vertices), 4), dtype=np.float64)
        self._bind_verts_homo[:, :3] = vertices
        self._inverse_bind_matrices = ibm
        self._bone_offsets = new_offsets
        self._fitted_bones = tuple(fitted)
        return self

    def pose_from_bones(self, bones, forward_mode: str = "coord_equivalent", fit: bool = True):
        """Pose the mesh from a streamed frame's bones.

        Args:
            bones: ``frame["bones"]`` from a MOVIN receiver.
            forward_mode: "coord_equivalent" or "isaac_world".
            fit: Fit the actor's streamed proportions first.  The fit is
                redone only when the streamed offsets change (e.g. the actor is
                recalibrated in MOVIN Studio).

        Returns:
            (V, 3) float32 vertices in Isaac Z-up world space.
        """
        if fit:
            names = set(self._bone_names[1:])
            key = tuple(
                (b["bone_name"], tuple(b["p"])) for b in bones if b["bone_name"] in names
            )
            if key != self._fit_key:
                self.fit_to_bones(bones)
                self._fit_key = key
        local_quats, root_pos = extract_movin_local_quats_yup(
            bones, skeleton_body_names=self._bone_names
        )
        return self.pose_vertices(local_quats, root_pos, forward_mode=forward_mode)

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
                where B is the mesh bone count.  Each is the bone's rotation
                relative to its parent's frame; the NPZ rig's bind pose has
                identity local rotations, so these are also the deltas from
                the bind pose.
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


def _bone_stretch(parents, ref_offsets, new_offsets):
    """(B, 3, 3) per-bone linear maps taking reference bone-local geometry to the actor's.

    For a bone with children, the map sends each reference child offset to the
    actor's (regularized least squares), leaving directions the children do
    not span untouched -- a limb gets longer or shorter but not thicker.  Leaf
    bones get the identity.
    """
    count = len(parents)
    stretch = np.tile(np.eye(3), (count, 1, 1))
    for j in range(count):
        kids = [c for c in range(1, count) if parents[c] == j]
        if not kids:
            continue
        ref = np.asarray(ref_offsets, dtype=np.float64)[kids]
        new = np.asarray(new_offsets, dtype=np.float64)[kids]
        lam = _FIT_REGULARIZATION * float((ref * ref).sum()) + 1e-12
        m = ref.T @ ref + lam * np.eye(3)
        n = new.T @ ref + lam * np.eye(3)
        a = n @ np.linalg.inv(m)
        u, sv, vt = np.linalg.svd(a)
        stretch[j] = u @ np.diag(np.clip(sv, *_FIT_SCALE_RANGE)) @ vt
    return stretch


def extract_movin_local_quats_yup(bones, skeleton_body_names=None):
    """Extract (B, 4) local quaternions + (3,) root pos from live OSC bones.

    The MOVIN live stream sends bones in Unity left-handed Y-up coords.
    This function converts to right-handed Y-up.  The streamed local rotation
    ``q`` is used as-is (no rest-pose removal), matching isaac_lab_utils: the
    legacy MOVINMan rig streams identity ``rq`` so this is unchanged for the
    legacy mesh asset, and the MOVINManV3 asset (``movinman_v3_mesh.npz``) is
    extracted from ``MOVINman_V3_local_rotation_identity.fbx``, whose bind pose
    has identity local rotations, so the same quaternions drive it.

    Args:
        bones: List of bone dicts from MocapReceiver with keys
            "bone_name", "p", "q", "rq".
        skeleton_body_names: DFS body order incl. root "Hips".
            Defaults to the legacy SKELETON_BODY_NAMES.

    Returns:
        (local_quats_yup, root_pos_yup) where:
            local_quats_yup: (B, 4) wxyz local quaternions
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
        local_quats[i] = quat_normalize(q_rh)

    return local_quats, root_pos_yup
