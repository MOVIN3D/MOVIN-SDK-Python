"""
Skeleton preset definitions - single source of truth for MOVIN body layouts.

A preset bundles everything name-keyed code needs to drive an articulation for
a given MOVINMan skeleton: the DFS body order (root "Hips" first), the default
hips height, and the associated MJCF / mesh assets.  The assets ship inside the
package (``movin_sdk_python/assets``); ``mjcf_path`` / ``mesh_npz_path`` resolve
them.  They carry the reference T-pose proportions -- see ``skeleton_fitting``
to fit them to an actor's streamed proportions.

Two presets are supported:
    - movinman     : 51-body legacy skeleton (MOVINMan.fbx)
    - movinman_v3  : 54-joint skeleton (MOVINman_V3_local_rotation_identity.fbx)
                     adding Spine2/Spine3/Neck1

Rest-pose convention (both presets): the MJCF assets place every body at the
joint's offset in its parent's local frame, as read from a T-pose whose local
rotations are all identity (for V3: ``MOVINman_V3_local_rotation_identity_tpose.bvh``,
the rest pose of ``MOVINman_V3_local_rotation_identity.fbx``).  That is the
frame MOVIN Studio streams ``p`` / ``q`` in and the OFFSET frame of its BVH
exports, so the streamed local rotation drives the hinges directly; the streamed
rest quaternion ``rq`` is not removed (see isaac_lab_utils._convert_bone).

This module must NOT import isaac_lab_utils (or any other utils module that
imports it) so it can stay the base of the dependency graph.  isaac_lab_utils
re-exports the legacy constants FROM here.
"""

from dataclasses import dataclass
from pathlib import Path


# Directory of the packaged MJCF / mesh assets.
ASSET_DIR = Path(__file__).resolve().parent.parent / "assets"


def asset_path(filename):
    """Absolute path of a packaged asset file (no existence check)."""
    return str(ASSET_DIR / filename)


# DFS body order of the legacy MOVINMan skeleton (matches the MJCF hierarchy).
MOVINMAN_BODY_NAMES = (
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
)

# DFS body order of the MOVINManV3 skeleton (matches assets/movinman_v3_skeleton.xml,
# which is generated from MOVINman_V3_local_rotation_identity_tpose.bvh, as well
# as the bone order of MOVIN Studio's V3 live stream and BVH exports).  Adds
# Spine2/Spine3/Neck1 vs legacy; Neck and both shoulders are parented under
# Spine3; fingers run Thumb, Index, Middle, Ring, Pinky within each hand.  (The
# FBX file lists fingers alphabetically -- Index, Middle, Pinky, Ring, Thumb --
# but no runtime path depends on that order; everything is looked up by name.)
MOVINMAN_V3_BODY_NAMES = (
    "Hips",
    "Spine", "Spine1", "Spine2", "Spine3",
    "Neck", "Neck1", "Head",
    "RightShoulder", "RightArm", "RightForeArm", "RightHand",
    "RightHandThumb1", "RightHandThumb2", "RightHandThumb3",
    "RightHandIndex1", "RightHandIndex2", "RightHandIndex3",
    "RightHandMiddle1", "RightHandMiddle2", "RightHandMiddle3",
    "RightHandRing1", "RightHandRing2", "RightHandRing3",
    "RightHandPinky1", "RightHandPinky2", "RightHandPinky3",
    "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
    "LeftHandThumb1", "LeftHandThumb2", "LeftHandThumb3",
    "LeftHandIndex1", "LeftHandIndex2", "LeftHandIndex3",
    "LeftHandMiddle1", "LeftHandMiddle2", "LeftHandMiddle3",
    "LeftHandRing1", "LeftHandRing2", "LeftHandRing3",
    "LeftHandPinky1", "LeftHandPinky2", "LeftHandPinky3",
    "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase",
    "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase",
)

# Bones unique to the V3 skeleton - presence of any of these marks a stream V3.
_V3_MARKER_BONES = frozenset({"Spine3", "Neck1"})


@dataclass(frozen=True)
class SkeletonPreset:
    """Immutable description of a MOVINMan skeleton layout.

    Attributes:
        name: Preset identifier ("movinman" or "movinman_v3").
        body_names: DFS body order incl. the root "Hips".
        default_hips_height: Rest-pose hips height in meters (Z-up).
        mjcf_filename: MJCF asset filename inside ``ASSET_DIR``.
        mesh_npz_filename: LBS mesh asset filename inside ``ASSET_DIR``, or
            None if the preset has no mesh overlay asset.
    """

    name: str
    body_names: tuple
    default_hips_height: float
    mjcf_filename: str
    mesh_npz_filename: str = None

    @property
    def joint_bones(self):
        """Non-root bones in skeleton order (each drives 3 DOFs: _x, _y, _z)."""
        return self.body_names[1:]

    @property
    def mjcf_path(self):
        """Absolute path of the packaged MJCF skeleton."""
        return asset_path(self.mjcf_filename)

    @property
    def mesh_npz_path(self):
        """Absolute path of the packaged LBS mesh, or None without a mesh asset."""
        if self.mesh_npz_filename is None:
            return None
        return asset_path(self.mesh_npz_filename)


MOVINMAN_PRESET = SkeletonPreset(
    name="movinman",
    body_names=MOVINMAN_BODY_NAMES,
    default_hips_height=0.8698,
    mjcf_filename="movinman_skeleton.xml",
    mesh_npz_filename="movinman_mesh.npz",
)

MOVINMAN_V3_PRESET = SkeletonPreset(
    name="movinman_v3",
    body_names=MOVINMAN_V3_BODY_NAMES,
    default_hips_height=0.8906,
    mjcf_filename="movinman_v3_skeleton.xml",
    mesh_npz_filename="movinman_v3_mesh.npz",
)

PRESETS = {
    "movinman": MOVINMAN_PRESET,
    "movinman_v3": MOVINMAN_V3_PRESET,
}


def get_preset(name):
    """Look up a preset by name.

    Args:
        name: Preset identifier ("movinman" or "movinman_v3").

    Returns:
        The matching SkeletonPreset.

    Raises:
        KeyError: If ``name`` is not a known preset.
    """
    try:
        return PRESETS[name]
    except KeyError:
        raise KeyError(
            f"Unknown skeleton preset '{name}'. "
            f"Valid presets: {sorted(PRESETS)}"
        )


def detect_preset_from_bone_names(names):
    """Detect the skeleton preset from a collection of bone names.

    V3-only bones (Spine3, Neck1) mark a V3 stream; anything else falls back to
    the legacy movinman preset.

    Args:
        names: Iterable of bone name strings (order irrelevant).

    Returns:
        MOVINMAN_V3_PRESET if any V3 marker bone is present, else MOVINMAN_PRESET.
    """
    if _V3_MARKER_BONES & set(names):
        return MOVINMAN_V3_PRESET
    return MOVINMAN_PRESET
