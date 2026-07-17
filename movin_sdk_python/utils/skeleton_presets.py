"""
Skeleton preset definitions - single source of truth for MOVIN body layouts.

A preset bundles everything name-keyed code needs to drive an articulation for
a given MOVINMan skeleton: the DFS body order (root "Hips" first), the default
hips height, and the associated MJCF / mesh asset filenames.

Two presets are supported:
    - movinman     : 51-body legacy skeleton (MOVINMan.fbx)
    - movinman_v3  : 54-joint skeleton (MOVINManV3) adding Spine2/Spine3/Neck1

This module must NOT import isaac_lab_utils (or any other utils module that
imports it) so it can stay the base of the dependency graph.  isaac_lab_utils
re-exports the legacy constants FROM here.
"""

from dataclasses import dataclass


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

# DFS body order of the MOVINManV3 skeleton (MOVINManV3_Tpose.bvh).  Adds
# Spine2/Spine3/Neck1 vs legacy; Neck & both Shoulders parent under Spine3.
MOVINMAN_V3_BODY_NAMES = (
    "Hips",
    "Spine", "Spine1", "Spine2", "Spine3",
    "Neck", "Neck1", "Head",
    "RightShoulder", "RightArm", "RightForeArm", "RightHand",
    "RightHandIndex1", "RightHandIndex2", "RightHandIndex3",
    "RightHandMiddle1", "RightHandMiddle2", "RightHandMiddle3",
    "RightHandPinky1", "RightHandPinky2", "RightHandPinky3",
    "RightHandRing1", "RightHandRing2", "RightHandRing3",
    "RightHandThumb1", "RightHandThumb2", "RightHandThumb3",
    "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
    "LeftHandIndex1", "LeftHandIndex2", "LeftHandIndex3",
    "LeftHandMiddle1", "LeftHandMiddle2", "LeftHandMiddle3",
    "LeftHandPinky1", "LeftHandPinky2", "LeftHandPinky3",
    "LeftHandRing1", "LeftHandRing2", "LeftHandRing3",
    "LeftHandThumb1", "LeftHandThumb2", "LeftHandThumb3",
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
        mjcf_filename: MJCF asset filename (resolved by the caller against its
            own data directory).
        mesh_npz_filename: LBS mesh asset filename, or None if the preset has
            no mesh overlay asset.
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
    mesh_npz_filename=None,
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
