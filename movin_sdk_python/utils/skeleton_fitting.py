"""
Fit the packaged skeleton assets to the body proportions MOVIN Studio streams.

MOVIN Studio calibrates every actor: the parent-local bone offsets ``p`` in the
stream carry that actor's limb lengths, while the packaged MJCF and mesh assets
carry the reference T-pose.  Driving a reference asset with streamed rotations
places its joints away from where the stream's own forward kinematics puts
them and can leave its feet above or below the ground.

Fitting replaces the asset's offsets with the streamed ones, so the asset
reproduces the stream's joint positions exactly.  The streamed offsets point
along the same directions as the asset's (both rigs share identity-local
T-pose frames); only lengths differ.

This module is dependency-free.  The mesh counterpart is
``MOVINMeshModel.fit_to_bones`` in ``movinman_mesh_utils`` (needs NumPy).
``body_scale_to_reference`` gives the overall size ratio the Retargeter uses
to normalize an actor to the reference body its IK tables were tuned for.

Coordinates: the stream is Unity left-handed Y-up; offsets returned here are
right-handed Y-up meters ``(-px, py, pz)``, and MJCF bodies are Z-up
``(x, -z, y)`` of that -- the same conventions as ``fk_utils`` and the
MJCF generator.
"""

from __future__ import annotations

import functools
import math
import xml.etree.ElementTree as ET

from .skeleton_presets import SkeletonPreset, get_preset


# Leg chains below Hips; identical in the legacy and V3 skeletons.
LEG_CHAINS = (
    ("LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase"),
    ("RightUpLeg", "RightLeg", "RightFoot", "RightToeBase"),
)
LEG_BONES = frozenset(name for chain in LEG_CHAINS for name in chain)

# A capsule follows a child body when it points at it within this angle.
_CAPSULE_CHILD_MAX_DEG = 2.0
_MIN_CAPSULE_LENGTH = 0.005


def stream_offsets(bones):
    """Parent-local bone offsets of a streamed frame.

    Args:
        bones: ``frame["bones"]`` from MocapReceiver / ReplayMocapReceiver.

    Returns:
        ``{bone_name: (parent_name_or_None, (x, y, z))}`` with offsets in
        right-handed Y-up meters.  The offset of the bone directly below the
        stream root (``Hips``) is its per-frame placement, not a bone length;
        fitting never uses it.
    """
    by_index = {b["bone_index"]: b for b in bones}
    result = {}
    for bone in bones:
        parent = by_index.get(bone["parent_index"])
        px, py, pz = bone["p"]
        result[bone["bone_name"]] = (
            parent["bone_name"] if parent is not None else None,
            (-float(px), float(py), float(pz)),
        )
    return result


def fitted_offsets(bones, body_names, parent_names, reference_offsets):
    """Replace reference offsets with the streamed ones, body by body.

    Args:
        bones: Streamed bones.
        body_names: Asset body names, root first.
        parent_names: Parent body name per body (None for the root).
        reference_offsets: Reference offset per body (right-handed Y-up).

    Returns:
        ``(offsets, fitted_names)``: offsets per body (the root keeps its
        reference entry; bodies absent from the stream keep theirs) and the
        names that were replaced.

    Raises:
        ValueError: If a streamed bone hangs under a different parent than in
            the asset (e.g. a MOVINManV3 stream fitted onto the legacy asset).
    """
    streamed = stream_offsets(bones)
    offsets = [tuple(o) for o in reference_offsets]
    fitted = []
    mismatched = []
    for i, name in enumerate(body_names):
        if i == 0 or name not in streamed:
            continue
        stream_parent, offset = streamed[name]
        if stream_parent != parent_names[i]:
            mismatched.append(f"{name} (asset parent {parent_names[i]}, stream parent {stream_parent})")
            continue
        offsets[i] = offset
        fitted.append(name)
    if mismatched:
        raise ValueError(
            "The streamed skeleton does not match the asset hierarchy: "
            + ", ".join(mismatched[:5])
            + (" ..." if len(mismatched) > 5 else "")
            + ". Check the skeleton preset against the stream."
        )
    return offsets, fitted


def hips_height(body_names, parent_names, offsets):
    """Root height that puts the lowest joint of the T-pose on the ground.

    ``max(0, -min cumulative Y)`` over all joints, rounded to 0.1 mm -- the
    rule the packaged MJCF was generated with.
    """
    index = {name: i for i, name in enumerate(body_names)}
    height = [0.0] * len(body_names)
    lowest = 0.0
    for i, name in enumerate(body_names):
        if i == 0:
            continue
        height[i] = height[index[parent_names[i]]] + offsets[i][1]
        lowest = min(lowest, height[i])
    return round(max(0.0, -lowest), 4)


def leg_height(offsets):
    """T-pose height of the hips above the lowest leg joint.

    Args:
        offsets: ``{bone_name: (parent_name, (x, y, z))}`` right-handed Y-up,
            as returned by ``stream_offsets``.

    Returns:
        The height in meters, or None if a leg chain is missing or hangs
        under unexpected parents.
    """
    lowest = 0.0
    for chain in LEG_CHAINS:
        height = 0.0
        parent = "Hips"
        for name in chain:
            entry = offsets.get(name)
            if entry is None or entry[0] != parent:
                return None
            height += entry[1][1]
            lowest = min(lowest, height)
            parent = name
    return -lowest if lowest < 0.0 else None


@functools.lru_cache(maxsize=None)
def _reference_offsets(mjcf_path):
    root = ET.parse(mjcf_path).getroot()
    return {
        body.get("name"): (parent, _zup_to_yup(_parse_vec(body.get("pos", "0 0 0"))))
        for body, parent in _mjcf_bodies(root)
    }


def body_scale_to_reference(bones, preset="movinman_v3"):
    """How much larger the preset's reference body is than the streamed actor.

    The ratio of T-pose leg heights (hips above the lowest leg joint) of the
    packaged reference skeleton and of the stream.  Multiplying the actor's
    positions by it normalizes the actor to the reference body.

    Args:
        bones: Streamed bones of one frame (``frame["bones"]``).
        preset: Preset name or SkeletonPreset providing the reference body.

    Returns:
        The ratio, or None if the stream lacks a complete leg chain.
    """
    if not isinstance(preset, SkeletonPreset):
        preset = get_preset(preset)
    actor = leg_height(stream_offsets(bones))
    reference = leg_height(_reference_offsets(preset.mjcf_path))
    if actor is None or reference is None:
        return None
    return reference / actor


# --------------------------------------------------------------------------- MJCF

def _yup_to_zup(v):
    return (v[0], -v[2], v[1])


def _zup_to_yup(v):
    return (v[0], v[2], -v[1])


def _parse_vec(text):
    return tuple(float(c) for c in text.split())


def _fmt(value):
    """Format like the MJCF generator: <= 6 decimals, no trailing zeros, no -0."""
    value = float(value)
    if abs(value) < 1e-9:
        return "0"
    text = "{:.6f}".format(value).rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def _fmt_vec(vec):
    return " ".join(_fmt(c) for c in vec)


def _norm(v):
    return math.sqrt(sum(c * c for c in v))


def _mjcf_bodies(root):
    """``[(body_element, parent_name_or_None)]`` in document (DFS) order."""
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF has no <worldbody>")
    result = []

    def walk(element, parent_name):
        for body in element.findall("body"):
            result.append((body, parent_name))
            walk(body, body.get("name"))

    walk(worldbody, None)
    if not result:
        raise ValueError("MJCF has no bodies")
    return result


def fit_skeleton_mjcf(bones, preset="movinman_v3", mjcf_path=None, model_name=None):
    """Return the preset's MJCF skeleton with the actor's streamed proportions.

    Body positions take the streamed offsets, the root height is recomputed so
    the fitted T-pose stands on the ground, and capsule geoms that span a bone
    are stretched to the new bone length (keeping any end gap the asset left
    before a thinner child).  Geom sizes, spheres at leaf bodies, foot boxes,
    joints, and actuators are unchanged: the stream carries no information on
    them.

    Args:
        bones: Streamed bones of one frame (``frame["bones"]``).
        preset: Preset name or SkeletonPreset whose packaged MJCF is fitted.
        mjcf_path: Fit this MJCF instead of the packaged one.
        model_name: Optional new ``<mujoco model=...>`` name.

    Returns:
        The fitted MJCF document as a string.

    Raises:
        ValueError: If the stream's hierarchy does not match the asset.
    """
    if not isinstance(preset, SkeletonPreset):
        preset = get_preset(preset)
    path = mjcf_path or preset.mjcf_path

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(path, parser=parser)
    root = tree.getroot()
    bodies = _mjcf_bodies(root)
    names = [body.get("name") for body, _ in bodies]
    parents = [parent for _, parent in bodies]

    reference = [(0.0, 0.0, 0.0)] + [
        _zup_to_yup(_parse_vec(body.get("pos", "0 0 0"))) for body, _ in bodies[1:]
    ]
    offsets, _ = fitted_offsets(bones, names, parents, reference)

    children = {name: [] for name in names}
    for i, parent in enumerate(parents):
        if parent is not None:
            children[parent].append(i)

    for i, (body, _) in enumerate(bodies):
        if i > 0 and offsets[i] != reference[i]:
            body.set("pos", _fmt_vec(_yup_to_zup(offsets[i])))
    # Raise / lower the root by the change in T-pose leg height, keeping the
    # asset's own ground clearance.
    lift = hips_height(names, parents, offsets) - hips_height(names, parents, reference)
    if lift != 0.0:
        rx, ry, rz = _parse_vec(bodies[0][0].get("pos", "0 0 0"))
        bodies[0][0].set("pos", _fmt_vec((rx, ry, rz + lift)))

    for i, (body, _) in enumerate(bodies):
        geom = body.find("geom")
        if geom is None or geom.get("type") != "capsule" or geom.get("fromto") is None:
            continue
        fromto = _parse_vec(geom.get("fromto"))
        start, end = fromto[:3], fromto[3:]
        if _norm(start) > 1e-9 or _norm(end) < 1e-9:
            continue
        child = _capsule_child(end, [(c, _yup_to_zup(reference[c])) for c in children[names[i]]])
        if child is None or offsets[child] == reference[child]:
            continue
        ref_child = _yup_to_zup(reference[child])
        new_child = _yup_to_zup(offsets[child])
        gap = _norm(ref_child) - _norm(end)  # end cap stopped short of the child
        length = max(_norm(new_child) - gap, _MIN_CAPSULE_LENGTH)
        scale = length / _norm(new_child)
        geom.set("fromto", "0 0 0 " + _fmt_vec(c * scale for c in new_child))

    if model_name is not None:
        root.set("model", model_name)
    return ET.tostring(root, encoding="unicode") + "\n"


def _capsule_child(end, child_positions):
    """Index of the child body a capsule ``0 -> end`` points at, or None."""
    best, best_cos = None, math.cos(math.radians(_CAPSULE_CHILD_MAX_DEG))
    end_len = _norm(end)
    for index, pos in child_positions:
        pos_len = _norm(pos)
        if pos_len < 1e-9 or end_len > pos_len + 1e-6:
            continue
        cos = sum(a * b for a, b in zip(end, pos)) / (end_len * pos_len)
        if cos >= best_cos:
            best, best_cos = index, cos
    return best
