"""Fitting the packaged skeleton/mesh assets to streamed body proportions.

Streams are synthesized from the assets themselves: one bone per asset body
under a stream root (as MOVIN Studio sends them), with the parent-local
offsets optionally scaled per bone to mimic an actor's calibration.
"""

import importlib.util
import unittest
import xml.etree.ElementTree as ET

from movin_sdk_python.utils.skeleton_fitting import (
    LEG_BONES,
    body_scale_to_reference,
    fit_skeleton_mjcf,
    hips_height,
    stream_offsets,
)
from movin_sdk_python.utils.skeleton_presets import PRESETS, get_preset

HAS_NUMPY = importlib.util.find_spec("numpy") is not None
HAS_MUJOCO = HAS_NUMPY and importlib.util.find_spec("mujoco") is not None
HAS_RETARGETING = HAS_MUJOCO and all(
    importlib.util.find_spec(name) is not None for name in ("scipy", "mink")
)

# Calibration-like per-bone length ratios.
ACTOR_SCALE = {
    "RightForeArm": 0.84, "LeftForeArm": 0.84, "RightHand": 0.84, "LeftHand": 0.84,
    "RightLeg": 0.872, "LeftLeg": 0.872, "RightFoot": 0.872, "LeftFoot": 0.872,
    "RightUpLeg": 1.131, "LeftUpLeg": 1.131, "Spine": 0.93, "Neck": 1.2,
}


def mjcf_skeleton(path):
    """[(name, parent_name, pos_zup)] of every body, document order."""
    bodies = []

    def walk(element, parent):
        for body in element.findall("body"):
            pos = tuple(float(c) for c in body.get("pos", "0 0 0").split())
            bodies.append((body.get("name"), parent, pos))
            walk(body, body.get("name"))

    walk(ET.parse(path).getroot().find("worldbody"), None)
    return bodies


def mjcf_skeleton_yup(preset):
    """[(name, parent_name, offset)] of the preset's MJCF, right-handed Y-up."""
    return [(n, p, (x, z, -y)) for n, p, (x, y, z) in mjcf_skeleton(preset.mjcf_path)]


def mesh_skeleton_yup(model):
    """[(name, parent_name, offset)] of a MOVINMeshModel's reference skeleton."""
    names = model.bone_names
    offsets = model._reference[3]
    return [
        (name, None if parent < 0 else names[parent], tuple(offsets[i]))
        for i, (name, parent) in enumerate(zip(names, model._bone_parents))
    ]


def synth_bones(skeleton, scale=None):
    """A streamed frame (bones list) for a skeleton, offsets scaled per bone.

    ``skeleton`` is a SkeletonPreset (its MJCF) or a ``*_skeleton_yup`` list.
    """
    if hasattr(skeleton, "mjcf_path"):
        skeleton = mjcf_skeleton_yup(skeleton)
    scale = scale or {}
    index = {name: i + 1 for i, (name, _, _) in enumerate(skeleton)}
    bones = [{"bone_index": 0, "parent_index": -1, "bone_name": "RootBone", "p": (0.0, 0.0, 0.0),
              "rq": (1.0, 0.0, 0.0, 0.0), "q": (1.0, 0.0, 0.0, 0.0), "s": (1.0, 1.0, 1.0)}]
    for name, parent, (x, y, z) in skeleton:
        k = scale.get(name, 1.0)
        # Right-handed Y-up offset (x, y, z) -> Unity p (-x, y, z)
        p = (0.3, 0.9, -0.1) if parent is None else (-x * k, y * k, z * k)
        bones.append({"bone_index": index[name], "parent_index": index.get(parent, 0),
                      "bone_name": name, "p": p, "rq": (1.0, 0.0, 0.0, 0.0),
                      "q": (1.0, 0.0, 0.0, 0.0), "s": (1.0, 1.0, 1.0)})
    return bones


def numeric_attrs(xml_text_or_path):
    """{(tag, name, attr): numbers} for every numeric body/geom attribute."""
    if xml_text_or_path.lstrip().startswith("<"):
        root = ET.fromstring(xml_text_or_path)
    else:
        root = ET.parse(xml_text_or_path).getroot()
    result = {}
    for element in root.iter():
        if element.tag not in ("body", "geom"):
            continue
        for attr in ("pos", "fromto", "size"):
            if element.get(attr) is not None:
                result[(element.tag, element.get("name"), attr)] = tuple(
                    float(c) for c in element.get(attr).split()
                )
    return result


class StreamOffsetTests(unittest.TestCase):
    def test_offsets_are_right_handed_with_parent_names(self):
        bones = [
            {"bone_index": 0, "parent_index": -1, "bone_name": "RootBone", "p": (0.0, 0.0, 0.0)},
            {"bone_index": 1, "parent_index": 0, "bone_name": "Hips", "p": (1.0, 2.0, 3.0)},
            {"bone_index": 2, "parent_index": 1, "bone_name": "Spine", "p": (0.5, 0.1, -0.2)},
        ]
        offsets = stream_offsets(bones)
        self.assertEqual(offsets["RootBone"], (None, (-0.0, 0.0, 0.0)))
        self.assertEqual(offsets["Spine"], ("Hips", (-0.5, 0.1, -0.2)))

    def test_hips_height_is_the_lowest_joint_depth(self):
        names = ["Hips", "UpLeg", "Leg", "Spine"]
        parents = [None, "Hips", "UpLeg", "Hips"]
        offsets = [(0, 0, 0), (0.1, -0.05, 0), (0, -0.4, 0), (0, 0.1, 0)]
        self.assertAlmostEqual(hips_height(names, parents, offsets), 0.45)


class MjcfFittingTests(unittest.TestCase):
    def assert_same_numbers(self, a, b):
        self.assertEqual(a.keys(), b.keys())
        for key in a:
            for x, y in zip(a[key], b[key]):
                self.assertAlmostEqual(x, y, places=6, msg=key)

    def test_reference_proportions_leave_the_asset_unchanged(self):
        for preset in PRESETS.values():
            original = numeric_attrs(preset.mjcf_path)
            self.assert_same_numbers(numeric_attrs(fit_skeleton_mjcf([], preset)), original)
            self.assert_same_numbers(
                numeric_attrs(fit_skeleton_mjcf(synth_bones(preset), preset)), original
            )

    def test_bodies_take_the_streamed_offsets(self):
        preset = get_preset("movinman_v3")
        original = {n: (p, pos) for n, p, pos in mjcf_skeleton(preset.mjcf_path)}
        fitted_xml = fit_skeleton_mjcf(synth_bones(preset, ACTOR_SCALE), preset)
        fitted = numeric_attrs(fitted_xml)
        for name, (parent, pos) in original.items():
            if parent is None:
                continue
            expected = tuple(c * ACTOR_SCALE.get(name, 1.0) for c in pos)
            for a, e in zip(fitted[("body", name, "pos")], expected):
                self.assertAlmostEqual(a, e, places=5, msg=name)

    def test_root_height_follows_the_legs(self):
        preset = get_preset("movinman_v3")
        names = [n for n, _, _ in mjcf_skeleton(preset.mjcf_path)]
        parents = [p for _, p, _ in mjcf_skeleton(preset.mjcf_path)]
        offsets = [(0.0, 0.0, 0.0)] + [
            (x * ACTOR_SCALE.get(n, 1.0), z * ACTOR_SCALE.get(n, 1.0), -y * ACTOR_SCALE.get(n, 1.0))
            for n, _, (x, y, z) in mjcf_skeleton(preset.mjcf_path)[1:]
        ]
        fitted = numeric_attrs(fit_skeleton_mjcf(synth_bones(preset, ACTOR_SCALE), preset))
        self.assertAlmostEqual(
            fitted[("body", "Hips", "pos")][2], hips_height(names, parents, offsets), places=4
        )
        self.assertLess(fitted[("body", "Hips", "pos")][2], preset.default_hips_height)

    def test_capsules_stretch_with_their_bone(self):
        preset = get_preset("movinman_v3")
        original = numeric_attrs(preset.mjcf_path)
        fitted = numeric_attrs(fit_skeleton_mjcf(synth_bones(preset, ACTOR_SCALE), preset))
        # RightArm's capsule spans to RightForeArm, which is 0.84x as far away.
        ref = original[("geom", "RightArm", "fromto")][3:]
        new = fitted[("geom", "RightArm", "fromto")][3:]
        for a, b in zip(new, ref):
            self.assertAlmostEqual(a, b * 0.84, places=5)
        # Sizes and leaf spheres are untouched.
        self.assertEqual(fitted[("geom", "RightArm", "size")], original[("geom", "RightArm", "size")])
        self.assertEqual(fitted[("geom", "Head", "pos")], original[("geom", "Head", "pos")])

    def test_mismatched_hierarchy_is_rejected(self):
        v3_stream = synth_bones(get_preset("movinman_v3"))
        with self.assertRaises(ValueError):
            fit_skeleton_mjcf(v3_stream, "movinman")

    @unittest.skipUnless(HAS_MUJOCO, "mujoco is not installed")
    def test_fitted_mjcf_zero_pose_is_the_actor_tpose(self):
        import mujoco
        import numpy as np

        preset = get_preset("movinman_v3")
        bones = synth_bones(preset, ACTOR_SCALE)
        model = mujoco.MjModel.from_xml_string(fit_skeleton_mjcf(bones, preset))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        xpos = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i): data.xpos[i]
                for i in range(1, model.nbody)}
        by_index = {b["bone_index"]: b for b in bones}
        for bone in bones[2:]:
            parent = by_index[bone["parent_index"]]["bone_name"]
            px, py, pz = bone["p"]
            expected = xpos[parent] + np.array([-px, -pz, py])
            np.testing.assert_allclose(xpos[bone["bone_name"]], expected, atol=1e-5)
        self.assertAlmostEqual(min(p[2] for p in xpos.values()), 0.0, places=3)


@unittest.skipUnless(HAS_NUMPY, "numeric dependencies are not installed")
class MeshFittingTests(unittest.TestCase):
    def setUp(self):
        from movin_sdk_python.utils.movinman_mesh_utils import MOVINMeshModel

        self.Model = MOVINMeshModel

    def test_reference_proportions_are_a_no_op(self):
        import numpy as np

        for preset in PRESETS.values():
            reference = self.Model.from_preset(preset.name)
            fitted = self.Model.from_preset(preset.name).fit_to_bones(
                synth_bones(mesh_skeleton_yup(reference))
            )
            self.assertTrue(fitted.is_fitted)
            np.testing.assert_allclose(fitted._vertices, reference._vertices, atol=1e-3)

    def test_fitted_skeleton_uses_streamed_offsets_and_keeps_ground(self):
        import numpy as np

        for preset in PRESETS.values():
            reference = self.Model.from_preset(preset.name)
            model = self.Model.from_preset(preset.name).fit_to_bones(
                synth_bones(mesh_skeleton_yup(reference), ACTOR_SCALE)
            )
            names = model.bone_names
            for i, name in enumerate(names[1:], start=1):
                np.testing.assert_allclose(
                    model._bone_offsets[i],
                    reference._bone_offsets[i] * ACTOR_SCALE.get(name, 1.0),
                    atol=1e-6, err_msg=name,
                )
            # T-pose: LBS reproduces the rebound vertices, sole stays at reference height.
            identity = np.tile([1.0, 0.0, 0.0, 0.0], (len(names), 1))
            root = -model._inverse_bind_matrices[0, :3, 3]
            posed = model.pose_vertices(identity, root)
            bind = model._vertices.astype(np.float64)
            np.testing.assert_allclose(posed, bind[:, [0, 1, 2]] / 100.0, atol=1e-4)
            ref_root = -reference._inverse_bind_matrices[0, :3, 3]
            ref_posed = reference.pose_vertices(identity, ref_root)
            self.assertAlmostEqual(posed[:, 2].min(), ref_posed[:, 2].min(), delta=0.01)
            self.assertLess(posed[:, 2].max(), ref_posed[:, 2].max())

    def test_pose_from_bones_refits_only_on_change_and_reset_restores(self):
        import numpy as np

        model = self.Model.from_preset("movinman_v3")
        skeleton = mesh_skeleton_yup(model)
        reference_vertices = model._vertices.copy()
        model.pose_from_bones(synth_bones(skeleton, ACTOR_SCALE))
        fitted_vertices = model._vertices
        model.pose_from_bones(synth_bones(skeleton, ACTOR_SCALE))
        self.assertIs(model._vertices, fitted_vertices)  # unchanged offsets: no refit
        model.pose_from_bones(synth_bones(skeleton))
        np.testing.assert_allclose(model._vertices, reference_vertices, atol=1e-3)
        model.pose_from_bones(synth_bones(skeleton, ACTOR_SCALE), fit=False)
        model.reset_fit()
        self.assertFalse(model.is_fitted)
        self.assertIs(model._vertices, model._reference[0])

    def test_mismatched_hierarchy_is_rejected(self):
        model = self.Model.from_preset("movinman")
        with self.assertRaises(ValueError):
            model.fit_to_bones(synth_bones(get_preset("movinman_v3")))


class BodyScaleTests(unittest.TestCase):
    def test_reference_body_has_scale_one(self):
        for preset in PRESETS.values():
            self.assertAlmostEqual(body_scale_to_reference(synth_bones(preset), preset), 1.0)

    def test_shorter_legs_scale_up(self):
        preset = get_preset("movinman_v3")
        bones = synth_bones(preset, {name: 0.8 for name in LEG_BONES})
        self.assertAlmostEqual(body_scale_to_reference(bones, preset), 1.25)

    def test_incomplete_legs_give_none(self):
        preset = get_preset("movinman_v3")
        bones = [b for b in synth_bones(preset) if b["bone_name"] != "LeftFoot"]
        self.assertIsNone(body_scale_to_reference(bones, preset))


@unittest.skipUnless(HAS_RETARGETING, "retargeting dependencies are not installed")
class RetargeterBodyScaleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import json
        from pathlib import Path

        from movin_sdk_python.retargeter.retargeter import Retargeter

        cls.Retargeter = Retargeter
        config = Path(__file__).parents[1] / "movin_sdk_python" / "retargeter" / "ik_configs" / "movinman_v3_to_g1.json"
        cls.base = json.loads(config.read_text(encoding="utf-8"))["human_scale_table"]
        cls.preset = get_preset("movinman_v3")

    def assert_table(self, retargeter, factor):
        for name, value in self.base.items():
            self.assertAlmostEqual(retargeter.human_scale_table[name], value * factor, places=9, msg=name)

    def test_auto_is_the_default_and_follows_the_stream(self):
        retargeter = self.Retargeter("unitree_g1")
        self.assertEqual(retargeter.human_height, "auto")
        self.assert_table(retargeter, 1.0)  # reference body until a frame arrives
        retargeter.process_mocap_frame(synth_bones(self.preset, {n: 0.8 for n in LEG_BONES}))
        self.assertAlmostEqual(retargeter.body_scale, 1.25)
        self.assert_table(retargeter, 1.25)
        # Recalibrated actor: the scale follows the new proportions.
        retargeter.process_mocap_frame(synth_bones(self.preset))
        self.assertAlmostEqual(retargeter.body_scale, 1.0)
        self.assert_table(retargeter, 1.0)

    def test_a_number_is_the_actors_height_and_turns_auto_off(self):
        retargeter = self.Retargeter("unitree_g1", human_height=1.6)
        retargeter.process_mocap_frame(synth_bones(self.preset, {n: 0.8 for n in LEG_BONES}))
        # A shorter actor than the 1.8 m reference gets larger targets; the
        # streamed body no longer changes the scale.
        self.assertAlmostEqual(retargeter.body_scale, 1.8 / 1.6)
        self.assert_table(retargeter, 1.8 / 1.6)

    def test_reference_height_is_the_reference_body(self):
        retargeter = self.Retargeter("unitree_g1", human_height=1.8)
        self.assertAlmostEqual(retargeter.body_scale, 1.0)
        self.assert_table(retargeter, 1.0)

    def test_invalid_heights_are_rejected(self):
        for value in ("tall", 0, -1.7):
            with self.assertRaises(ValueError):
                self.Retargeter("unitree_g1", human_height=value)


if __name__ == "__main__":
    unittest.main()
