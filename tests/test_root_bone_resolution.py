import importlib.util
import unittest


IDENTITY = (1.0, 0.0, 0.0, 0.0)
# 90 degrees about Y and 30 degrees about X, (w, x, y, z), Unity convention.
ROOT_Q = (0.7071067811865476, 0.0, 0.7071067811865476, 0.0)
HIPS_Q = (0.9659258262890683, 0.25881904510252074, 0.0, 0.0)
ROOT_P = (1.0, 2.0, 3.0)
HIPS_P = (0.1, 0.9, -0.2)


def bone(index, parent, name, p=(0.0, 0.0, 0.0), q=IDENTITY, rq=IDENTITY):
    return {
        "bone_index": index,
        "parent_index": parent,
        "bone_name": name,
        "p": p,
        "rq": rq,
        "q": q,
        "s": (1.0, 1.0, 1.0),
    }


def frame_with_root(root_name, root_p=ROOT_P, root_q=ROOT_Q):
    return [
        bone(0, -1, root_name, p=root_p, q=root_q),
        bone(1, 0, "Hips", p=HIPS_P, q=HIPS_Q),
        bone(2, 1, "Spine", p=(0.0, 0.1, 0.0)),
    ]


def frame_hips_only():
    return [
        bone(0, -1, "Hips", p=HIPS_P, q=HIPS_Q),
        bone(1, 0, "Spine", p=(0.0, 0.1, 0.0)),
    ]


@unittest.skipUnless(
    importlib.util.find_spec("numpy") is not None,
    "numeric dependencies are not installed",
)
class HipsGlobalTransformTests(unittest.TestCase):
    def assert_transform_equal(self, a, b):
        import numpy as np

        np.testing.assert_allclose(a[0], b[0], atol=1e-12)
        np.testing.assert_allclose(a[1], b[1], atol=1e-12)

    def test_root_bone_name_is_irrelevant(self):
        from movin_sdk_python.utils.isaac_lab_utils import hips_global_transform_rh

        reference = hips_global_transform_rh(frame_with_root("Root"))
        for name in ("RootBone", "Armature", "reference"):
            with self.subTest(root_name=name):
                self.assert_transform_equal(
                    hips_global_transform_rh(frame_with_root(name)), reference
                )

    def test_root_transform_is_composed_onto_hips(self):
        import numpy as np

        from movin_sdk_python.utils.isaac_lab_utils import (
            _convert_bone,
            hips_global_transform_rh,
        )
        from movin_sdk_python.utils.quat_utils import quat_mul, rotate_vec_by_quat

        bones = frame_with_root("RootBone")
        p_root, q_root = _convert_bone(bones[0])
        p_hips, q_hips = _convert_bone(bones[1])
        expected = (
            p_root + rotate_vec_by_quat(p_hips, q_root),
            quat_mul(q_root, q_hips),
        )
        self.assert_transform_equal(hips_global_transform_rh(bones), expected)

        # And it is not simply the local Hips pose.
        hips_only = hips_global_transform_rh(frame_hips_only())
        self.assertFalse(np.allclose(hips_only[0], expected[0]))

    def test_identity_root_matches_hips_only_stream(self):
        # MOVINManV3 live streams send an identity "RootBone" above a global
        # "Hips"; that must be indistinguishable from a lone global Hips.
        from movin_sdk_python.utils.isaac_lab_utils import hips_global_transform_rh

        self.assert_transform_equal(
            hips_global_transform_rh(
                frame_with_root("RootBone", root_p=(0.0, 0.0, 0.0), root_q=IDENTITY)
            ),
            hips_global_transform_rh(frame_hips_only()),
        )

    def test_deeper_ancestor_chain_is_composed(self):
        from movin_sdk_python.utils.isaac_lab_utils import (
            _convert_bone,
            hips_global_transform_rh,
        )
        from movin_sdk_python.utils.quat_utils import quat_mul, rotate_vec_by_quat

        bones = [
            bone(0, -1, "Root", p=(0.5, 0.0, 0.0), q=HIPS_Q),
            bone(1, 0, "RootBone", p=ROOT_P, q=ROOT_Q),
            bone(2, 1, "Hips", p=HIPS_P, q=HIPS_Q),
        ]
        p0, q0 = _convert_bone(bones[0])
        p1, q1 = _convert_bone(bones[1])
        p2, q2 = _convert_bone(bones[2])
        p12 = p1 + rotate_vec_by_quat(p2, q1)
        q12 = quat_mul(q1, q2)
        expected = (p0 + rotate_vec_by_quat(p12, q0), quat_mul(q0, q12))
        self.assert_transform_equal(hips_global_transform_rh(bones), expected)

    def test_missing_hips_falls_back_to_stream_root(self):
        from movin_sdk_python.utils.isaac_lab_utils import (
            _convert_bone,
            find_stream_root_bone,
            hips_global_transform_rh,
        )

        bones = [bone(0, -1, "Pelvis", p=ROOT_P, q=ROOT_Q), bone(1, 0, "Spine")]
        self.assertIs(find_stream_root_bone(bones), bones[0])
        self.assert_transform_equal(
            hips_global_transform_rh(bones), _convert_bone(bones[0])
        )
        self.assertIsNone(hips_global_transform_rh([]))
        self.assertIsNone(find_stream_root_bone([]))


@unittest.skipUnless(
    importlib.util.find_spec("numpy") is not None,
    "numeric dependencies are not installed",
)
class IsaacLabRootTests(unittest.TestCase):
    def test_isaaclab_root_pose_ignores_root_bone_name(self):
        import numpy as np

        from movin_sdk_python.utils.isaac_lab_utils import (
            process_movin_bones_for_isaaclab,
        )

        ref_pos, ref_quat, ref_dof = process_movin_bones_for_isaaclab(
            frame_with_root("Root"), skeleton_bone_names=("Spine",)
        )
        pos, quat, dof = process_movin_bones_for_isaaclab(
            frame_with_root("RootBone"), skeleton_bone_names=("Spine",)
        )
        np.testing.assert_allclose(pos, ref_pos)
        np.testing.assert_allclose(quat, ref_quat)
        np.testing.assert_allclose(dof, ref_dof)

        hips_pos, _, _ = process_movin_bones_for_isaaclab(
            frame_hips_only(), skeleton_bone_names=("Spine",)
        )
        self.assertFalse(np.allclose(pos, hips_pos))

    def test_mesh_root_pose_ignores_root_bone_name(self):
        import numpy as np

        from movin_sdk_python.utils.movinman_mesh_utils import (
            extract_movin_local_quats_yup,
        )

        body_names = ("Hips", "Spine")
        ref_quats, ref_root = extract_movin_local_quats_yup(
            frame_with_root("Root"), skeleton_body_names=body_names
        )
        quats, root = extract_movin_local_quats_yup(
            frame_with_root("RootBone"), skeleton_body_names=body_names
        )
        np.testing.assert_allclose(quats, ref_quats)
        np.testing.assert_allclose(root, ref_root)

        _, hips_root = extract_movin_local_quats_yup(
            frame_hips_only(), skeleton_body_names=body_names
        )
        self.assertFalse(np.allclose(root, hips_root))


if __name__ == "__main__":
    unittest.main()
