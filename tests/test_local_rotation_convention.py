"""The streamed local rotation ``q`` is applied as-is; ``rq`` is never removed.

MOVIN Studio streams ``p`` as the joint offset in the parent's frame and ``q``
as the full local rotation (rest rotation included).  The skeleton assets the
SDK drives are generated from a T-pose whose local rotations are all identity,
so ``q`` maps onto them directly.  Removing ``rq`` (the old ``conj(rq) * q``)
would straighten the fingers of streams from the earlier MOVINManV3 rig, whose
finger bones carry non-identity rest rotations (Thumb1 ~60 deg about Y); the
identity-local rig streams identity ``rq``, where removal is a no-op.
"""

import importlib.util
import unittest

IDENTITY = (1.0, 0.0, 0.0, 0.0)
# 60 degrees about Y, Unity (w, x, y, z): the MOVINManV3 RightHandThumb1 rest rotation.
REST_Q = (0.8660254037844387, 0.0, 0.5, 0.0)
# An arbitrary full local rotation (unit quaternion).
LOCAL_Q = (0.8446231986207332, 0.19134171618254492, 0.46193976625564337, 0.19134171618254492)
ROOT_Q = (0.9659258262890683, 0.25881904510252074, 0.0, 0.0)
ROOT_P = (0.1, 0.9, -0.2)
THUMB_P = (0.041726, -0.020077, 0.027704)


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


def thumb_frame(rq=REST_Q, q=LOCAL_Q):
    return [
        bone(0, -1, "Hips", p=ROOT_P, q=ROOT_Q),
        bone(1, 0, "RightHandThumb1", p=THUMB_P, q=q, rq=rq),
    ]


@unittest.skipUnless(
    importlib.util.find_spec("numpy") is not None,
    "numeric dependencies are not installed",
)
class LocalRotationConventionTests(unittest.TestCase):
    def test_convert_bone_keeps_the_full_local_rotation(self):
        import numpy as np

        from movin_sdk_python.utils.isaac_lab_utils import _convert_bone
        from movin_sdk_python.utils.quat_utils import (
            unity_to_opengl_quat,
            unity_to_opengl_vec,
        )

        p, q = _convert_bone(thumb_frame()[1])
        np.testing.assert_allclose(p, unity_to_opengl_vec(np.array(THUMB_P)))
        np.testing.assert_allclose(q, unity_to_opengl_quat(np.array(LOCAL_Q)), atol=1e-12)

        # The result must not depend on rq at all.
        _, q_identity_rest = _convert_bone(thumb_frame(rq=IDENTITY)[1])
        np.testing.assert_allclose(q, q_identity_rest, atol=1e-12)

    def test_isaaclab_dofs_are_the_exp_map_of_the_full_local_rotation(self):
        import numpy as np

        from movin_sdk_python.utils.isaac_lab_utils import (
            process_movin_bones_for_isaaclab,
            quat_to_exp_map,
            yup_to_zup_quat,
        )
        from movin_sdk_python.utils.quat_utils import unity_to_opengl_quat

        _, _, dof = process_movin_bones_for_isaaclab(
            thumb_frame(), skeleton_bone_names=("RightHandThumb1",)
        )
        expected = quat_to_exp_map(yup_to_zup_quat(unity_to_opengl_quat(np.array(LOCAL_Q))))
        np.testing.assert_allclose(dof, expected, atol=1e-12)

        # In the rest pose (q == rq) the finger hinges are NOT zero: the MJCF's
        # zero pose is the identity-local-rotation pose, not the T-pose.
        _, _, rest_dof = process_movin_bones_for_isaaclab(
            thumb_frame(q=REST_Q), skeleton_bone_names=("RightHandThumb1",)
        )
        self.assertAlmostEqual(np.linalg.norm(rest_dof), np.radians(60.0), places=9)

    def test_forward_kinematics_ignores_the_rest_quaternion(self):
        import numpy as np

        from movin_sdk_python.utils.fk_utils import compute_forward_kinematics
        from movin_sdk_python.utils.quat_utils import (
            quat_mul,
            rotate_vec_by_quat,
            unity_to_opengl_quat,
            unity_to_opengl_vec,
        )

        result = compute_forward_kinematics(thumb_frame())
        same_without_rest = compute_forward_kinematics(thumb_frame(rq=IDENTITY))
        for name in ("Hips", "RightHandThumb1"):
            np.testing.assert_allclose(result[name][0], same_without_rest[name][0], atol=1e-12)
            np.testing.assert_allclose(result[name][1], same_without_rest[name][1], atol=1e-12)

        # Child global = parent * T(p) * R(q) with the FULL local q, then Y-up -> Z-up.
        p_root = unity_to_opengl_vec(np.array(ROOT_P))
        q_root = unity_to_opengl_quat(np.array(ROOT_Q))
        p_child = unity_to_opengl_vec(np.array(THUMB_P))
        q_child = unity_to_opengl_quat(np.array(LOCAL_Q))
        global_pos = p_root + rotate_vec_by_quat(p_child, q_root)
        global_rot = quat_mul(q_root, q_child)
        to_zup = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
        q_x90 = np.array([0.7071067811865476, 0.7071067811865476, 0.0, 0.0])
        np.testing.assert_allclose(result["RightHandThumb1"][0], global_pos @ to_zup.T, atol=1e-12)
        np.testing.assert_allclose(result["RightHandThumb1"][1], quat_mul(q_x90, global_rot), atol=1e-12)

    def test_mesh_local_quats_are_the_full_local_rotation(self):
        import numpy as np

        from movin_sdk_python.utils.movinman_mesh_utils import extract_movin_local_quats_yup
        from movin_sdk_python.utils.quat_utils import unity_to_opengl_quat

        quats, _ = extract_movin_local_quats_yup(
            thumb_frame(), skeleton_body_names=("Hips", "RightHandThumb1")
        )
        np.testing.assert_allclose(quats[1], unity_to_opengl_quat(np.array(LOCAL_Q)), atol=1e-12)


class V3PresetOrderTests(unittest.TestCase):
    FINGERS = ("Thumb", "Index", "Middle", "Ring", "Pinky")

    def test_v3_body_order_follows_studio_exports(self):
        from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_BODY_NAMES

        names = MOVINMAN_V3_BODY_NAMES
        self.assertEqual(len(names), 54)
        self.assertEqual(len(set(names)), 54)
        self.assertEqual(names[0], "Hips")
        for side in ("Right", "Left"):
            start = names.index(f"{side}Hand") + 1
            expected = tuple(
                f"{side}Hand{finger}{segment}"
                for finger in self.FINGERS
                for segment in (1, 2, 3)
            )
            self.assertEqual(names[start:start + 15], expected)

    def test_joint_bones_exclude_the_root(self):
        from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_PRESET

        self.assertEqual(len(MOVINMAN_V3_PRESET.joint_bones), 53)
        self.assertNotIn("Hips", MOVINMAN_V3_PRESET.joint_bones)


if __name__ == "__main__":
    unittest.main()
