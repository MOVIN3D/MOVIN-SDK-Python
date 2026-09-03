import importlib.util
import unittest


VIEWER_DEPENDENCIES_AVAILABLE = all(
    importlib.util.find_spec(name) is not None
    for name in ("mujoco", "numpy", "loop_rate_limiters")
)


@unittest.skipUnless(
    VIEWER_DEPENDENCIES_AVAILABLE,
    "viewer extra is not installed",
)
class MocapViewerTests(unittest.TestCase):
    @staticmethod
    def frame():
        identity = (1.0, 0.0, 0.0, 0.0)
        return {
            "frame_idx": 1,
            "bones": [
                {
                    "bone_index": 0,
                    "parent_index": -1,
                    "bone_name": "Hips",
                    "p": (0.0, 0.0, 0.0),
                    "rq": identity,
                    "q": identity,
                    "s": (1.0, 1.0, 1.0),
                },
                {
                    "bone_index": 1,
                    "parent_index": 0,
                    "bone_name": "Spine",
                    "p": (0.0, 1.0, 0.0),
                    "rq": identity,
                    "q": identity,
                    "s": (1.0, 1.0, 1.0),
                },
            ],
        }

    def test_raw_frame_builds_joint_positions_and_edges(self):
        import numpy as np

        from movin_sdk_python.viewer.mocap_viewer import MocapViewer

        positions, edges, bones = MocapViewer._stick_geometry(self.frame())

        self.assertEqual(len(positions), 2)
        self.assertEqual(len(edges), 1)
        np.testing.assert_allclose(positions[1], [0.0, 0.0, 1.0])
        np.testing.assert_array_equal(
            MocapViewer._camera_target(positions, bones), positions[0]
        )

    def test_invalid_frame_is_rejected(self):
        from movin_sdk_python.viewer.mocap_viewer import MocapViewer

        with self.assertRaisesRegex(TypeError, "receiver frame"):
            MocapViewer._stick_geometry([])
        with self.assertRaisesRegex(ValueError, "at least one bone"):
            MocapViewer._stick_geometry({"bones": []})

    def test_scene_style_matches_robot_viewer(self):
        import mujoco as mj
        import numpy as np

        from movin_sdk_python.viewer.mocap_viewer import _EMPTY_SCENE_XML
        from movin_sdk_python.viewer.mujoco_viewer import ROBOT_XML_DICT

        stick_model = mj.MjModel.from_xml_string(_EMPTY_SCENE_XML)
        robot_model = mj.MjModel.from_xml_path(str(ROBOT_XML_DICT["unitree_g1"]))

        np.testing.assert_allclose(
            stick_model.vis.headlight.ambient,
            robot_model.vis.headlight.ambient,
        )
        np.testing.assert_allclose(
            stick_model.vis.headlight.diffuse,
            robot_model.vis.headlight.diffuse,
        )
        np.testing.assert_allclose(
            stick_model.vis.headlight.specular,
            robot_model.vis.headlight.specular,
        )
        np.testing.assert_allclose(
            stick_model.vis.rgba.haze,
            robot_model.vis.rgba.haze,
        )
        np.testing.assert_allclose(stick_model.stat.center, robot_model.stat.center)
        self.assertEqual(stick_model.stat.extent, robot_model.stat.extent)
        self.assertEqual(
            stick_model.vis.global_.azimuth,
            robot_model.vis.global_.azimuth,
        )
        self.assertEqual(
            stick_model.vis.global_.elevation,
            robot_model.vis.global_.elevation,
        )
        self.assertEqual(
            stick_model.vis.global_.offwidth,
            robot_model.vis.global_.offwidth,
        )
        self.assertEqual(
            stick_model.vis.global_.offheight,
            robot_model.vis.global_.offheight,
        )
        self.assertEqual(stick_model.ntex, robot_model.ntex)
        self.assertEqual(stick_model.nmat, robot_model.nmat)
        self.assertGreaterEqual(stick_model.geom("floor").id, 0)

    def test_step_populates_mujoco_custom_scene(self):
        import contextlib

        import mujoco as mj
        import numpy as np

        from movin_sdk_python.viewer.mocap_viewer import (
            MocapViewer,
            _EMPTY_SCENE_XML,
        )

        class FakeHandle:
            def __init__(self, scene):
                self.user_scn = scene
                self.cam = type("Camera", (), {"lookat": np.zeros(3)})()
                self.sync_count = 0

            def lock(self):
                return contextlib.nullcontext()

            def sync(self):
                self.sync_count += 1

            def is_running(self):
                return True

        model = mj.MjModel.from_xml_string(_EMPTY_SCENE_XML)
        mocap_viewer = MocapViewer.__new__(MocapViewer)
        mocap_viewer.joint_radius = 0.025
        mocap_viewer.bone_radius = 0.012
        mocap_viewer.joint_color = np.ones(4, dtype=np.float32)
        mocap_viewer.bone_color = np.ones(4, dtype=np.float32)
        mocap_viewer.viewer = FakeHandle(mj.MjvScene(model, maxgeom=10))

        running = mocap_viewer.step(self.frame(), rate_limit=False)

        self.assertTrue(running)
        self.assertEqual(mocap_viewer.viewer.user_scn.ngeom, 3)
        self.assertEqual(mocap_viewer.viewer.sync_count, 1)


if __name__ == "__main__":
    unittest.main()
