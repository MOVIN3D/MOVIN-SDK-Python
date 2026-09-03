import importlib.util
import json
import unittest
from pathlib import Path

from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_BODY_NAMES


EXPECTED_V3_MAPPING = {
    "Hips": "pelvis",
    "Spine3": "torso_link",
    "LeftUpLeg": "left_hip_roll_link",
    "LeftLeg": "left_knee_link",
    "LeftToeBase": "left_toe_link",
    "RightUpLeg": "right_hip_roll_link",
    "RightLeg": "right_knee_link",
    "RightToeBase": "right_toe_link",
    "LeftArm": "left_shoulder_yaw_link",
    "LeftForeArm": "left_elbow_link",
    "LeftHand": "left_wrist_yaw_link",
    "RightArm": "right_shoulder_yaw_link",
    "RightForeArm": "right_elbow_link",
    "RightHand": "right_wrist_yaw_link",
}


class RetargeterV3MappingTests(unittest.TestCase):
    @staticmethod
    def config():
        path = (
            Path(__file__).parents[1]
            / "movin_sdk_python"
            / "retargeter"
            / "ik_configs"
            / "movinman_v3_to_g1.json"
        )
        return json.loads(path.read_text(encoding="utf-8"))

    def test_v3_mapping_uses_expected_human_and_robot_bones(self):
        config = self.config()
        v3_bones = set(MOVINMAN_V3_BODY_NAMES)

        for table_name in ("ik_match_table1", "ik_match_table2"):
            table = config[table_name]
            actual = {entry[0]: robot_bone for robot_bone, entry in table.items()}

            self.assertEqual(actual, EXPECTED_V3_MAPPING)
            self.assertTrue(set(actual).issubset(v3_bones))

        self.assertEqual(
            set(config["human_scale_table"]),
            set(EXPECTED_V3_MAPPING),
        )

    @unittest.skipUnless(
        all(
            importlib.util.find_spec(name) is not None
            for name in ("mujoco", "mink", "numpy", "scipy")
        ),
        "retargeting extra is not installed",
    )
    def test_retargeter_defaults_to_v3_mapping(self):
        from movin_sdk_python import Retargeter

        retargeter = Retargeter()

        self.assertEqual(retargeter.source_preset, "movinman_v3")
        self.assertIn("Spine3", retargeter.get_required_bones())
        self.assertNotIn("Spine1", retargeter.get_required_bones())


if __name__ == "__main__":
    unittest.main()
