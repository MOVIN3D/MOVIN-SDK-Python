"""The skeleton presets' MJCF and mesh assets ship inside the package.

The SDK must not depend on another checkout for its assets: every preset's
files resolve under ``movin_sdk_python/assets`` and match the preset.
"""

import importlib.util
import os
import unittest
import xml.etree.ElementTree as ET

from movin_sdk_python.utils.skeleton_presets import PRESETS

HAS_NUMPY = importlib.util.find_spec("numpy") is not None


def mjcf_body_names(path):
    """Names of every <body> in document (depth-first) order."""
    return tuple(body.get("name") for body in ET.parse(path).getroot().iter("body"))


class PackagedAssetTests(unittest.TestCase):
    def test_every_preset_ships_its_assets(self):
        for preset in PRESETS.values():
            self.assertTrue(os.path.isfile(preset.mjcf_path), preset.mjcf_path)
            self.assertTrue(os.path.isfile(preset.mesh_npz_path), preset.mesh_npz_path)
            self.assertIn(os.path.join("movin_sdk_python", "assets"), preset.mjcf_path)

    def test_mjcf_body_order_matches_the_preset(self):
        for preset in PRESETS.values():
            self.assertEqual(mjcf_body_names(preset.mjcf_path), preset.body_names, preset.name)

    @unittest.skipUnless(HAS_NUMPY, "numeric dependencies are not installed")
    def test_meshes_load_for_their_presets(self):
        from movin_sdk_python.utils.movinman_mesh_utils import MOVINMeshModel

        for preset in PRESETS.values():
            model = MOVINMeshModel.from_preset(preset.name)
            self.assertEqual(tuple(model.bone_names), preset.body_names)
            self.assertGreater(model.num_vertices, 1000)

    @unittest.skipUnless(HAS_NUMPY, "numeric dependencies are not installed")
    def test_default_mesh_is_the_packaged_legacy_mesh(self):
        from movin_sdk_python.utils.movinman_mesh_utils import MOVINMeshModel

        model = MOVINMeshModel()
        self.assertEqual(tuple(model.bone_names), PRESETS["movinman"].body_names)


if __name__ == "__main__":
    unittest.main()
