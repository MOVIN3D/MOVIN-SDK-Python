"""MJCF color resolution used to fix materials of converted robot USDs."""

import importlib.util
import os
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from movin_sdk_python.utils.usd_utils import mjcf_mesh_colors

ROBOT_DIR = Path(__file__).parents[1] / "movin_sdk_python" / "retargeter" / "assets" / "unitree_g1"
G1 = ROBOT_DIR / "g1_mocap_29dof.xml"
G1_HANDS = ROBOT_DIR / "g1_mocap_29dof_with_hands.xml"
BLACK = (0.2, 0.2, 0.2, 1.0)
METAL = (0.7, 0.7, 0.7, 1.0)
HAS_MUJOCO = importlib.util.find_spec("mujoco") is not None


def direct_rgba_colors(path):
    """The previous behavior: only rgba written on the geom itself."""
    colors = {}
    for geom in ET.parse(path).getroot().iter("geom"):
        if geom.get("mesh") and geom.get("rgba"):
            colors[geom.get("mesh")] = tuple(float(v) for v in geom.get("rgba").split())
    return colors


class MjcfMeshColorTests(unittest.TestCase):
    def test_directly_colored_mjcf_is_unchanged(self):
        self.assertEqual(mjcf_mesh_colors(G1), direct_rgba_colors(G1))

    def test_material_and_class_colored_mjcf(self):
        colors = mjcf_mesh_colors(G1_HANDS)
        self.assertEqual(direct_rgba_colors(G1_HANDS), {})  # why it used to render white
        visual_meshes = {
            geom.get("mesh")
            for geom in ET.parse(G1_HANDS).getroot().iter("geom")
            if geom.get("class") == "visual"
        }
        black = {
            geom.get("mesh")
            for geom in ET.parse(G1_HANDS).getroot().iter("geom")
            if geom.get("class") == "visual" and geom.get("material") == "black"
        }
        self.assertEqual(set(colors), visual_meshes)
        self.assertEqual({m for m, c in colors.items() if c == BLACK}, black)
        self.assertEqual({m for m, c in colors.items() if c != BLACK}, visual_meshes - black)
        self.assertEqual(set(colors.values()), {BLACK, METAL})

    def test_inheritance_rules(self):
        mjcf = """<mujoco>
          <default>
            <default class="robot">
              <geom rgba="0.5 0.5 0.5 1"/>
              <default class="visual">
                <geom contype="0" conaffinity="0" material="blue"/>
              </default>
              <default class="collision"><geom group="3"/></default>
            </default>
          </default>
          <asset>
            <material name="blue" rgba="0 0 1 1"/>
            <material name="red" rgba="1 0 0 1"/>
          </asset>
          <worldbody>
            <geom type="mesh" mesh="world_plain"/>
            <body childclass="robot">
              <geom class="collision" type="mesh" mesh="a" rgba="0 1 0 1"/>
              <geom class="visual" type="mesh" mesh="a"/>
              <frame>
                <body>
                  <geom class="visual" type="mesh" mesh="b" material="red"/>
                  <geom class="visual" type="mesh" mesh="c" rgba="1 1 0 1"/>
                  <geom type="mesh" mesh="d"/>
                </body>
              </frame>
            </body>
          </worldbody>
        </mujoco>"""
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "robot.xml")
            with open(path, "w") as f:
                f.write(mjcf)
            colors = mjcf_mesh_colors(path)
        self.assertNotIn("world_plain", colors)          # no color information
        self.assertEqual(colors["a"], (0.0, 0.0, 1.0, 1.0))  # visual geom wins over collision
        self.assertEqual(colors["b"], (1.0, 0.0, 0.0, 1.0))  # own material over class material
        self.assertEqual(colors["c"], (1.0, 1.0, 0.0, 1.0))  # non-default rgba over material
        self.assertEqual(colors["d"], (0.5, 0.5, 0.5, 1.0))  # childclass through a frame

    @unittest.skipUnless(HAS_MUJOCO, "mujoco is not installed")
    def test_matches_mujoco_compiled_colors(self):
        import mujoco
        import numpy as np

        for path in (G1, G1_HANDS):
            model = mujoco.MjModel.from_xml_path(str(path))
            expected = {}
            for g in range(model.ngeom):
                if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                    continue
                if model.geom_contype[g] or model.geom_conaffinity[g]:
                    continue
                mesh = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, model.geom_dataid[g])
                rgba = model.geom_rgba[g]
                if model.geom_matid[g] >= 0 and np.allclose(rgba, (0.5, 0.5, 0.5, 1.0)):
                    rgba = model.mat_rgba[model.geom_matid[g]]
                expected[mesh] = tuple(round(float(v), 6) for v in rgba)
            resolved = {
                mesh: tuple(round(v, 6) for v in color)
                for mesh, color in mjcf_mesh_colors(path).items()
            }
            self.assertEqual(resolved, expected, path.name)


if __name__ == "__main__":
    unittest.main()
