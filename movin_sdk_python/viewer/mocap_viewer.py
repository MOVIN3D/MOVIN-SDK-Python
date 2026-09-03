"""MuJoCo stick-figure viewer for raw MOVIN mocap frames."""

from __future__ import annotations

import sys
import time

import mujoco as mj
import mujoco.viewer as mjv
import numpy as np
from loop_rate_limiters import RateLimiter

from ..utils.fk_utils import compute_forward_kinematics


_EMPTY_SCENE_XML = """
<mujoco model="movin_mocap_stick_figure">
  <statistic center="1.0 0.7 1.0" extent="0.8"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1"
               specular="0.9 0.9 0.9"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-140" elevation="-20"
            offwidth="2080" offheight="1170"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="1 1 1" rgb2="1 1 1"
             width="800" height="800"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="1 1 1" rgb2="1 1 1" markrgb="0 0 0"
             width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true"
              texrepeat="5 5" reflectance="0"/>
    <texture type="skybox" builtin="gradient" rgb1=".4 .5 .6" rgb2="0 0 0"
             width="100" height="100"/>
    <texture builtin="flat" height="1278" mark="cross" markrgb="1 1 1"
             name="texgeom" random="0.01" rgb1="0.8 0.6 0.4"
             rgb2="0.8 0.6 0.4" type="cube" width="127"/>
    <texture name="texplane" builtin="checker" height="512" width="512"
             rgb1=".2 .3 .4" rgb2=".1 .15 .2" type="2d"/>
    <material name="MatPlane" reflectance="0.5" shininess="0.01"
              specular="0.1" texrepeat="1 1" texture="texplane"
              texuniform="true"/>
    <material name="geom" texture="texgeom" texuniform="true"/>
  </asset>
  <worldbody>
    <geom name="floor" size="0 0 0.01" type="plane"
          material="groundplane" contype="1" conaffinity="0" priority="1"
          friction="0.6" condim="3"/>
    <light diffuse=".5 .5 .5" pos="-3 -3 5" dir="3 3 -5"
           castshadow="true"/>
  </worldbody>
</mujoco>
"""


class MocapViewer:
    """Visualize an unretargeted MOVIN frame as a MuJoCo stick figure.

    Bone-local transforms are converted to global, right-handed Z-up
    coordinates with the SDK's existing mocap forward-kinematics utility.
    The viewer renders custom spheres and capsules and does not create or solve
    a robot model.
    """

    def __init__(
        self,
        motion_fps: int = 60,
        camera_distance: float = 3.0,
        camera_elevation: float = -10.0,
        joint_radius: float = 0.025,
        bone_radius: float = 0.012,
        joint_color=(1.0, 0.55, 0.1, 1.0),
        bone_color=(0.15, 0.65, 1.0, 1.0),
        show_left_ui: bool = False,
        show_right_ui: bool = False,
        keyboard_callback=None,
    ):
        if motion_fps <= 0:
            raise ValueError("motion_fps must be positive")
        if joint_radius <= 0 or bone_radius <= 0:
            raise ValueError("joint_radius and bone_radius must be positive")

        self.motion_fps = motion_fps
        self.joint_radius = float(joint_radius)
        self.bone_radius = float(bone_radius)
        self.joint_color = self._as_rgba(joint_color, "joint_color")
        self.bone_color = self._as_rgba(bone_color, "bone_color")
        self.rate_limiter = RateLimiter(frequency=motion_fps, warn=False)

        self.model = mj.MjModel.from_xml_string(_EMPTY_SCENE_XML)
        self.data = mj.MjData(self.model)
        mj.mj_forward(self.model, self.data)

        try:
            self.viewer = mjv.launch_passive(
                model=self.model,
                data=self.data,
                show_left_ui=show_left_ui,
                show_right_ui=show_right_ui,
                key_callback=keyboard_callback,
            )
        except RuntimeError as exc:
            if "mjpython" in str(exc) and sys.platform == "darwin":
                raise RuntimeError(
                    "On macOS, the MuJoCo viewer must be run with mjpython. "
                    "Use: mjpython your_script.py ..."
                ) from exc
            raise

        with self.viewer.lock():
            self.viewer.cam.distance = camera_distance
            self.viewer.cam.elevation = camera_elevation
        self.viewer.sync()

    @staticmethod
    def _as_rgba(value, name):
        rgba = np.asarray(value, dtype=np.float32)
        if rgba.shape != (4,):
            raise ValueError(f"{name} must contain exactly four RGBA values")
        return rgba

    @staticmethod
    def _stick_geometry(frame):
        if not isinstance(frame, dict) or "bones" not in frame:
            raise TypeError("MocapViewer expects a receiver frame with 'bones'")
        bones = frame["bones"]
        if not isinstance(bones, (list, tuple)) or not bones:
            raise ValueError("MocapViewer requires at least one bone")

        transforms = compute_forward_kinematics(bones)
        bone_by_index = {int(bone["bone_index"]): bone for bone in bones}
        positions = {}

        for bone_index, bone in bone_by_index.items():
            transform = transforms.get(bone["bone_name"])
            if transform is not None:
                positions[bone_index] = np.asarray(transform[0], dtype=np.float64)

        edges = []
        for bone_index, bone in bone_by_index.items():
            parent_index = int(bone["parent_index"])
            if bone_index not in positions or parent_index not in positions:
                continue
            start = positions[parent_index]
            end = positions[bone_index]
            if np.linalg.norm(end - start) > 1e-8:
                edges.append((start, end))

        if not positions:
            raise ValueError("MocapViewer could not compute any bone positions")
        return positions, edges, bone_by_index

    def step(
        self,
        frame,
        rate_limit: bool = True,
        follow_camera: bool = True,
    ) -> bool:
        """Render one raw mocap frame and return whether the window is open."""
        positions, edges, bone_by_index = self._stick_geometry(frame)
        required_geoms = len(edges) + len(positions)
        scene = self.viewer.user_scn
        if required_geoms > scene.maxgeom:
            raise RuntimeError(
                f"Mocap frame needs {required_geoms} custom geoms, but MuJoCo "
                f"allocated only {scene.maxgeom}"
            )

        zero = np.zeros(3, dtype=np.float64)
        identity = np.eye(3, dtype=np.float64).ravel()

        with self.viewer.lock():
            geom_index = 0
            for start, end in edges:
                geom = scene.geoms[geom_index]
                mj.mjv_initGeom(
                    geom,
                    type=mj.mjtGeom.mjGEOM_CAPSULE,
                    size=zero,
                    pos=zero,
                    mat=identity,
                    rgba=self.bone_color,
                )
                mj.mjv_connector(
                    geom,
                    mj.mjtGeom.mjGEOM_CAPSULE,
                    self.bone_radius,
                    start,
                    end,
                )
                geom_index += 1

            sphere_size = np.array(
                [self.joint_radius, 0.0, 0.0], dtype=np.float64
            )
            for position in positions.values():
                mj.mjv_initGeom(
                    scene.geoms[geom_index],
                    type=mj.mjtGeom.mjGEOM_SPHERE,
                    size=sphere_size,
                    pos=position,
                    mat=identity,
                    rgba=self.joint_color,
                )
                geom_index += 1

            scene.ngeom = geom_index
            if follow_camera:
                self.viewer.cam.lookat[:] = self._camera_target(
                    positions, bone_by_index
                )

        self.viewer.sync()
        if rate_limit:
            self.rate_limiter.sleep()
        return self.viewer.is_running()

    @staticmethod
    def _camera_target(positions, bone_by_index):
        for preferred_name in ("Hips", "Root"):
            for bone_index, bone in bone_by_index.items():
                if bone["bone_name"] == preferred_name and bone_index in positions:
                    return positions[bone_index]
        return next(iter(positions.values()))

    def on_frame(self, frame):
        """Render a raw frame when attached as a ``MovinSession`` sink."""
        return self.step(frame)

    def is_running(self) -> bool:
        """Return whether the MuJoCo window remains open."""
        return self.viewer.is_running()

    def close(self):
        """Close the viewer window."""
        self.viewer.close()
        time.sleep(0.5)
