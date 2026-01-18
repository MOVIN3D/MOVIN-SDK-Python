"""
Retargeter class for motion retargeting from optitrack mocap to Unitree robots.
"""

import pathlib
import json

import mink
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation as R

from ..utils.quat_utils import quat_mul, rotate_vec_by_quat
from ..utils.bvh_loader import load_bvh_file
from ..utils.fk_utils import process_mocap_frame


# Package paths
HERE = pathlib.Path(__file__).parent
ASSETS_ROOT = HERE / "assets"
IK_CONFIG_ROOT = HERE / "ik_configs"

# Robot XML paths
ROBOT_XML_DICT = {
    "unitree_g1": ASSETS_ROOT / "unitree_g1" / "g1_mocap_29dof.xml",
    "unitree_g1_with_hands": ASSETS_ROOT / "unitree_g1" / "g1_mocap_29dof_with_hands.xml",
}

# IK config paths (optitrack format only)
IK_CONFIG_DICT = {
    "unitree_g1": IK_CONFIG_ROOT / "bvh_optitrack_to_g1.json",
    "unitree_g1_with_hands": IK_CONFIG_ROOT / "bvh_optitrack_to_g1.json",
}


class Retargeter:
    """
    Motion retargeter from optitrack mocap data to Unitree robots.
    
    Supports:
    - unitree_g1: Unitree G1 robot (29 DoF)
    - unitree_g1_with_hands: Unitree G1 robot with hands
    
    Example usage:
        retargeter = Retargeter(robot_type="unitree_g1", human_height=1.75)
        
        # From BVH file
        frames, height = retargeter.load_bvh("motion.bvh")
        for frame in frames:
            qpos = retargeter.retarget(frame)
            
        # From real-time mocap
        mocap_data = retargeter.process_mocap_frame(bones)
        qpos = retargeter.retarget(mocap_data)
    """
    
    def __init__(
        self,
        robot_type: str = "unitree_g1",
        human_height: float = 1.75,
        solver: str = "daqp",
        damping: float = 5e-1,
        verbose: bool = False,
        use_velocity_limit: bool = False,
    ):
        """
        Initialize the retargeter.
        
        Args:
            robot_type: Robot type ("unitree_g1" or "unitree_g1_with_hands")
            human_height: Human height in meters for scaling
            solver: IK solver type ("daqp" or "quadprog")
            damping: IK damping factor
            verbose: Print debug information
            use_velocity_limit: Enable velocity limits in IK
        """
        if robot_type not in ROBOT_XML_DICT:
            raise ValueError(f"Unknown robot type: {robot_type}. "
                           f"Supported: {list(ROBOT_XML_DICT.keys())}")
        
        self.robot_type = robot_type
        self.verbose = verbose
        
        # Load the robot model
        self.xml_file = str(ROBOT_XML_DICT[robot_type])
        if verbose:
            print(f"Loading robot model: {self.xml_file}")
        self.model = mj.MjModel.from_xml_path(self.xml_file)
        
        # Get DoF names
        self.robot_dof_names = {}
        for i in range(self.model.nv):
            dof_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, self.model.dof_jntid[i])
            self.robot_dof_names[dof_name] = i
            
        # Get body names
        self.robot_body_names = {}
        for i in range(self.model.nbody):
            body_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, i)
            self.robot_body_names[body_name] = i
            
        # Get motor names
        self.robot_motor_names = {}
        for i in range(self.model.nu):
            motor_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i)
            self.robot_motor_names[motor_name] = i

        # Load the IK config
        ik_config_path = IK_CONFIG_DICT[robot_type]
        if verbose:
            print(f"Loading IK config: {ik_config_path}")
        with open(ik_config_path) as f:
            ik_config = json.load(f)
        
        # Compute scale ratio based on human height
        ratio = human_height / ik_config["human_height_assumption"]
        
        # Adjust the human scale table
        for key in ik_config["human_scale_table"].keys():
            ik_config["human_scale_table"][key] = ik_config["human_scale_table"][key] * ratio

        # Store IK configuration
        self.ik_match_table1 = ik_config["ik_match_table1"]
        self.ik_match_table2 = ik_config["ik_match_table2"]
        self.human_root_name = ik_config["human_root_name"]
        self.robot_root_name = ik_config["robot_root_name"]
        self.use_ik_match_table1 = ik_config["use_ik_match_table1"]
        self.use_ik_match_table2 = ik_config["use_ik_match_table2"]
        self.human_scale_table = ik_config["human_scale_table"]
        self.ground = ik_config["ground_height"] * np.array([0, 0, 1])

        # IK parameters
        self.max_iter = 10
        self.ik_convergence_threshold = 0.001
        self.solver = solver
        self.damping = damping

        # Task mappings
        self.human_body_to_task1 = {}
        self.human_body_to_task2 = {}
        self.pos_offsets1 = {}
        self.rot_offsets1 = {}
        self.pos_offsets2 = {}
        self.rot_offsets2 = {}
        
        # Pre-cached rotation data for optimization
        self._rot_offset_quats1 = {}

        # IK limits
        self.ik_limits = [mink.ConfigurationLimit(self.model)]
        if use_velocity_limit:
            VELOCITY_LIMITS = {k: 3*np.pi for k in self.robot_motor_names.keys()}
            self.ik_limits.append(mink.VelocityLimit(self.model, VELOCITY_LIMITS))
            
        self._setup_retarget_configuration()
        
        self.ground_offset = 0.0
        self.scaled_human_data = None

    def _setup_retarget_configuration(self):
        """Set up the IK configuration and tasks."""
        self.configuration = mink.Configuration(self.model)
    
        self.tasks1 = []
        self.tasks2 = []
        
        for frame_name, entry in self.ik_match_table1.items():
            body_name, pos_weight, rot_weight, pos_offset, rot_offset = entry
            if pos_weight != 0 or rot_weight != 0:
                task = mink.FrameTask(
                    frame_name=frame_name,
                    frame_type="body",
                    position_cost=pos_weight,
                    orientation_cost=rot_weight,
                    lm_damping=1,
                )
                self.human_body_to_task1[body_name] = task
                self.pos_offsets1[body_name] = np.array(pos_offset) - self.ground
                self.rot_offsets1[body_name] = R.from_quat(rot_offset, scalar_first=True)
                self._rot_offset_quats1[body_name] = np.array(rot_offset)
                self.tasks1.append(task)
        
        for frame_name, entry in self.ik_match_table2.items():
            body_name, pos_weight, rot_weight, pos_offset, rot_offset = entry
            if pos_weight != 0 or rot_weight != 0:
                task = mink.FrameTask(
                    frame_name=frame_name,
                    frame_type="body",
                    position_cost=pos_weight,
                    orientation_cost=rot_weight,
                    lm_damping=1,
                )
                self.human_body_to_task2[body_name] = task
                self.pos_offsets2[body_name] = np.array(pos_offset) - self.ground
                self.rot_offsets2[body_name] = R.from_quat(rot_offset, scalar_first=True)
                self.tasks2.append(task)

    def load_bvh(self, bvh_file, human_height=None):
        """
        Load a BVH file and return frame data.
        
        Args:
            bvh_file: Path to BVH file
            human_height: Override human height (optional)
            
        Returns:
            Tuple of (frames, human_height, parents, bones) where:
            - frames: list of dictionaries with bone names as keys
            - human_height: assumed human height in meters
            - parents: numpy array of parent indices for each joint
            - bones: list of bone names
        """
        height = human_height if human_height is not None else 1.75
        return load_bvh_file(bvh_file, human_height=height)

    def process_mocap_frame(self, bones):
        """
        Process real-time mocap frame from OSC data.
        
        Args:
            bones: List of bone dicts from OSC receiver with keys:
                - bone_index, parent_index, bone_name
                - p: (px, py, pz) local position
                - q: (w, x, y, z) local rotation
                - rq: (w, x, y, z) rest pose
                - s: (sx, sy, sz) scale
                
        Returns:
            Dict mapping bone_name to [position, rotation] suitable for retarget()
        """
        return process_mocap_frame(bones)

    def retarget(self, human_data, offset_to_ground=False):
        """
        Retarget human motion data to robot joint positions.
        
        Args:
            human_data: Dict mapping bone names to [position, quaternion]
                       Position is 3D vector, quaternion is (w, x, y, z)
            offset_to_ground: If True, offset to place lowest foot at ground
            
        Returns:
            numpy array of robot joint positions (qpos):
                - qpos[:3] = root position (x, y, z)
                - qpos[3:7] = root orientation (w, x, y, z quaternion)
                - qpos[7:] = joint angles
        """
        # Update task targets
        self._update_targets(human_data, offset_to_ground)

        # Solve IK for table 1
        if self.use_ik_match_table1:
            curr_error = self._error1()
            dt = self.configuration.model.opt.timestep
            vel1 = mink.solve_ik(
                self.configuration, self.tasks1, dt, self.solver, self.damping, self.ik_limits
            )
            self.configuration.integrate_inplace(vel1, dt)
            next_error = self._error1()
            num_iter = 0
            while curr_error - next_error > self.ik_convergence_threshold and num_iter < self.max_iter:
                curr_error = next_error
                dt = self.configuration.model.opt.timestep
                vel1 = mink.solve_ik(
                    self.configuration, self.tasks1, dt, self.solver, self.damping, self.ik_limits
                )
                self.configuration.integrate_inplace(vel1, dt)
                next_error = self._error1()
                num_iter += 1

        # Solve IK for table 2
        if self.use_ik_match_table2:
            curr_error = self._error2()
            dt = self.configuration.model.opt.timestep
            vel2 = mink.solve_ik(
                self.configuration, self.tasks2, dt, self.solver, self.damping, self.ik_limits
            )
            self.configuration.integrate_inplace(vel2, dt)
            next_error = self._error2()
            num_iter = 0
            while curr_error - next_error > self.ik_convergence_threshold and num_iter < self.max_iter:
                curr_error = next_error
                dt = self.configuration.model.opt.timestep
                vel2 = mink.solve_ik(
                    self.configuration, self.tasks2, dt, self.solver, self.damping, self.ik_limits
                )
                self.configuration.integrate_inplace(vel2, dt)
                next_error = self._error2()
                num_iter += 1
                
        return self.configuration.data.qpos.copy()

    def get_required_bones(self):
        """
        Get the set of bone names required for retargeting.
        
        Returns:
            Set of bone name strings
        """
        required = (
            set(self.human_body_to_task1.keys())
            | set(self.human_body_to_task2.keys())
            | {self.human_root_name}
        )
        return required

    def set_ground_offset(self, ground_offset):
        """Set ground offset for height adjustment."""
        self.ground_offset = ground_offset

    def _update_targets(self, human_data, offset_to_ground=False):
        """Update IK targets from human data."""
        # Convert to numpy
        human_data = self._to_numpy(human_data)
        # Scale human data in local frame
        human_data = self._scale_human_data(human_data)
        # Apply rotation offsets
        human_data = self._offset_human_data(human_data)
        # Apply ground offset
        human_data = self._apply_ground_offset(human_data)
        if offset_to_ground:
            human_data = self._offset_human_data_to_ground(human_data)
        self.scaled_human_data = human_data

        if self.use_ik_match_table1:
            for body_name in self.human_body_to_task1.keys():
                task = self.human_body_to_task1[body_name]
                pos, rot = human_data[body_name]
                task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))
        
        if self.use_ik_match_table2:
            for body_name in self.human_body_to_task2.keys():
                task = self.human_body_to_task2[body_name]
                pos, rot = human_data[body_name]
                task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))

    def _error1(self):
        """Compute total error for IK table 1."""
        return np.linalg.norm(
            np.concatenate(
                [task.compute_error(self.configuration) for task in self.tasks1]
            )
        )
    
    def _error2(self):
        """Compute total error for IK table 2."""
        return np.linalg.norm(
            np.concatenate(
                [task.compute_error(self.configuration) for task in self.tasks2]
            )
        )

    def _to_numpy(self, human_data):
        """Convert human data to numpy arrays."""
        for body_name in human_data.keys():
            human_data[body_name] = [
                np.asarray(human_data[body_name][0]),
                np.asarray(human_data[body_name][1])
            ]
        return human_data

    def _scale_human_data(self, human_data):
        """Scale human data based on human height."""
        human_data_local = {}
        root_pos, root_quat = human_data[self.human_root_name]
        
        # Scale root
        scaled_root_pos = self.human_scale_table[self.human_root_name] * root_pos
        
        # Scale other body parts in local frame
        for body_name in human_data.keys():
            if body_name not in self.human_scale_table:
                continue
            if body_name == self.human_root_name:
                continue
            else:
                # Transform to local frame (only position)
                human_data_local[body_name] = (human_data[body_name][0] - root_pos) * self.human_scale_table[body_name]
            
        # Transform back to global frame
        human_data_global = {self.human_root_name: (scaled_root_pos, root_quat)}
        for body_name in human_data_local.keys():
            human_data_global[body_name] = (human_data_local[body_name] + scaled_root_pos, human_data[body_name][1])

        return human_data_global
    
    def _offset_human_data(self, human_data):
        """Apply position and rotation offsets."""
        offset_human_data = {}
        for body_name in human_data.keys():
            pos, quat = human_data[body_name]
            # Use pre-cached quaternion offset
            rot_offset_quat = self._rot_offset_quats1[body_name]
            # Quaternion multiplication
            updated_quat = quat_mul(quat, rot_offset_quat)
            # Rotate local_offset by updated_quat to get global offset
            local_offset = self.pos_offsets1[body_name]
            global_pos_offset = rotate_vec_by_quat(local_offset, updated_quat)
            offset_human_data[body_name] = [pos + global_pos_offset, updated_quat]
        return offset_human_data

    def _offset_human_data_to_ground(self, human_data):
        """Find lowest foot point and offset to ground."""
        offset_human_data = {}
        ground_offset = 0.1
        lowest_pos = np.inf

        for body_name in human_data.keys():
            if "Foot" not in body_name and "foot" not in body_name:
                continue
            pos, quat = human_data[body_name]
            if pos[2] < lowest_pos:
                lowest_pos = pos[2]
                
        for body_name in human_data.keys():
            pos, quat = human_data[body_name]
            offset_human_data[body_name] = [pos, quat]
            offset_human_data[body_name][0] = pos - np.array([0, 0, lowest_pos]) + np.array([0, 0, ground_offset])
        return offset_human_data

    def _apply_ground_offset(self, human_data):
        """Apply ground offset to all positions."""
        for body_name in human_data.keys():
            pos, quat = human_data[body_name]
            human_data[body_name][0] = pos - np.array([0, 0, self.ground_offset])
        return human_data
