"""
BVH file loader for motion retargeting.

Supports optitrack format for Unitree robots.
"""

import re
import numpy as np
from scipy.spatial.transform import Rotation as R

from .quat_utils import (
    euler_to_quat,
    remove_quat_discontinuities,
    quat_mul_batch,
    quat_mul_vec_batch,
)


class BVHAnimation:
    """A simple animation object containing BVH data."""
    
    def __init__(self, quats, pos, offsets, parents, bones):
        """
        Args:
            quats: local quaternions tensor (T, J, 4) in (w, x, y, z) format
            pos: local positions tensor (T, J, 3)
            offsets: local joint offsets (J, 3)
            parents: bone hierarchy (list of parent indices)
            bones: bone names (list of strings)
        """
        self.quats = quats
        self.pos = pos
        self.offsets = offsets
        self.parents = parents
        self.bones = bones


def read_bvh(filename, start=None, end=None, order=None):
    """
    Reads a BVH file and extracts animation information.
    
    Args:
        filename: BVH filename
        start: start frame (optional)
        end: end frame (optional)
        order: order of euler rotations (optional, auto-detected if None)
        
    Returns:
        BVHAnimation object containing the extracted information.
    """
    channelmap = {
        'Xrotation': 'x',
        'Yrotation': 'y',
        'Zrotation': 'z'
    }

    f = open(filename, "r")

    i = 0
    active = -1
    end_site = False

    names = []
    orients = np.array([]).reshape((0, 4))
    offsets = np.array([]).reshape((0, 3))
    parents = np.array([], dtype=int)

    # Parse the file, line by line
    for line in f:
        if "HIERARCHY" in line:
            continue
        if "MOTION" in line:
            continue

        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            names.append(rmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "{" in line:
            continue

        if "}" in line:
            if end_site:
                end_site = False
            else:
                active = parents[active]
            continue

        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            if not end_site:
                offsets[active] = np.array([list(map(float, offmatch.groups()))])
            continue

        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            channels = int(chanmatch.group(1))
            if order is None:
                channelis = 0 if channels == 3 else 3
                channelie = 3 if channels == 3 else 6
                parts = line.split()[2 + channelis:2 + channelie]
                if any([p not in channelmap for p in parts]):
                    continue
                order = "".join([channelmap[p] for p in parts])
            continue

        jmatch = re.match(r"\s*JOINT\s+(\w+)", line)
        if jmatch:
            names.append(jmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "End Site" in line:
            end_site = True
            continue

        fmatch = re.match(r"\s*Frames:\s+(\d+)", line)
        if fmatch:
            if start and end:
                fnum = (end - start) - 1
            else:
                fnum = int(fmatch.group(1))
            positions = offsets[np.newaxis].repeat(fnum, axis=0)
            rotations = np.zeros((fnum, len(orients), 3))
            continue

        fmatch = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            frametime = float(fmatch.group(1))
            continue

        if (start and end) and (i < start or i >= end - 1):
            i += 1
            continue

        dmatch = line.strip().split(' ')
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = len(parents)
            fi = i - start if start else i
            if channels == 3:
                positions[fi, 0:1] = data_block[0:3]
                rotations[fi, :] = data_block[3:].reshape(N, 3)
            elif channels == 6:
                data_block = data_block.reshape(N, 6)
                positions[fi, :] = data_block[:, 0:3]
                rotations[fi, :] = data_block[:, 3:6]
            elif channels == 9:
                positions[fi, 0] = data_block[0:3]
                data_block = data_block[3:].reshape(N - 1, 9)
                rotations[fi, 1:] = data_block[:, 3:6]
                positions[fi, 1:] += data_block[:, 0:3] * data_block[:, 6:9]
            else:
                raise Exception("Too many channels! %i" % channels)

            i += 1

    f.close()

    rotations = euler_to_quat(np.radians(rotations), order=order)
    rotations = remove_quat_discontinuities(rotations)

    return BVHAnimation(rotations, positions, offsets, parents, names)


def quat_fk(lrot, lpos, parents):
    """
    Performs Forward Kinematics (FK) on local quaternions and local positions 
    to retrieve global representations.
    
    Args:
        lrot: tensor of local quaternions with shape (..., Nb of joints, 4) in (w, x, y, z) format
        lpos: tensor of local positions with shape (..., Nb of joints, 3)
        parents: list of parents indices
        
    Returns:
        tuple of tensors of global quaternion, global positions
    """
    gp, gr = [lpos[..., :1, :]], [lrot[..., :1, :]]
    for i in range(1, len(parents)):
        gp.append(quat_mul_vec_batch(gr[parents[i]], lpos[..., i:i+1, :]) + gp[parents[i]])
        gr.append(quat_mul_batch(gr[parents[i]], lrot[..., i:i+1, :]))

    res = np.concatenate(gr, axis=-2), np.concatenate(gp, axis=-2)
    return res


def load_bvh_file(bvh_file, human_height=1.75):
    """
    Load a BVH file and return frame data in the format expected by Retargeter.
    
    Args:
        bvh_file: Path to BVH file
        human_height: Assumed human height in meters (default: 1.75)
        
    Returns:
        Tuple of (frames, human_height) where frames is a list of dictionaries
        with bone names as keys and [position, orientation] as values.
    """
    data = read_bvh(bvh_file)
    global_data = quat_fk(data.quats, data.pos, data.parents)

    # Rotation matrix for Y-up to Z-up conversion (90 deg around X)
    rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)

    frames = []
    for frame in range(data.pos.shape[0]):
        result = {}
        for i, bone in enumerate(data.bones):
            orientation = quat_mul_batch(rotation_quat, global_data[0][frame, i])
            position = global_data[1][frame, i] @ rotation_matrix.T
            result[bone] = [position, orientation]
        
        # Add modified foot pose for optitrack format
        # FootMod uses foot position with toe rotation
        if "LeftFoot" in result and "LeftToeBase" in result:
            result["LeftFootMod"] = [result["LeftFoot"][0].copy(), result["LeftToeBase"][1].copy()]
        if "RightFoot" in result and "RightToeBase" in result:
            result["RightFootMod"] = [result["RightFoot"][0].copy(), result["RightToeBase"][1].copy()]
            
        frames.append(result)

    return frames, human_height
