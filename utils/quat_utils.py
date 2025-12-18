"""
Quaternion utility functions for motion retargeting.

All quaternions are in (w, x, y, z) format unless otherwise specified.
"""

import numpy as np


def quat_mul(q1, q2):
    """
    Multiply two quaternions (w, x, y, z format).
    
    Args:
        q1: First quaternion (w, x, y, z)
        q2: Second quaternion (w, x, y, z)
        
    Returns:
        Product quaternion (w, x, y, z)
    """
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def quat_conj(q):
    """
    Quaternion conjugate (w, x, y, z format).
    
    Args:
        q: Quaternion (w, x, y, z)
        
    Returns:
        Conjugate quaternion (w, -x, -y, -z)
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q):
    """
    Normalize quaternion (w, x, y, z format).
    
    Args:
        q: Quaternion (w, x, y, z)
        
    Returns:
        Normalized quaternion
    """
    norm = np.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    if norm < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def rotate_vec_by_quat(v, q):
    """
    Rotate vector v by quaternion q (w, x, y, z format).
    Uses optimized Rodrigues' rotation formula.
    
    Args:
        v: 3D vector
        q: Quaternion (w, x, y, z)
        
    Returns:
        Rotated 3D vector
    """
    w, x, y, z = q[0], q[1], q[2], q[3]
    # t = 2 * cross(q_xyz, v)
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    # result = v + w * t + cross(q_xyz, t)
    return np.array([
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx)
    ])


def unity_to_opengl_vec(v):
    """
    Convert Unity position to OpenGL coordinate system.
    Unity: left-handed, Y-up
    OpenGL: right-handed, Z-up
    Conversion: (-x, y, z) - mirrors X axis
    
    Args:
        v: Position vector in Unity coordinates
        
    Returns:
        Position vector in OpenGL coordinates
    """
    return np.array([-v[0], v[1], v[2]])


def unity_to_opengl_quat(q):
    """
    Convert Unity quaternion to OpenGL coordinate system.
    q is in (w, x, y, z) format.
    Conversion: (w, x, -y, -z) - mirrors Y and Z components
    
    Args:
        q: Quaternion in Unity coordinates (w, x, y, z)
        
    Returns:
        Quaternion in OpenGL coordinates (w, x, -y, -z)
    """
    return np.array([q[0], q[1], -q[2], -q[3]])


# Batch quaternion operations for BVH processing

def quat_mul_batch(x, y):
    """
    Performs quaternion multiplication on arrays of quaternions.
    
    Args:
        x: tensor of quaternions of shape (..., 4) in (w, x, y, z) format
        y: tensor of quaternions of shape (..., 4) in (w, x, y, z) format
        
    Returns:
        The resulting quaternions
    """
    x0, x1, x2, x3 = x[..., 0:1], x[..., 1:2], x[..., 2:3], x[..., 3:4]
    y0, y1, y2, y3 = y[..., 0:1], y[..., 1:2], y[..., 2:3], y[..., 3:4]

    res = np.concatenate([
        y0 * x0 - y1 * x1 - y2 * x2 - y3 * x3,
        y0 * x1 + y1 * x0 - y2 * x3 + y3 * x2,
        y0 * x2 + y1 * x3 + y2 * x0 - y3 * x1,
        y0 * x3 - y1 * x2 + y2 * x1 + y3 * x0], axis=-1)

    return res


def quat_mul_vec_batch(q, x):
    """
    Performs multiplication of an array of 3D vectors by an array of quaternions (rotation).
    
    Args:
        q: tensor of quaternions of shape (..., 4) in (w, x, y, z) format
        x: tensor of vectors of shape (..., 3)
        
    Returns:
        The resulting array of rotated vectors
    """
    t = 2.0 * np.cross(q[..., 1:], x)
    res = x + q[..., 0][..., np.newaxis] * t + np.cross(q[..., 1:], t)
    return res


def angle_axis_to_quat(angle, axis):
    """
    Converts from angle-axis representation to quaternion representation.
    
    Args:
        angle: angles tensor
        axis: axis tensor
        
    Returns:
        quaternion tensor in (w, x, y, z) format
    """
    c = np.cos(angle / 2.0)[..., np.newaxis]
    s = np.sin(angle / 2.0)[..., np.newaxis]
    q = np.concatenate([c, s * axis], axis=-1)
    return q


def euler_to_quat(e, order='zyx'):
    """
    Converts from euler representation to quaternion representation.
    
    Args:
        e: euler tensor (radians)
        order: order of euler rotations
        
    Returns:
        quaternion tensor in (w, x, y, z) format
    """
    axis = {
        'x': np.asarray([1, 0, 0], dtype=np.float32),
        'y': np.asarray([0, 1, 0], dtype=np.float32),
        'z': np.asarray([0, 0, 1], dtype=np.float32)}

    q0 = angle_axis_to_quat(e[..., 0], axis[order[0]])
    q1 = angle_axis_to_quat(e[..., 1], axis[order[1]])
    q2 = angle_axis_to_quat(e[..., 2], axis[order[2]])

    return quat_mul_batch(q0, quat_mul_batch(q1, q2))


def remove_quat_discontinuities(rotations):
    """
    Removing quaternion discontinuities on the time dimension (removing flips).
    
    Args:
        rotations: Array of quaternions of shape (T, J, 4)
        
    Returns:
        The processed array without quaternion inversion.
    """
    rots_inv = -rotations

    for i in range(1, rotations.shape[0]):
        # Compare dot products
        replace_mask = np.sum(rotations[i - 1: i] * rotations[i: i + 1], axis=-1) < np.sum(
            rotations[i - 1: i] * rots_inv[i: i + 1], axis=-1)
        replace_mask = replace_mask[..., np.newaxis]
        rotations[i] = replace_mask * rots_inv[i] + (1.0 - replace_mask) * rotations[i]

    return rotations
