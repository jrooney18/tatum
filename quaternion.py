import numpy as np


def Quaternion(axis, angle):
    ''' Creates a quaternion from the axis-angle rotation representation '''
    quat = np.array([np.cos(angle/2),
                     np.sin(angle/2) * axis[0],
                     np.sin(angle/2) * axis[1],
                     np.sin(angle/2) * axis[2]
                     ])
    return quat

def quaternion_inverse(quat):
    ''' Returns the inverse of the input quaternion '''
    quat_inv = np.copy(quat)
    quat_inv[1:] = -quat_inv[1:]
    return quat_inv

def quat_mult(q, p):
    ''' Multiplies 2 quaternions, q and p, in format (a, bi, cj, dk) '''
    q0, q1, q2, q3 = q.T
    p0, p1, p2, p3 = p.T
    qp = np.array([
        q0*p0 - q1*p1 - q2*p2 - q3*p3,
        q0*p1 + q1*p0 - q2*p3 + q3*p2,
        q0*p2 + q1*p3 + q2*p0 - q3*p1,
        q0*p3 - q1*p2 + q2*p1 + q3*p0
        ])
    return qp.T

def rotate_via_quat(points, quat):
    ''' Rotate a set of points using a quaterion '''
    _points = np.insert(points, 0, 0, axis=1)
    quat_inv = quaternion_inverse(quat)
    _points_rot = quat_mult(quat, _points)
    _points_rot = quat_mult(_points_rot, quat_inv)
    _points_rot = np.delete(_points_rot, 0, axis=1)
    return _points_rot