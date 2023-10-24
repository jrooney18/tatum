import matplotlib.pyplot as plt
import numpy as np
import serial
import time


def rotation_matrix(axis, angle):
    ''' Create rotation matrix for angle (in rad) about axis ('x', 'y', 'z')'''
    c = np.cos(angle)
    s = np.sin(angle)
    
    match axis:
        case 'x':
            rot = np.array([[1, 0,  0],
                            [0, c, -s],
                            [0, s,  c]])
        case 'y':
            rot = np.array([[ c, 0, s],
                            [ 0, 1, 0],
                            [-s, 0, c]])
        case 'z':
            rot = np.array([[c, -s, 0],
                            [s,  c, 0],
                            [0,  0, 1]])
    return rot

def dh_rot(points, rot, x_angle, z_offset=0):
    ''' Find the Denavit-Hartenberg angle about Z for a given link '''
    # Define the rotation about the x-axis
    rot_x = rotation_matrix('x', np.deg2rad(x_angle))
    # Rotate the segment by rot and the new x-axis rotation
    points = np.matmul(rot_x, np.matmul(np.linalg.inv(rot), points))
    # Shift both points to place point 0 at the origin
    points -= points[:, [0]]
    # Find the angle about the z-axis to point 1
    angle = np.arctan2(points[1, 1], points[0, 1]) + np.deg2rad(z_offset)
    # Define the rotation matrices about the z-axis and overall
    rot_z = rotation_matrix('z', -angle)
    rot_new = np.matmul(rot_x, rot_z)
    rot = np.matmul(rot, rot_new)
    return angle, rot

def plot_axes(ax, wcs, origin = np.zeros(3)):
    ax.plot([origin[0], wcs[0, 0]],
            [origin[1], wcs[1, 0]],
            [origin[2], wcs[2, 0]],
            'r')
    ax.plot([origin[0], wcs[0, 1]],
            [origin[1], wcs[1, 1]],
            [origin[2], wcs[2, 1]],
            'g')
    ax.plot([origin[0], wcs[0, 2]],
            [origin[1], wcs[1, 2]],
            [origin[2], wcs[2, 2]],
            'b')
    

if __name__ == '__main__':
    fig = plt.figure(figsize=[16, 16])
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(azim=50)
  
    # Generate a unit vector with -1 < x < 1, y > 0, z < 0
    points = np.zeros([3, 3])
    points[0, 1] = (np.random.rand() - 0.5) * 2
    points[1, 1] = np.random.rand()
    points[2, 1] = np.random.rand() * -1
    
    # Generate a second unit vector with its endpoint on the first
    points[0, 2] = np.random.rand()
    points[1, 2] = np.random.rand()
    points[2, 2] = np.random.rand()
    
    # Normalize both vectors
    points[:, 1:] /= np.linalg.norm(points[:, 1:], axis=0)
    points[:, 2] += points[:, 1]
    
    # Plot the vectors
    wcs = np.eye(3) * 0.3
    plot_axes(ax, wcs)

    ax.plot([points[0, 0], points[0, 1]],
            [points[0, 0], points[1, 1]],
            [points[0, 0], points[2, 1]],
            'ko-')
    ax.plot([points[0, 1], points[0, 2]],
            [points[1, 1], points[1, 2]],
            [points[2, 1], points[2, 2]],
            'ko-')
    
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_aspect('equal')
    
    #### Finding angles #   #   #   #   #
    angles = np.zeros([4])
    rot_0 = np.eye(3)
    rot = rot_0
    
    angles[0], rot = dh_rot(points[:, 0:2], rot, x_angle=-90)
    cs1 = np.matmul(rot, wcs)
    plot_axes(ax, cs1)
    angles[1], rot = dh_rot(points[:, 0:2], rot, x_angle=-90, z_offset=90)
    cs2 = np.matmul(rot, wcs)
    plot_axes(ax, cs2)
    angles[2], rot = dh_rot(points[:, 1:3], rot, x_angle=-90)
    cs3 = np.matmul(rot, wcs) + points[:, [1]] * 0.5
    plot_axes(ax, cs3, origin=points[:, 1] * 0.5)
    angles[3], rot = dh_rot(points[:, 1:3], rot, x_angle=90)
    cs4 = np.matmul(rot, wcs) + points[:, [1]]
    plot_axes(ax, cs4, origin=points[:, 1])
    
    # Apply offsets to joint angles
    deg_angles = np.rad2deg(angles)
    deg_angles[0] = -deg_angles[0] + 90
    deg_angles[2] = -deg_angles[2] + 180
    deg_angles[3] += 135
    
    
    # Report the angles
    for i, angle in enumerate(angles):
        print(f'Angle {i}: {deg_angles[i]}')
    print('\n')
    
    # Convert the two angles to motor ticks
    ticks_per_deg = 4096 / (360)
    ticks = deg_angles * ticks_per_deg
    for i, tick in enumerate(ticks):
        print(f'Motor {i}: {ticks[i]}')