import numpy as np 
import matplotlib.pyplot as plt

import keypoint_connections as conn
from Rotation_Matrix import rotation_matrix

def read_kpts(filename):
    ''' Reads mediapipe keyframes into numpy array
    Arguments: filename (str): name of the file containing keypoints
        (in working directory)
    Returns: 
        points (np array): n x 3 x 33 numpy array, containing 33 (x,y,z) 
            coordinate points for each of the n frames in the file
        num_frames (int): number of frames
    '''
    with open(filename, 'r') as f:
        lines = f.readlines()
        num_frames = len(lines)

    points = np.zeros((num_frames, 3, 33))

    # Split each frame's pts, reshape into float array, and store
    for i, line in enumerate(lines):
        tokens = line.strip().split()
        kpt = [float(token) for token in tokens]
        kpt_array = np.array(kpt).reshape((33, 3))
        points[i] = kpt_array.transpose()

    num_frames = points.shape[2]
    return points, num_frames

def unit_vector(point, origin=None):
    ''' Returns [x, y, z] unit vector aligned from [0, 0, 0] to input point '''
    if origin is not None:
        magnitude = np.linalg.norm(point - origin, axis=0)
        unit = (point - origin) / magnitude + origin
    else:
        magnitude = np.linalg.norm(point, axis=0)
        unit = point / magnitude
    return unit

def orient_kpts(points):
    ''' Orients Mediapipe keypoints to place the right shoulder at the origin,
        align the shoulder axis with +Y-axis, and align the vertical body axis
        with the Z-axis.
    Arguments:
        points (array): Keypoints for n frames, shape=(33, 3, 60)
    Returns: points_oriented (array)
    '''
    
    # Translate all points to place the right shoulder at (0, 0, 0)
    points -= points[:, [conn.shoulder_r]]

    # Rotate points to align the shoulder axis with the +Y-axis
    shoulder_unit = unit_vector(points[:, conn.shoulder_l])
    angle_z = np.pi + np.arctan2(shoulder_unit[1], shoulder_unit[0])
    angle_y = np.arctan2(shoulder_unit[2], shoulder_unit[0])
    
    rot_Z = rotation_matrix('z', -angle_z)
    rot_Y = rotation_matrix('y', angle_y)
    rot = np.matmul(rot_Z, rot_Y)
    points = np.matmul(rot, points)
    
    # Rotate to align the vertical body axis with the z-axis
    hip_unit = unit_vector(points[:, conn.hip_r])
    angle_hip = np.pi/2 + np.arctan2(hip_unit[2], hip_unit[1])
    rot_X = rotation_matrix('x', -angle_hip)
    points = np.matmul(rot_X, points)
    
    return points

def plot_axes(axes, length=1):
    ''' Plots X,Y,Z vectors of given length (default 1) on the input axes '''
    artists = []
    artists += axes.plot([0, length], [0, 0], [0, 0], 'r')
    artists += axes.plot([0, 0], [0, length], [0, 0], 'g')
    artists += axes.plot([0, 0], [0, 0], [0, length], 'b')
    artists += axes.plot(length, 0, 0, 'ro')
    artists += axes.plot(0, length, 0, 'go')
    artists += axes.plot(0, 0, length, 'bo')
    return artists

def plot_kpts(axes, points, connections, color):
    ''' Plots keypoints according to defined connections
    Arguments:
        axes: matplotlib axes object
        points (array): Numpy array of Mediapipe keypoints, with (x, y, z) data
            stored in the second axis
        connections (tuple): tuple of point connections, formatted 
            as (point 1, point 2)
        color (str): matplotlib-defined color
    '''
    artists = []
    for _c in connections:
        artists += axes.plot([points[0, _c[0]], points[0, _c[1]]],
                             [points[1, _c[0]], points[1, _c[1]]],
                             [points[2, _c[0]], points[2, _c[1]]],
                             '.-', color=color)
    return artists

def set_axis_limits(axes, x_min, x_max, y_min, y_max, z_min, z_max):
    ''' Set limits for the x-, y-, and z-axes of an axes object in one pass '''
    axes.set_xlim3d(x_min, x_max)
    axes.set_ylim3d(y_min, y_max)
    axes.set_zlim3d(z_min, z_max)
    axes.set_aspect('equal')
    return None


if __name__ == '__main__':
    path = r'Extreme ROM data\type\\'
    filename = r'kpts-type0.dat'
    points, num_frames = read_kpts(path + filename)
    
    fig = plt.figure("t", figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    lengths = np.zeros([num_frames, 2])
    
    for i in range(num_frames):
        vals = orient_kpts(points[i])
    
        # Plot points and lines for the body, arm, and axes
        plot_axes(ax, length=2)
        plot_kpts(ax, vals, conn.body, 'mediumblue')
        plot_kpts(ax, vals, conn.arm_r + conn.hand_r, 'fuchsia')
        
        set_axis_limits(ax,
                        x_min=-40,
                        x_max=15,
                        y_min=-15,
                        y_max=35,
                        z_min=-40,
                        z_max=20)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        plt.pause(0.1)
        if i < num_frames - 1:
            ax.cla()