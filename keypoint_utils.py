import numpy as np 
import matplotlib.pyplot as plt

import keypoint_connections as conn

def read_kpts_3d_file(filename):
    ''' Reads mediapipe keyframes into numpy array
    Arguments: filename (str): name of the file containing keypoints
        (in working directory)
    Returns: 
        points (np array): 33 x 3 x n numpy array, containing 33 (x,y,z) 
            coordinate points for each of the n frames in the file
        num_frames (int): number of frames
    '''
    with open(filename, 'r') as f:
        lines = f.readlines()
        num_frames = len(lines)

    points = np.zeros((33, 3, num_frames))

    # Split each frame's pts, reshape into 33x3 float array, and store
    for i, line in enumerate(lines):
        tokens = line.strip().split()
        kpt = [float(token) for token in tokens]
        kpt = np.array(kpt).reshape((33, 3))
        points[:, :, i] = kpt

    num_frames = points.shape[2]
    return points, num_frames

def unit_vector(point):
    ''' Returns [x, y, z] unit vector aligned from [0, 0, 0] to input point '''
    magnitude = np.linalg.norm(point, axis=0)
    unit = point / magnitude
    return unit

def rotation_X(angle):
    ''' Returns 3x3 matrix for rotation in radians about the global X-axis '''
    cos = np.cos(angle)
    sin = np.sin(angle)
    
    rot_X = np.zeros((angle.size, 3, 3))
    rot_X[:, (1, 2), (1, 2)] = cos
    rot_X[:, 1, 2] = -sin
    rot_X[:, 2, 1] = sin
    rot_X[:, 0, 0] = 1
    return rot_X

def rotation_Y(angle):
    ''' Returns 3x3 matrix for rotation in radians about the global Y-axis '''
    cos = np.cos(angle)
    sin = np.sin(angle)
    
    rot_Y = np.zeros((angle.size, 3, 3))
    rot_Y[:, 0, 0] = cos
    rot_Y[:, 0, 2] = -sin
    rot_Y[:, 2, 0] = sin
    rot_Y[:, 2, 2] = cos
    rot_Y[:, 1, 1] = 1
    return rot_Y

def rotation_Z(angle):
    ''' Returns 3x3 matrix for rotation in radians about the global Z-axis '''
    cos = np.cos(angle)
    sin = np.sin(angle)

    rot_Z = np.zeros((angle.size, 3, 3))
    rot_Z[:, 0, 0] = cos
    rot_Z[:, 0, 1] = sin
    rot_Z[:, 1, 0] = -sin
    rot_Z[:, 1, 1] = cos
    rot_Z[:, 2, 2] = 1
    return rot_Z

def orient_kpts(points):
    ''' Orients Mediapipe keypoints to place the right shoulder at the origin,
        align the shoulder axis with +Y-axis, and align the vertical body axis
        with the Z-axis.
    Arguments:
        points (array): Keypoints for n frames, shape=(33, 3, 60)
    Returns: points_oriented (array)
    '''
    
    # Translate all points to place the right shoulder at (0, 0, 0)
    points_trans = points - points[conn.shoulder_r]
    
    # Rotate points to align the shoulder axis with the +Y-axis
    shoulder_unit = unit_vector(points_trans[conn.shoulder_l])
    angle_z = np.pi/2 - np.arctan2(shoulder_unit[1], shoulder_unit[0])
    angle_y = np.arctan2(shoulder_unit[2], shoulder_unit[0])
    
    rot_Z = rotation_Z(-angle_z)
    rot_Y = rotation_Y(-angle_y)
    rot = np.matmul(rot_Z, rot_Y)
    points_rot = np.matmul(rot, points_trans.transpose())
    points_rot = points_rot.transpose()
    
    # Rotate to align the vertical body axis with the z-axis
    hip_unit = unit_vector(points_rot[conn.hip_r])
    angle_hip = np.pi/2 + np.arctan2(hip_unit[2], hip_unit[0])
    rot_Y2 = rotation_Y(-angle_hip)
    points_oriented = np.matmul(rot_Y2, points_rot.transpose())
    points_oriented = points_oriented.transpose()
    
    return points_oriented

def plot_axes(axes, length=1):
    ''' Plots X,Y,Z vectors of given length (default 1) on the input axes '''
    axes.plot([0, length], [0, 0], [0, 0], 'r')
    axes.plot([0, 0], [0, length], [0, 0], 'g')
    axes.plot([0, 0], [0, 0], [0, length], 'b')
    axes.plot(length, 0, 0, 'ro')
    axes.plot(0, length, 0, 'go')
    axes.plot(0, 0, length, 'bo')
    return None

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
    for _c in connections:
        axes.plot([points[_c[0],0], points[_c[1],0]],
                [points[_c[0],1], points[_c[1],1]],
                [points[_c[0],2], points[_c[1],2]],
                'o-', color=color)
    return None

def set_axis_limits(axes, x_min, x_max, y_min, y_max, z_min, z_max):
    ''' Set limits for the x-, y-, and z-axes of an axes object in one pass '''
    axes.set_xlim3d(x_min, x_max)
    axes.set_ylim3d(y_min, y_max)
    axes.set_zlim3d(z_min, z_max)
    axes.set_aspect('equal')
    return None


if __name__ == '__main__':
    path = r'Keypoints Data\tpose_bend\\'
    filename = r'kpts-tpose_bend0.dat'
    points, num_frames = read_kpts_3d_file(path + filename)
    
    points_oriented = orient_kpts(points)
    
    fig = plt.figure("t", figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    for i in range(num_frames):
        vals = points_oriented[:, :, i]
        
        # Plot points and lines for the body, arm, and axes
        plot_axes(ax, length=2)
        plot_kpts(ax, vals, conn.body, 'mediumblue')
        plot_kpts(ax, vals, conn.arm_r + conn.hand_r, 'fuchsia')
        
        set_axis_limits(ax,
                        x_min=-5,
                        x_max=10,
                        y_min=-12,
                        y_max=16,
                        z_min=-20,
                        z_max=10)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        plt.pause(0.05)
        if i < num_frames - 1:
            ax.cla()