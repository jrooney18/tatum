import numpy as np 
import matplotlib.pyplot as plt

from scipy.signal import savgol_filter

import keypoint_utils as kpt
import keypoint_connections as conn


def scale_kpts(points, segment, target):
    ''' Scale all keypoints to set the given segment to a fixed size
    Arguments:
        points (array): points to scale
        segment(int 2-tuple): point indices for the reference segment
        target (float): target length for the reference segment
    Returns:
        points_scaled
    '''
    lengths = np.linalg.norm(points[segment[0]] - points[segment[1]], axis=0)
    scale_factor = target / lengths
    scale_matrix = np.zeros((scale_factor.size, 3, 3))
    scale_matrix[:] = np.eye(3)
    scale_matrix = scale_matrix * scale_factor.reshape(scale_factor.size, 1, 1)
    points_scaled = np.matmul(scale_matrix, points.transpose())
    points_scaled = points_scaled.transpose()
    
    return points_scaled
  

path = 'Keypoints Data\\'
filename = 'kpts-3d.dat'
points, num_frames = kpt.read_kpts_3d_file(path + filename)

points = kpt.orient_kpts(points)

forearm_length = 0.235 # meters
points = scale_kpts(points,
                    (conn.wrist_r, conn.elbow_r),
                    forearm_length)

# Add a point for the robot's end effector
robot_length = 0.13 # meters
dist_elbow2effector = forearm_length - robot_length
elbow_wrist_unit = ((points[conn.wrist_r] - points[conn.elbow_r]) /
                    forearm_length)
end_eff = ((elbow_wrist_unit * dist_elbow2effector) + points[conn.elbow_r])
points = np.concatenate((points, end_eff.reshape(1, 3, 60)), axis=0)
end_eff_idx = 33

# Find unit vectors to end effector and wrist
end_eff_unit = kpt.unit_vector(points[end_eff_idx])
wrist_unit = kpt.unit_vector(points[conn.wrist_r] - points[end_eff_idx])

# Find angles of unit vectors relative to -Y in the XY plane and the YZ plane
end_eff_angle_xy = np.arctan2(end_eff_unit[0], -end_eff_unit[1])
end_eff_angle_yz = np.arctan2(end_eff_unit[2], -end_eff_unit[1])
end_eff_angle_xz = np.arctan2(end_eff_unit[2], end_eff_unit[0])
wrist_angle_xy = np.arctan2(wrist_unit[0], -wrist_unit[1])
wrist_angle_yz = np.arctan2(wrist_unit[2], -wrist_unit[1])
wrist_angle_xz = np.arctan2(wrist_unit[2], wrist_unit[0])

# Subtract to find angle of forearm platform relative to end effector vector
platform_angle_xy = wrist_angle_xy - end_eff_angle_xy
platform_angle_yz = wrist_angle_yz - end_eff_angle_yz
platform_angle_xz = wrist_angle_xz - end_eff_angle_xz

# platform_angle_xy = np.unwrap(platform_angle_xy)
# platform_angle_yz = np.unwrap(platform_angle_yz)
platform_angle_xy = np.rad2deg(platform_angle_xy)
platform_angle_yz = np.rad2deg(platform_angle_yz)
platform_angle_xz = np.rad2deg(platform_angle_xz)

angle_xy_filt = savgol_filter(platform_angle_xy, 15, 6)
angle_yz_filt = savgol_filter(platform_angle_yz, 15, 6)
angle_xz_filt = savgol_filter(platform_angle_xz, 15, 6)

fig = plt.figure("t", figsize=(15, 10))
ax1 = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122)

num_frames = 32
for i in range(num_frames):
    vals = points[:, :, i]
    
    # Plot points and lines for the body, arm, and axes
    kpt.plot_axes(ax1, length=0.1)
    kpt.plot_kpts(ax1, vals, conn.body, 'mediumblue')
    kpt.plot_kpts(ax1, vals, conn.arm_r, 'pink')
    kpt.plot_kpts(ax1, vals, conn.arm_robot, 'fuchsia')

    ax1.plot(vals[end_eff_idx, 0],
              vals[end_eff_idx, 1],
              vals[end_eff_idx, 2],
              'o', color='lime')
    
    kpt.set_axis_limits(ax1,
                        x_min=-0.1,
                        x_max=0.4,
                        y_min=-0.3,
                        y_max=0.4,
                        z_min=-0.4,
                        z_max=0.3)
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    
    # Plot angles
    ax2.plot(platform_angle_xy[:i], 'r')
    ax2.plot(platform_angle_yz[:i], 'g')
    ax2.plot(platform_angle_xz[:i], 'b')
    ax2.set_xlim(0, 60)
    ax2.legend(('From -Y in XY plane',
                'From -Y in YZ plane',
                'From +X in XZ plane'))
    ax2.set_xlabel('Frame number')
    ax2.set_ylabel('Angle (degrees)')
    
    plt.pause(0.01)
    if i < num_frames - 1:
        ax1.cla()
        ax2.cla()
    else:
        ax2.cla()
        ax2.plot(platform_angle_xy, color='pink')
        ax2.plot(platform_angle_yz, color='lightgreen')
        ax2.plot(platform_angle_xz, color='lightblue')
        ax2.plot(angle_xy_filt, 'r')
        ax2.plot(angle_yz_filt, 'g')
        ax2.plot(angle_xz_filt, 'b')
        ax2.legend(('From -Y in XY plane',
                    'From -Y in YZ plane',
                    'From +X in XZ plane'))
        ax2.set_xlabel('Frame number')
        ax2.set_ylabel('Angle (degrees)')