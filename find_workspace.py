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
  

path = 'Keypoints Data\Autumn\\'
filename = 'kpts_3d_Autumn_0.dat'
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
wrist_unit = kpt.unit_vector(points[conn.wrist_r] - points[end_eff_idx])

# Find angles of wrist unit vector relative to:
#     The +X-axis in the XY plane
#     The +X-axis in the XZ plane
#     The +Z-axis in the YZ plane
wrist_angle_xy = np.arctan2(wrist_unit[1], wrist_unit[0])
wrist_angle_xz = np.arctan2(wrist_unit[2], wrist_unit[0])
wrist_angle_yz = np.arctan2(wrist_unit[1], wrist_unit[2])

# wrist_angle_xy = np.unwrap(wrist_angle_xy)
# wrist_angle_xz = np.unwrap(wrist_angle_xz)
wrist_angle_yz = np.unwrap(wrist_angle_yz)
wrist_angle_xy = np.rad2deg(wrist_angle_xy)
wrist_angle_xz = np.rad2deg(wrist_angle_xz)
wrist_angle_yz = np.rad2deg(wrist_angle_yz) % 360

savgol_window = 7
savgol_degree = 1
x_filt = savgol_filter(end_eff[0], savgol_window, savgol_degree)
y_filt = savgol_filter(end_eff[1], savgol_window, savgol_degree)
z_filt = savgol_filter(end_eff[2], savgol_window, savgol_degree)
angle_xy_filt = savgol_filter(wrist_angle_xy, savgol_window, savgol_degree)
angle_xz_filt = savgol_filter(wrist_angle_xz, savgol_window, savgol_degree)
angle_yz_filt = savgol_filter(wrist_angle_yz, savgol_window, savgol_degree)

fig = plt.figure("t", figsize=(15, 10))
ax1 = fig.add_subplot(131, projection='3d')
ax2 = fig.add_subplot(132)
ax3 = fig.add_subplot(133)

# num_frames = 24
for i in range(num_frames):
    # Clear axes
    ax1.cla()
    ax2.cla()
    ax3.cla()
    
    # Plot points and lines for the body, arm, and axes
    kpt.plot_axes(ax1, length=0.1)
    kpt.plot_kpts(ax1, points[:, :, i], conn.body, 'mediumblue')
    kpt.plot_kpts(ax1, points[:, :, i], conn.arm_r, 'pink')
    kpt.plot_kpts(ax1, points[:, :, i], conn.arm_robot, 'fuchsia')

    ax1.plot(points[:, :, i][end_eff_idx, 0],
             points[:, :, i][end_eff_idx, 1],
             points[:, :, i][end_eff_idx, 2],
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

    # Plot positions
    ax2.plot(points[end_eff_idx, 0, :i], 'r')
    ax2.plot(points[end_eff_idx, 1, :i], 'g')
    ax2.plot(points[end_eff_idx, 2, :i], 'b')
    ax2.set_xlim(0, num_frames)
    ax2.legend(('X-coordinate',
                'Y-coordinate',
                'Z-coordinate'))
    ax2.set_xlabel('Frame number')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Positions')
    
    # Plot angles
    ax3.plot(wrist_angle_xy[:i], 'r')
    ax3.plot(wrist_angle_xz[:i], 'g')
    ax3.plot(wrist_angle_yz[:i], 'b')
    ax3.set_xlim(0, num_frames)
    ax3.legend(('From +X in XY plane',
                'From +X in XZ plane',
                'From +Z in YZ plane'))
    ax3.set_xlabel('Frame number')
    ax3.set_ylabel('Angle (degrees)')
    ax3.set_title('Angles')

    plt.pause(0.01)

ax2.cla()
ax2.plot(points[end_eff_idx, 0, :i], color='pink')
ax2.plot(points[end_eff_idx, 1, :i], color='lightgreen')
ax2.plot(points[end_eff_idx, 2, :i], color='lightblue')
ax2.plot(x_filt[:i], 'r')
ax2.plot(y_filt[:i], 'g')
ax2.plot(z_filt[:i], 'b')
ax2.set_xlim(0, num_frames)
ax2.legend(('X-coordinate',
            'Y-coordinate',
            'Z-coordinate'))
ax2.set_xlabel('Frame number')
ax2.set_ylabel('Position (m)')
ax2.set_title('Positions')

ax3.cla()
ax3.plot(wrist_angle_xy[:i], color='pink')
ax3.plot(wrist_angle_xz[:i], color='lightgreen')
ax3.plot(wrist_angle_yz[:i], color='lightblue')
ax3.plot(angle_xy_filt[:i], 'r')
ax3.plot(angle_xz_filt[:i], 'g')
ax3.plot(angle_yz_filt[:i], 'b')
ax3.set_xlim(0, num_frames)
ax3.legend(('From +X in XY plane',
            'From +X in XZ plane',
            'From +Z in YZ plane'))
ax3.set_xlabel('Frame number')
ax3.set_ylabel('Angle (degrees)')
ax3.set_title('Angles')