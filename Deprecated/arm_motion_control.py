import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.signal import savgol_filter

import arm_controller as arm
import config_arm_prototype_horizontal as config
import keypoint_connections as conn
import keypoint_utils as kpt


# Read in keypoints from file
path = r'Extreme ROM data\weird\\'
filename = r'kpts-weird2.dat'
points, num_frames = kpt.read_kpts_3d_file(path + filename)

# Define human forearm and robot forearm lengths
scale_factor = 0.2
len_forearm = 235 * scale_factor # mm
len_robot = 130 * scale_factor # mm

# Orient keypoints with shoulder at origin and scale based on forearm length
points = kpt.orient_kpts(points)
points = kpt.scale_kpts(points, (conn.wrist_r, conn.elbow_r), len_forearm)

# Offset points to robot reference frame
robot_origin = np.array([-220, -20, -100])
points = points - robot_origin.reshape([1, 3, 1])

# Add a point on the forearm for the robot's platform
dist_elbow2platform = len_forearm - len_robot
forearm_vector = (points[conn.wrist_r] - points[conn.elbow_r]) / len_forearm
plat_center = forearm_vector * dist_elbow2platform + points[conn.elbow_r]

# Clip forearm vector to always have x > 0
forearm_vector = np.array([np.maximum(0.2, forearm_vector[0]),
                           forearm_vector[1],
                           forearm_vector[2]])

savgol_window = 10
savgol_degree = 1
plat_center = savgol_filter(plat_center, savgol_window, savgol_degree)


fig = plt.figure("t", figsize=(15, 10))
ax = fig.add_subplot(111, projection='3d')

artists = []
for i in range(num_frames):
    frame_artists = []
    plat_joints = arm.get_joint_positions(plat_center[:, [i]],
                                          config.plat_radius,
                                          config.joint_angles)

    plat_joints_rot = arm.orient_platform(forearm_vector[:, i],
                                          plat_center[:, [i]],
                                          plat_joints)
    
    lengths = np.zeros(config.num_legs)
    for j in range(config.num_legs):
        lengths[j] = np.sqrt(
            (plat_joints_rot[0, (j-1)%6] - config.basepoints[0, j])**2+
            (plat_joints_rot[1, (j-1)%6] - config.basepoints[1, j])**2+
            (plat_joints_rot[2, (j-1)%6] - config.basepoints[2, j])**2
            )
    lengths = lengths.round(1)
        
    frame_artists += arm.plot_platform(ax,
                      forearm_vector[:, i],
                      plat_center[:, [i]],
                      config.basepoints,
                      plat_joints_rot,
                      color='slateblue')
    
    
    # Plot points and lines for the body, arm, and axes
    frame_artists += kpt.plot_axes(ax, length=20)
    # kpt.plot_kpts(ax, points[:, :, i], conn.body, 'mediumblue')
    
    is_length_ok = (lengths > config.min_length) & (lengths < config.max_length)
    for _i in range(6):
        if is_length_ok[_i]:
            color='slateblue'
        else:
            color='red'
        frame_artists += ax.plot([config.basepoints[0, _i], plat_joints_rot[0, (_i-1)%6]],
                           [config.basepoints[1, _i], plat_joints_rot[1, (_i-1)%6]],
                           [config.basepoints[2, _i], plat_joints_rot[2, (_i-1)%6]],
                           color=color)
    
    frame_artists += kpt.plot_kpts(ax,
                                   points[:, :, i],
                                   conn.arm_r + conn.hand_r,
                                   'fuchsia')
    
    kpt.set_axis_limits(ax,
                        x_min=-50,
                        x_max=250,
                        y_min=-50,
                        y_max=200,
                        z_min=-300,
                        z_max=100)
        
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    artists.append(frame_artists)
    # plt.pause(0.2)
    # if i < num_frames - 1:
    #     ax.cla()
ani = animation.ArtistAnimation(fig=fig, artists=artists, interval=100)
# ani.save('testfile.mp4')
plt.show()