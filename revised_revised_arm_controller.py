import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from scipy.signal import savgol_filter

import config_arm_prototype_horizontal as config
import keypoint_connections as conn
import keypoint_utils as kpt
from shape3d import Shape3d


# Read in keypoints from file
points, num_frames = kpt.read_kpts_3d_file(
    r'Extreme ROM data\extremes\kpts_3D_extremes_1.dat')

# Orient keypoints with shoulder at origin and scale based on forearm length
points = kpt.orient_kpts(points)
points = kpt.scale_kpts(points,
                        (conn.wrist_r, conn.elbow_r),
                        config.len_forearm)

# Offset points to robot reference frame
robot_origin = np.array([-220, -20, -100])
points = points - robot_origin.reshape([1, 3, 1])

# Add a point on the forearm for the robot's platform
dist_elbow2platform = config.len_forearm - config.len_robot
forearm_vector = ((points[conn.wrist_r] - points[conn.elbow_r])
                  / config.len_forearm)
forearm_point = forearm_vector * dist_elbow2platform + points[conn.elbow_r]
savgol_window = 10
savgol_degree = 1
forearm_point = savgol_filter(forearm_point, savgol_window, savgol_degree)

# Find the unit vector that defines hand pronation/supination
# hand_vector = (points[conn.pinky_r] - points[conn.index_r])
# hand_vector = savgol_filter(hand_vector, savgol_window, savgol_degree)
# hand_vector /= np.linalg.norm(hand_vector, axis=0)

# Define the platform
platform = Shape3d(config.joints,
                   edges=[(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 0)])
# platform.set_edge_colors(['brown',
#                           'yellowgreen',
#                           'darkturquoise',
#                           'darkmagenta',
#                           'deepskyblue',
#                           'gold'])
platform.set_edge_colors('purple')

fig = plt.figure(figsize=(15, 10))
ax = fig.add_subplot(111, projection='3d')
artists = []

# for frame in range(0, 11):
for frame in range(num_frames):
    # Align the platform with the forearm
    platform.set_origin(forearm_point[:, frame])
    platform.align_with_vector(forearm_vector[:, frame])
    
    # Find the angle to align the platform's x-axis with hand rotation
    axes = platform.get_local_axes()
    base_vector = config.basepoints[0] - config.basepoints[1]
    base_vector /= np.linalg.norm(base_vector)
    cross = np.cross(axes[2], base_vector)
    cross /= np.linalg.norm(cross)
    angle = np.arctan2(
        np.dot(np.cross(axes[1], cross), axes[2]),
        np.dot(axes[1], cross)
        )
    platform.rotate_about_z(-angle)
    ######### Aligns platform x-axis with back of hand ########################
    # hand_vector_proj = (hand_vector[:, frame]
    #                     - np.dot(hand_vector[:, frame], axes[2])
    #                     * axes[2]
    #                     )
    # hand_vector_proj /= np.linalg.norm(hand_vector_proj)
    # angle_x_to_hand = np.arctan2(
    #     np.dot(np.cross(axes[0], hand_vector_proj), axes[2]),
    #     np.dot(axes[0], hand_vector_proj)
    #     )
    
    # Align the platform's x-axis with the hand's rotation+
    # platform.rotate_about_z(-(angle_x_to_hand - .785))
    ###########################################################################
    
    plat_points = platform.get_global_points()
    
    # Calculate leg lengths 
    lengths = np.zeros(config.num_legs)
    for j in range(config.num_legs):
        lengths[j] = np.sqrt(
            (plat_points[(j+1)%6, 0] - config.basepoints[j, 0])**2+
            (plat_points[(j+1)%6, 1] - config.basepoints[j, 1])**2+
            (plat_points[(j+1)%6, 2] - config.basepoints[j, 2])**2
            )
    lengths = lengths.round(1)
    
    frame_artists = []

    # Plot platform, platform axes, and base points
    frame_artists += platform.plot(ax)
    frame_artists += platform.plot_axes(ax, scale=15)
    frame_artists += ax.plot(config.basepoints[:, 0],
                             config.basepoints[:, 1],
                             config.basepoints[:, 2],
                             'o', color='darkred')
    
    frame_artists += ax.plot([0, base_vector[0] * 20],
                             [0, base_vector[1] * 20],
                             [0, base_vector[2] * 20],
                             '.-', color='red')
    frame_artists += ax.plot([0, axes[2, 0] * 20],
                             [0, axes[2, 1] * 20],
                             [0, axes[2, 2] * 20],
                             '.-', color='blue')
    frame_artists += ax.plot([0, cross[0] * 20],
                             [0, cross[1] * 20],
                             [0, cross[2] * 20],
                             '.-', color='green')
    # frame_artists += kpt.plot_kpts(ax, points[:, :, frame], conn.arm_r + conn.hand_r, 'fuchsia')
    # frame_artists += ax.plot([0, plat_x[0] * 20],
    #                          [0, plat_x[1] * 20],
    #                          [0, plat_x[2] * 20],
    #                          '.-', color='blue')
    # frame_artists += ax.plot([0, plat_x_proj[0] * 20],
    #                          [0, plat_x_proj[1] * 20],
    #                          [0, plat_x_proj[2] * 20],
    #                          '.-', color='green')
    
    # Plot legs
    for i in range(6):
        if lengths[i] < config.min_length:
            color='magenta'
        elif lengths[i] > config.max_length:
            color='red'
        else:
            color='green'
        # color='violet'
        frame_artists += ax.plot(
            [config.basepoints[i, 0], plat_points[(i+1)%6, 0]],
            [config.basepoints[i, 1], plat_points[(i+1)%6, 1]],
            [config.basepoints[i, 2], plat_points[(i+1)%6, 2]],
            color=color)
    
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    artists.append(frame_artists)

ani = animation.ArtistAnimation(fig=fig, artists=artists, interval=100)
plt.show()
