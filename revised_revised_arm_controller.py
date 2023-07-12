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
    r'Extreme ROM data\weird\kpts-weird2.dat')

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
hand_vector = (points[conn.pinky_r] - points[conn.index_r])
hand_vector = savgol_filter(hand_vector, savgol_window, savgol_degree)
hand_vector /= np.linalg.norm(hand_vector, axis=0)

# Define the platform
platform = Shape3d(config.joints,
                   edges=[(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 0)])
platform.set_edge_colors(['brown',
                          'yellowgreen',
                          'darkturquoise',
                          'darkmagenta',
                          'deepskyblue',
                          'gold'])

fig = plt.figure(figsize=(15, 10))
ax = fig.add_subplot(111, projection='3d')
artists = []

for frame in range(num_frames):
    # Align the platform with the forearm
    platform.set_origin(forearm_point[:, frame])
    platform.align_with_vector(forearm_vector[:, frame])
    
    # Find the angle to align the platform's x-axis with hand rotation
    axes = platform.get_local_axes()
    hand_vector_proj = (hand_vector[:, frame]
                        - np.dot(hand_vector[:, frame], axes[2])
                        * axes[2]
                        )
    hand_vector_proj /= np.linalg.norm(hand_vector_proj)
    angle_x_to_hand = np.arctan2(
        np.dot(np.cross(axes[0], hand_vector_proj), axes[2]),
        np.dot(axes[0], hand_vector_proj)
        )
    
    # Align the platform's x-axis with the hand's rotation
    platform.rotate_about_z(-angle_x_to_hand)
    
    frame_artists = []
    
    frame_artists += platform.plot(ax)
    frame_artists += platform.plot_axes(ax, scale=15)
    frame_artists += ax.plot(config.basepoints[:, 0],
                             config.basepoints[:, 1],
                             config.basepoints[:, 2],
                             'o', color='darkred')
    for i in range(6):
        frame_artists += ax.plot(
            [config.basepoints[i, 0], platform.get_global_points()[(i+1)%6, 0]],
            [config.basepoints[i, 1], platform.get_global_points()[(i+1)%6, 1]],
            [config.basepoints[i, 2], platform.get_global_points()[(i+1)%6, 2]],
            color='slateblue')
    
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    artists.append(frame_artists)

ani = animation.ArtistAnimation(fig=fig, artists=artists, interval=300)
plt.show()