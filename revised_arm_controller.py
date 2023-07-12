import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.signal import savgol_filter

import arm_controller as arm
import config_arm_prototype_horizontal as config
import keypoint_connections as conn
import keypoint_utils as kpt
from shape3d import Shape3d
        
        
# Read in keypoints from file
path = r'Extreme ROM data\weird\\'
filename = r'kpts-weird2.dat'
points, num_frames = kpt.read_kpts_3d_file(path + filename)

# Define human forearm and robot forearm lengths
scale_factor = 0.5
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
# forearm_vector = np.array([np.maximum(0.2, forearm_vector[0]),
#                            forearm_vector[1],
#                            forearm_vector[2]])

# savgol_window = 10
# savgol_degree = 1
# plat_center = savgol_filter(plat_center, savgol_window, savgol_degree)


# Find the unit vector that defines hand pronation/supination
hand_vector = (points[conn.pinky_r] - points[conn.index_r])
savgol_window = 10
savgol_degree = 1
hand_vector = savgol_filter(hand_vector, savgol_window, savgol_degree)
hand_vector /= np.linalg.norm(hand_vector, axis=0)

fig = plt.figure("t", figsize=(15, 10))
ax = fig.add_subplot(111, projection='3d')

artists = []
for i in range(num_frames):
    frame_artists = []
    
    # Define the world coordinate frame
    # origin = np.zeros(3)
    # frame = shape3d(origin,
    #                 points=np.insert(np.eye(3), 0, 0, axis=1),
    #                 connections=[[0, 1], [0, 2], [0, 3]]
    #                 )
    # frame.set_segment_colors(['red', 'green', 'blue'])
    frame = np.eye(3)
    unit = forearm_vector[:, i]
    cent = plat_center[:, [i]]
    Z = np.array([0, 0, 1])
    
    # Find the quaternion that aligns the world frame with the forearm
    axis = np.cross(unit, Z)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(np.dot(unit, Z))
    quat = arm.quaternion(axis, angle)
    quat_inv = np.copy(quat)
    quat_inv[1:] = -quat_inv[1:]
    
    # Rotate the world frame to align with the forearm
    _frame = np.insert(frame, 0, 0, axis=0)
    frame_rot = arm.quat_mult(quat, _frame)
    frame_rot = arm.quat_mult(frame_rot, quat_inv)
    frame_rot = np.delete(frame_rot, 0, axis=0)
    
    # Find the hand vector projection onto the rotated XY plane, normalized.
    hand_vector_proj = (hand_vector[:, i]
                        - np.dot(hand_vector[:, i],
                                 frame_rot[:, 2])
                        * frame_rot[:, 2]
                        )
    hand_vector_proj /= np.linalg.norm(hand_vector_proj)
    
    # Find the angle from the projected hand vector to the rotated X-axis
    angle_x_to_hand = -np.arctan2(
        np.dot(
            np.cross(frame_rot[:, 0], hand_vector_proj),
            frame_rot[:, 2]
            ),
        np.dot(frame_rot[:, 0], hand_vector_proj)
        )
    quat2 = arm.quaternion(frame_rot[:, 2], angle_x_to_hand)
    quat2_inv = np.copy(quat2)
    quat2_inv[1:] = -quat2_inv[1:]
    
    _frame_rot = np.insert(frame_rot, 0, 0, axis=0)
    frame_rot = arm.quat_mult(quat2, _frame_rot)
    frame_rot = arm.quat_mult(frame_rot, quat2_inv)
    frame_rot = np.delete(frame_rot, 0, axis=0)
    
    # hand_vector *= 30
    # hand_vector += points[conn.pinky_r]
    # frame *= 40
    # frame_rot *= 40
    # frame_rot += cent
    
    # frame_rotated = arm.orient_platform(forearm_vector[:, i], plat_center[:, [i]], frame)
    
    # plat_joints = arm.get_joint_positions(plat_center[:, [i]],
    #                                       config.plat_radius,
    #                                       config.joint_angles)

    # plat_joints_rot = arm.orient_platform(forearm_vector[:, i],
    #                                       plat_center[:, [i]],
    #                                       plat_joints)
    
    # rot_angles = np.arange(0.02, 2*np.pi, step=.02)
    # lengths = np.zeros(config.num_legs)
    # rot_direction = 1
    # for check_angle in rot_angles:
    #     check_angle *= rot_direction
    #     for j in range(config.num_legs):
    #         lengths[j] = np.sqrt(
    #             (plat_joints_rot[0, (j-1)%6] - config.basepoints[0, j])**2+
    #             (plat_joints_rot[1, (j-1)%6] - config.basepoints[1, j])**2+
    #             (plat_joints_rot[2, (j-1)%6] - config.basepoints[2, j])**2
    #             )
    #         plat_joints_rot = np.insert(plat_joints_rot, 0, 0, axis=0)
    #         _plat_center = np.insert(plat_center[:, [i]], 0, 0, axis=0)
    #         new_quat = arm.quaternion(forearm_vector[:, i], check_angle)
    #         new_quat_inv = np.copy(new_quat)
    #         new_quat_inv[1:] = -new_quat_inv[1:]
    #         new_points_rot = arm.quat_mult(new_quat, plat_joints_rot - _plat_center)
    #         new_points_rot = arm.quat_mult(new_points_rot, new_quat_inv) + _plat_center
    #         plat_joints_rot = np.delete(plat_joints_rot, 0, axis=0)
    #         new_points_rot = np.delete(new_points_rot, 0, axis=0)
    #         lengths_new = np.zeros(config.num_legs)
    #         for j in range(config.num_legs):
    #             lengths_new[j] = np.sqrt(
    #                 (new_points_rot[0, (j-1)%6] - config.basepoints[0, j])**2+
    #                 (new_points_rot[1, (j-1)%6] - config.basepoints[1, j])**2+
    #                 (new_points_rot[2, (j-1)%6] - config.basepoints[2, j])**2
    #                 )
    #         plat_joints_rot = new_points_rot
    #         epsilon = 0.1
    #         if lengths_new.sum() > lengths.sum():
    #             rot_direction *= -1
    #             if abs(lengths[0] - lengths_new[0]) < epsilon and rot_direction == -1:
    #                 lengths = lengths_new
    #                 break
        
    # lengths = lengths.round(1)
        
    # frame_artists += arm.plot_platform(ax,
    #                   forearm_vector[:, i],
    #                   plat_center[:, [i]],
    #                   config.basepoints,
    #                   plat_joints_rot,
    #                   color='slateblue')
    
    
    # Plot points and lines for the body, arm, and axes
    # frame_artists += kpt.plot_axes(ax, length=20)
    # kpt.plot_kpts(ax, points[:, :, i], conn.body, 'mediumblue')
    
    # is_length_ok = (lengths > config.min_length) & (lengths < config.max_length)
    # for _i in range(6):
    #     if is_length_ok[_i]:
    #         color='slateblue'
    #     else:
    #         color='red'
    #     frame_artists += ax.plot([config.basepoints[0, _i], plat_joints_rot[0, (_i-1)%6]],
    #                         [config.basepoints[1, _i], plat_joints_rot[1, (_i-1)%6]],
    #                         [config.basepoints[2, _i], plat_joints_rot[2, (_i-1)%6]],
    #                         color=color)
    
    # frame_artists += kpt.plot_kpts(ax,
    #                                 points[:, :, i],
    #                                 conn.arm_r + conn.hand_r,
    #                                 'thistle')
    # frame_artists += ax.plot(plat_center[0, i],
    #                          plat_center[1, i],
    #                          plat_center[2, i],
    #                          'o')
    frame_artists += ax.plot([0, forearm_vector[0, i]],
                             [0, forearm_vector[1, i]],
                             [0, forearm_vector[2, i]],
                             '.-', color='tab:orange')
    
    frame_artists += ax.plot([0, frame[0, 0]],
                             [0, frame[1, 0]],
                             [0, frame[2, 0]],
                             'o-', color='red')
    frame_artists += ax.plot([0, frame[0, 1]],
                             [0, frame[1, 1]],
                             [0, frame[2, 1]],
                             'o-', color='green')
    frame_artists += ax.plot([0, frame[0, 2]],
                             [0, frame[1, 2]],
                             [0, frame[2, 2]],
                             'o-', color='blue')
 
    frame_artists += ax.plot([0, frame_rot[0, 0]],
                              [0, frame_rot[1, 0]],
                              [0, frame_rot[2, 0]],
                              'o-', color='lightcoral')
    frame_artists += ax.plot([0, frame_rot[0, 1]],
                              [0, frame_rot[1, 1]],
                              [0, frame_rot[2, 1]],
                              'o-', color='greenyellow')
    frame_artists += ax.plot([0, frame_rot[0, 2]],
                              [0, frame_rot[1, 2]],
                              [0, frame_rot[2, 2]],
                              'o-', color='cornflowerblue')
    
    # frame_artists += ax.plot([cent[0], frame_rot[0, 0]],
    #                          [cent[1], frame_rot[1, 0]],
    #                          [cent[2], frame_rot[2, 0]],
    #                          'o-', color='lightcoral')
    # frame_artists += ax.plot([cent[0], frame_rot[0, 1]],
    #                          [cent[1], frame_rot[1, 1]],
    #                          [cent[2], frame_rot[2, 1]],
    #                          'o-', color='greenyellow')
    # frame_artists += ax.plot([cent[0], frame_rot[0, 2]],
    #                          [cent[1], frame_rot[1, 2]],
    #                          [cent[2], frame_rot[2, 2]],
    #                          'o-', color='cornflowerblue')
    
    # frame_artists += ax.plot([0, axis[0]],
    #                          [0, axis[1]],
    #                          [0, axis[2]],
    #                          '.-', color='tab:green')
    
    # frame_artists += ax.plot([points[conn.pinky_r][0, i], hand_vector[0, i]],
    #                          [points[conn.pinky_r][1, i], hand_vector[1, i]],
    #                          [points[conn.pinky_r][2, i], hand_vector[2, i]],
    #                          '.-', color='tab:red')
    frame_artists += ax.plot([0, hand_vector[0, i]],
                             [0, hand_vector[1, i]],
                             [0, hand_vector[2, i]],
                             '.-', color='fuchsia')
    
    # frame_artists += ax.plot(plat_joints[0, :],
    #                          plat_joints[1, :],
    #                          plat_joints[2, :],
    #                          'o')
    # frame_artists += ax.plot(plat_joints_rot[0, :],
    #                          plat_joints_rot[1, :],
    #                          plat_joints_rot[2, :],
    #                          'o')
    ax.set_aspect('equal')
    
    # kpt.set_axis_limits(ax,
    #                     x_min=-50,
    #                     x_max=250,
    #                     y_min=-50,
    #                     y_max=200,
    #                     z_min=-300,
    #                     z_max=100)
        
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    artists.append(frame_artists)
    # plt.pause(0.2)
    # if i < num_frames - 1:
    #     ax.cla()
ani = animation.ArtistAnimation(fig=fig, artists=artists, interval=200)
# ani.save('testfile.mp4')
plt.show()