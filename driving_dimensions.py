import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

import keypoint_connections as conn
from shape3d import Shape3d
from trajectory import Trajectory


###  Define constants
num_legs      = 6
plat_rad      = 42  # mm
joint_angle   = 24  # degrees
joint_angle_0 = 108  # degrees

base_angles   = np.deg2rad([joint_angle_0,
                            joint_angle_0 + joint_angle,
                            joint_angle_0 + 120,
                            joint_angle_0 + 120 + joint_angle,
                            joint_angle_0 + 240,
                            joint_angle_0 + 240 + joint_angle
                            ])
plat_angles   = base_angles + np.pi/3


fig = plt.figure(figsize=(15, 10))
ax = fig.add_subplot(projection='3d')
artists = []

num_radii = 7
num_angles = 1
num_offsets = 3

traj = Trajectory(r'Extreme ROM data\Mohith_Nicole_17_07_2023\3D_files\Extremes\2.npy',
                  robot_length=130,
                  forearm_length=235
                  )
traj_offset = np.array([-100, -200, -100])
traj.offset(traj_offset)

# Initialize data storage
length_data = np.zeros([num_radii,
                        num_angles,
                        num_offsets, 
                        traj.num_frames,
                        6])

# length_data = np.load('length_data.npy')
# print('Working...', end='')
# for rad_idx, base_rad in enumerate(np.linspace(50, 350, num_radii)):
for rad_idx, base_rad in enumerate(np.linspace(150, 150, 1)):
    ###  Locate base and platform points
    base_points = np.array([base_rad * np.cos(base_angles),
                            base_rad * np.sin(base_angles),
                            np.zeros(6)]).T
    plat_points = np.array([plat_rad * np.cos(plat_angles),
                            plat_rad * np.sin(plat_angles),
                            np.zeros(6)]).T
    
    ### Create base and platform objects
    edges = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 0)]
    base = Shape3d(base_points, edges)
    base.set_edge_colors('tab:blue')
    platform = Shape3d(plat_points, edges)
    platform.set_edge_colors('tab:orange')
    
    ### Set neutral angle relative to body frame and starting base offset 
    # for angle_idx, angle in enumerate(np.linspace(0, 90, num_angles)):
    for angle_idx, angle in enumerate(np.linspace(0, 0, 1)):    
        angle = np.deg2rad(angle)
        # for offset_idx, offset in enumerate(np.linspace(-400, -500, num_offsets)):
        for offset_idx, offset in enumerate(np.linspace(-400, -400, 1)):
            # base_offset = np.array([offset * np.cos(angle),
            #                     offset * np.sin(angle),
            #                     0])
            base_offset = np.array([offset, 70, 20])
            base.origin = base_offset
            base.align_with_vector(np.array([1, 0, 0]))
            for frame in range(1):
                platform.align_with_vector(traj.vector[frame])
                platform.origin = traj.location[frame]
                
                ### Align rotated base and platform with the global z-axis
                angle_to_vert = np.dot(base.local_axes[1],
                                        np.array([0, 0, 1]))
                base.rotate_about_z(-angle_to_vert)
                platform.rotate_about_z(-angle_to_vert)
                
                ### Define the legs
                leg_connections = [(0, 11),
                                    (1, 6),
                                    (2, 7),
                                    (3, 8),
                                    (4, 9),
                                    (5, 10)]
                leg_pairs = [np.array([5, 0]),
                             np.array([1, 2]),
                             np.array([3, 4])]
                
                legs = Shape3d(np.concatenate([base.global_points,
                                               platform.global_points]),
                                edges=leg_connections)
                
                
                ###### Platform rotational alignment ##########################
                
                # Find the leg pair with the shortest combined length
                shortest_pair = legs.edge_lengths[leg_pairs].sum(axis=1).argmin()
                
                # Create array of angles to rotate through
                rot_increment = 0.02
                rot_direction = 1
                # epsilon = 0.1
                lengths = np.zeros(3)
                
                # Store the initial pre-rotation leg length sum
                lengths[0] = legs.edge_lengths.sum()/6
                
                # Rotate by one increment and store the new leg length sum
                platform.rotate_about_z(rot_increment)
                legs = Shape3d(np.concatenate([base.global_points,
                                                platform.global_points]),
                                edges=leg_connections)
                lengths[1] = legs.edge_lengths.sum()/6
                
                while True:
                    # Rotate again and store the leg length sum
                    platform.rotate_about_z(rot_increment * rot_direction)
                    legs = Shape3d(np.concatenate([base.global_points,
                                                    platform.global_points]),
                                    edges=leg_connections)
                    lengths[2] = legs.edge_lengths.sum()/6
                    
                    # Check if the rotation direction is correct
                    if lengths[2] > lengths[1] and lengths[1] > lengths[0]:
                        rot_direction *= -1
                    
                    # Check if the middle value gives the shortest length
                    if lengths[2] > lengths[1] and lengths[0] > lengths[1]:
                        # If so, rotate the platform back an increment and exit
                        rot_direction *= -1
                        platform.rotate_about_z(rot_increment * rot_direction)
                        legs = Shape3d(np.concatenate(
                                            [base.global_points,
                                             platform.global_points]),
                                        edges=leg_connections)
                        break
                    lengths[0:2] = lengths[1:3]
                ###############################################################
                
                legs.set_edge_colors(['tab:red', 'tab:green',
                                      'tab:green', 'tab:blue',
                                      'tab:blue', 'tab:red'])
                
                length_data[rad_idx,
                            angle_idx,
                            offset_idx,
                            frame] = legs.edge_lengths 
                
                frame_artists = []
                ax.cla()
                
                arm = Shape3d(traj.points[frame] - traj_offset,
                              ((1, 2), (2, 3), (3, 4),
                                (1, 5), (5, 6), (6, 7),
                                (0, 1)))
                arm.set_edge_colors('mediumpurple')
                frame_artists += arm.plot(ax)
                
                frame_artists += base.plot(ax)
                frame_artists += platform.plot(ax)
                frame_artists += legs.plot(ax)
                
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_xlim([-500, 100])
                ax.set_ylim([-500, 100])
                ax.set_zlim([-400, 400])
                ax.set_aspect('equal')
                artists.append(frame_artists)
                plt.pause(0.03)
            maxes = length_data[0, 0, 0].max(axis=0)
            mins = length_data[0, 0, 0].min(axis=0)
            print(f'Max lengths: {maxes.round(1)}')
            print(f'Min lengths: {mins.round(1)}')
            print(f'Difference : {np.round(maxes - mins, 1)}')
            print(f'Remaining  : {np.round(mins - (maxes - mins), 1)}')
    # print(f'{rad_idx * 10 + 10}%...', end='')

# np.save('length_data_2.npy', length_data)
# ani = animation.ArtistAnimation(fig=fig, artists=artists, interval=100)
# ani.save('without_rotation.avi')

