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


fig = plt.figure()
ax = fig.add_subplot(projection='3d')
artists = []

num_radii = 10
num_angles = 10
num_offsets = 19

traj = Trajectory(r'Extreme ROM data\extremes\kpts_3D_extremes_0.dat',
                  ref_segment=[conn.elbow_r, conn.wrist_r],
                  ref_length=235,
                  robot_length=130,
                  forearm_length=235
                  )
traj.offset(np.array([1, 1, 1]))

for base_rad in np.linspace(50, 500, num_radii):
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
    for angle in np.linspace(0, 90, num_angles):
        angle = np.deg2rad(angle)
        for offset in np.linspace(-50, -500, num_offsets):
            vector = np.array([offset * np.cos(angle),
                               offset * np.sin(angle),
                               0])
            base.set_origin(vector)
            base.align_with_vector(vector)
            for frame in range(traj.num_frames):
                platform.align_with_vector(traj.vector[frame])
                platform.set_origin(traj.location[frame])
                
                ### Align rotated base and platform with the global z-axis
                angle_to_vert = np.dot(base.get_local_axes()[1],
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
                legs = Shape3d(np.concatenate([base.get_global_points(),
                                               platform.get_global_points()]),
                               edges=leg_connections)
                legs.set_edge_colors('tab:red')
                
                frame_artists = []
                ax.cla()
                
                frame_artists += base.plot(ax)
                frame_artists += platform.plot(ax)
                frame_artists += legs.plot(ax)
                frame_artists += ax.plot([0, vector[0]],
                                         [0, vector[1]],
                                         [0, vector[2]],
                                         color='tab:green')
                
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_xlim([-500, 10])
                ax.set_ylim([-500, 0])
                ax.set_zlim([-400, 400])
                ax.set_aspect('equal')
                artists.append(frame_artists)
                plt.pause(0.03)
            
# ani = animation.ArtistAnimation(fig=fig, artists=artists, interval=100)