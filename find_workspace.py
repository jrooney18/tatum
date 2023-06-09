import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np 
import os

from scipy.signal import savgol_filter

import keypoint_utils as kpt
import keypoint_connections as conn


def remove_discont(angles):
    ''' Remove discontinuities in periodic degree data, akin to np.unwrap '''
    for i in range(angles.size - 1):
        if angles[i] - angles[i+1] > 150:
            angles[i+1] = angles[i+1] + 360
        elif angles[i] - angles[i+1] < -150:
            angles[i+1] = angles[i+1] - 360
    return None


# fig, axes = plt.subplots(3, 2, figsize=(15, 10))
# fig.set_facecolor('#e9edee')
# fig.suptitle('Forearm displacements and angles for extreme RoM signs',
#              fontsize=16, fontweight='bold')
# axes[0, 0].set_facecolor('#e9edee')
# axes[1, 0].set_facecolor('#e9edee')
# axes[2, 0].set_facecolor('#e9edee')
# axes[0, 1].set_facecolor('#e9edee')
# axes[1, 1].set_facecolor('#e9edee')
# axes[2, 1].set_facecolor('#e9edee')

numfiles = 36
pts_per_file = 60
data = np.zeros([numfiles * pts_per_file, 3, 1])

directory = r'Extreme ROM Data\\'
for j, file in enumerate(os.listdir(directory)):
    sign = os.fsdecode(file)
    # plt.pause(1)
    for i in range(3):
        path = directory + sign
        filename = '\kpts-' + sign + str(i) + '.dat'
        points, num_frames = kpt.read_kpts_3d_file(path + filename)
        
        points = kpt.orient_kpts(points)
        
        forearm_length = 0.235 # meters
        points = kpt.scale_kpts(points,
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
        
        startpoint = j * 60 + i * 60
        data[startpoint:startpoint + 60] = end_eff.T.reshape([60, 3, 1])

data = data * 1000
cdata = np.arange(12).repeat(180)
ax = plt.figure(figsize=(12,10)).add_subplot(projection='3d')
ax.scatter(data[:, 0], data[:, 1], data[:, 2], '.', c=cdata, cmap='tab20')
kpt.plot_axes(ax, 100)
ax.set_aspect('equal')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

        
        # # Find unit vectors to end effector and wrist
        # wrist_unit = kpt.unit_vector(points[conn.wrist_r] - points[end_eff_idx])
        
        # # Find angles of wrist unit vector relative to:
        # #     The +X-axis in the XY plane
        # #     The +X-axis in the XZ plane
        # #     The +Z-axis in the YZ plane
        # angle_xy = np.arctan2(wrist_unit[1], wrist_unit[0])
        # angle_xz = np.arctan2(wrist_unit[2], wrist_unit[0])
        # angle_yz = np.arctan2(wrist_unit[1], wrist_unit[2])
        
        # # Convert angles to degrees and remove periodic discontinuities
        # angle_xy = np.rad2deg(angle_xy)
        # angle_xz = np.rad2deg(angle_xz)
        # angle_yz = np.rad2deg(angle_yz)
        
        # remove_discont(angle_xy)
        # remove_discont(angle_xz)
        # remove_discont(angle_yz)
        
        # # Filter all data using a Savitzky-Golay filter 
        # savgol_window = 3
        # savgol_degree = 1
        # x_filt = savgol_filter(end_eff[0], savgol_window, savgol_degree)
        # y_filt = savgol_filter(end_eff[1], savgol_window, savgol_degree)
        # z_filt = savgol_filter(end_eff[2], savgol_window, savgol_degree)
        # angle_xy_filt = savgol_filter(angle_xy, savgol_window, savgol_degree)
        # angle_xz_filt = savgol_filter(angle_xz, savgol_window, savgol_degree)
        # angle_yz_filt = savgol_filter(angle_yz, savgol_window, savgol_degree)
        
        # # Shift all data to start at zero
        # x_filt = x_filt - x_filt[0]
        # y_filt = y_filt - y_filt[0]
        # z_filt = z_filt - z_filt[0]
        # angle_xy_filt = angle_xy_filt - angle_xy_filt[0]
        # angle_xz_filt = angle_xz_filt - angle_xz_filt[0]
        # angle_yz_filt = angle_yz_filt - angle_yz_filt[0]
    
        # # Plot displacement and angle data 
        # axes[0, 0].plot(x_filt, 'r')
        # axes[1, 0].plot(y_filt, 'g')
        # axes[2, 0].plot(z_filt, 'b')
        
        # axes[0, 1].plot(angle_xy_filt, 'r')
        # axes[1, 1].plot(angle_xz_filt, 'g')
        # axes[2, 1].plot(angle_yz_filt, 'b')
        
        # titles = np.array([['End effector X-displacement over time',
        #           'End effector Y-displacement over time',
        #           'End effector Z-displacement over time'],
        #           ['Forearm angle from +X-axis in XY plane',
        #           'Forearm angle from +X-axis in XZ plane',
        #           'Forearm angle from +Z-axis in YZ plane']]
        #           ).transpose()
        
        # xmin = -3
        # xmax = num_frames + 3
        # disp_tick_spacing = 0.05
        # ang_tick_spacing = 30
        # for i in range(axes.shape[0]):
        #     for j in range(axes.shape[1]):
        #         axes[i, 0].yaxis.set_major_locator(
        #             ticker.MultipleLocator(disp_tick_spacing)
        #             )
        #         axes[i, 1].yaxis.set_major_locator(
        #             ticker.MultipleLocator(ang_tick_spacing)
        #             )
        #         axes[i, j].grid(True, axis='y')
        #         axes[i, j].hlines(0, xmin, xmax,
        #                           colors='black', linestyles='dashed')
        #         axes[i, j].set_xlim(xmin, xmax)
        #         axes[i, j].set_title(titles[i, j])
        #         axes[2, j].set_xlabel('Frame number')
                
        # axes[1, 0].set_ylabel('displacement (m)')
        # axes[1, 1].set_ylabel('Angle (degrees)')