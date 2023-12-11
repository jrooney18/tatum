''' 

Serial Arm Controller v2
11 November 2023

Generates 2 random vectors, finds the requisite 4 joint angles needed to 
match those vectors with the robot, and sends the pose to the robot

'''
import matplotlib.pyplot as plt
import numpy as np
import serial
import time

import keypoint_utils_serial as kpt
import keypoint_connections as conn
from Rotation_Matrix import rotation_matrix


def read_serial(port):
    ''' Reads line at open port, converts to a string, and removes limiters '''
    message = port.readline()
    message = str(message, 'utf-8')
    message = message.rstrip()
    return message

def dh_rot(points, rot, x_angle, z_offset=0):
    ''' Find the Denavit-Hartenberg angle about Z for a given link '''
    # Define the rotation about the x-axis
    rot_x = rotation_matrix('x', np.deg2rad(x_angle))
    # Rotate the segment by rot and the new x-axis rotation
    points = np.matmul(rot_x, np.matmul(np.linalg.inv(rot), points))
    # Shift both points to place point 0 at the origin
    points -= points[:, [0]]
    # Find the angle about the z-axis to point 1
    angle = np.arctan2(points[1, 1], points[0, 1]) + np.deg2rad(z_offset)
    # Define the rotation matrices about the z-axis and overall
    rot_z = rotation_matrix('z', -angle)
    rot_new = np.matmul(rot_x, rot_z)
    rot = np.matmul(rot, rot_new)
    return angle, rot

def plot_axes(ax, wcs, origin = np.zeros(3)):
    ax.plot([origin[0], wcs[0, 0]],
            [origin[1], wcs[1, 0]],
            [origin[2], wcs[2, 0]],
            'r')
    ax.plot([origin[0], wcs[0, 1]],
            [origin[1], wcs[1, 1]],
            [origin[2], wcs[2, 1]],
            'g')
    ax.plot([origin[0], wcs[0, 2]],
            [origin[1], wcs[1, 2]],
            [origin[2], wcs[2, 2]],
            'b')
    

if __name__ == '__main__':
    num_motors = 7
    
    # home_pose = np.array([2050, 1200, 1850, 1700, 3000, 2000, 2000])
    home_pose = np.array([2050, 1200, 2000, 400, 2500, 2000, 2000])
    # pose_1 = np.array([1200, 1600, 2500, 1800, 3700, 1600, 2000])
    # pose_2 = np.array([1200, 1600, 2500, 1250, 3500, 2200, 2000])
    
    # with serial.Serial(port='COM7', baudrate=115200, timeout=0.5) as openrb:   
    # while True:
    #     # Allow arm to run through homing sequence
    #     message = read_serial(openrb)
    #     if message:
    #         if message.endswith('...'):
    #             print(message, end='')
    #         else:
    #             print(message)
    #         if message == 'Ready.':
    #             break
    #     time.sleep(.05)
    
    fig = plt.figure(figsize=[12, 6])
    axes = [fig.add_subplot(121, projection='3d'),
            fig.add_subplot(122, projection='3d')]
        
    # Read in MediaPipe keypoints
    path = r'Extreme ROM data\autumn\kpts-autumn0.dat'
    all_points, num_frames = kpt.read_kpts(path)
    
    for frame in range(num_frames):
        # Extract and orient one frame's keypoints
        frame_points = kpt.orient_kpts(all_points[frame])
        
        # Find points for the upper and lower arm unit vectors
        arm_right = np.zeros([3, 3])
        arm_right[:, 1] = kpt.unit_vector(frame_points[:, conn.elbow_r])
        arm_right[:, 2] = kpt.unit_vector(frame_points[:, conn.wrist_r] - 
                                          frame_points[:, conn.elbow_r])
        arm_right[:, 2] += arm_right[:, 1]
        
        # Plot the arm vectors
        wcs = np.eye(3) * 0.3
        for ax in axes:
            ax.cla()
            plot_axes(ax, wcs)
            
        kpt.plot_kpts(axes[0], frame_points, conn.arm_r, 'b')
        axes[1].plot([arm_right[0, 0], arm_right[0, 1]],
                      [arm_right[1, 0], arm_right[1, 1]],
                      [arm_right[2, 0], arm_right[2, 1]],
                      'ko-')
        axes[1].plot([arm_right[0, 1], arm_right[0, 2]],
                    [arm_right[1, 1], arm_right[1, 2]],
                    [arm_right[2, 1], arm_right[2, 2]],
                    'ko-')

        # Pre-initialize angles and rotation matrix
        angles = np.zeros([4])
        rot_0 = np.eye(3)
        rot = rot_0
        
        # Find joint angles using Denavit-Hartenberg theta values
        angles[0], rot = dh_rot(arm_right[:, 0:2], rot, x_angle=-90)
        cs1 = np.matmul(rot, wcs)
        # plot_axes(ax, cs1)
        angles[1], rot = dh_rot(arm_right[:, 0:2], rot, x_angle=-90, z_offset=90)
        cs2 = np.matmul(rot, wcs)
        # plot_axes(ax, cs2)
        angles[2], rot = dh_rot(arm_right[:, 1:3], rot, x_angle=-90)
        cs3 = np.matmul(rot, wcs) + arm_right[:, [1]] * 0.5
        # plot_axes(ax, cs3, origin=arm_right[:, 1] * 0.5)
        angles[3], rot = dh_rot(arm_right[:, 1:3], rot, x_angle=90)
        cs4 = np.matmul(rot, wcs) + arm_right[:, [1]]
        # plot_axes(ax, cs4, origin=arm_right[:, 1])
        
        # Apply offsets to joint angles
        deg_angles = np.rad2deg(angles)
        deg_angles[0] = -deg_angles[0] + 90
        deg_angles[2] = -deg_angles[2] + 180
        deg_angles[3] += 135
        
        # Report the angles
        # for i, angle in enumerate(angles):
        #     print(f'Angle {i}: {deg_angles[i]}')
        
        # Convert the two angles to motor ticks
        ticks_per_deg = 4096 / (360)
        ticks = deg_angles * ticks_per_deg
        # for i, tick in enumerate(ticks):
        #     print(f'Motor {i}: {ticks[i]}')
    
        
        # Create a pose array to transmit
        pose = np.zeros(num_motors)
        pose[0:4] = ticks.round()
        pose[4:] = home_pose[4:]
            
        # Add checksum to leg lengths
        pose = np.append(pose, np.sum(pose))
        
        # # t1 = time.time()
        # for value in pose:
        #     openrb.write(str(value).encode())
        #     openrb.write('\n'.encode())
        # # t2 = time.time()
        # # print(f'Encode time: {t2 - t1}')
        # for i in range(pose.size + 1):
        #     received_data = openrb.readline()
            
            # print(f'Message {i}: ',
            #       received_data.decode('utf-8').strip())
        
        # Set axes views
        axes[0].set_title('Raw data')
        axes[0].set_xlim3d(-40, 15)
        axes[0].set_ylim3d(-15, 35)
        axes[0].set_zlim3d(-40, 20)
        axes[0].set_aspect('equal')
        axes[0].view_init(azim=50)
        
        axes[1].set_title('Unit vectors')
        axes[1].set_xlim3d(-1, 1)
        axes[1].set_ylim3d(-1, 1)
        axes[1].set_zlim3d(-1, 1)
        axes[1].set_aspect('equal')
        axes[1].view_init(azim=50)

        # time.sleep(0.0666)
        print(f'Frame: {frame}')
        plt.pause(0.0666)
                
    print('Connection terminated.')