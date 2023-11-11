''' 

Serial Arm Controller v1
1 November 2023

Generates 2 random vectors, finds the requisite 4 joint angles needed to 
match those vectors with the robot, and sends the pose to the robot

'''
import matplotlib.pyplot as plt
import numpy as np
import serial
import time


def read_serial(port):
    ''' Reads line at open port, converts to a string, and removes limiters '''
    message = port.readline()
    message = str(message, 'utf-8')
    message = message.rstrip()
    return message

def rotation_matrix(axis, angle):
    ''' Create rotation matrix for angle (rad) about axis (str x, y , z) '''
    c = np.cos(angle)
    s = np.sin(angle)
    
    match axis:
        case 'x':
            rot = np.array([[1, 0,  0],
                            [0, c, -s],
                            [0, s,  c]])
        case 'y':
            rot = np.array([[ c, 0, s],
                            [ 0, 1, 0],
                            [-s, 0, c]])
        case 'z':
            rot = np.array([[c, -s, 0],
                            [s,  c, 0],
                            [0,  0, 1]])
    return rot

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
    
    with serial.Serial(port='COM7', baudrate=115200, timeout=0.5) as openrb:   
        while True:
            # Allow arm to run through homing sequence
            message = read_serial(openrb)
            if message:
                if message.endswith('...'):
                    print(message, end='')
                else:
                    print(message)
                if message == 'Ready.':
                    break
            time.sleep(.05)
        
        fig = plt.figure(figsize=[16, 16])
        axes = [fig.add_subplot(221, projection='3d'),
                fig.add_subplot(222, projection='3d'),
                fig.add_subplot(223, projection='3d'),
                fig.add_subplot(224, projection='3d'),]
        
        while True:
            # Wait for user input to proceed
            command = input()
            if command == 'x':
                break
            
            # Generate a unit vector with -1 < x < 1, y > 0, z < 0
            points = np.zeros([3, 3])
            points[0, 1] = (np.random.rand() - 0.5) * 2
            points[1, 1] = np.random.rand()
            points[2, 1] = np.random.rand() * -1
            
            # Generate a second unit vector with its endpoint on the first
            points[0, 2] = np.random.rand()
            points[1, 2] = np.random.rand()
            points[2, 2] = np.random.rand()
            
            # Normalize both vectors
            points[:, 1:] /= np.linalg.norm(points[:, 1:], axis=0)
            points[:, 2] += points[:, 1]
            
            # Plot the vectors
            for ax in axes:
                ax.cla()
                wcs = np.eye(3) * 0.3
                plot_axes(ax, wcs)
            
                ax.plot([points[0, 0], points[0, 1]],
                        [points[0, 0], points[1, 1]],
                        [points[0, 0], points[2, 1]],
                        'ko-')
                ax.plot([points[0, 1], points[0, 2]],
                        [points[1, 1], points[1, 2]],
                        [points[2, 1], points[2, 2]],
                        'ko-')
                
                ax.set_xlim(-1, 1)
                ax.set_ylim(-1, 1)
                ax.set_zlim(-1, 1)
                ax.set_aspect('equal')
            
            # Pre-initialize angles and rotation matrix
            angles = np.zeros([4])
            rot_0 = np.eye(3)
            rot = rot_0
            
            # Find joint angles using Denavit-Hartenberg theta values
            angles[0], rot = dh_rot(points[:, 0:2], rot, x_angle=-90)
            cs1 = np.matmul(rot, wcs)
            plot_axes(ax, cs1)
            angles[1], rot = dh_rot(points[:, 0:2], rot, x_angle=-90, z_offset=90)
            cs2 = np.matmul(rot, wcs)
            plot_axes(ax, cs2)
            angles[2], rot = dh_rot(points[:, 1:3], rot, x_angle=-90)
            cs3 = np.matmul(rot, wcs) + points[:, [1]] * 0.5
            plot_axes(ax, cs3, origin=points[:, 1] * 0.5)
            angles[3], rot = dh_rot(points[:, 1:3], rot, x_angle=90)
            cs4 = np.matmul(rot, wcs) + points[:, [1]]
            plot_axes(ax, cs4, origin=points[:, 1])
            
            # Apply offsets to joint angles
            deg_angles = np.rad2deg(angles)
            deg_angles[0] = -deg_angles[0] + 90
            deg_angles[2] = -deg_angles[2] + 180
            deg_angles[3] += 135
            
            
            # Report the angles
            for i, angle in enumerate(angles):
                print(f'Angle {i}: {deg_angles[i]}')
            
            # Convert the two angles to motor ticks
            ticks_per_deg = 4096 / (360)
            ticks = deg_angles * ticks_per_deg
            for i, tick in enumerate(ticks):
                print(f'Motor {i}: {ticks[i]}')

            
            # Create a pose array to transmit
            pose = np.zeros(num_motors)
            pose[0:4] = ticks.round()
            pose[4:] = home_pose[4:]
                
            # Add checksum to leg lengths
            pose = np.append(pose, np.sum(pose))
            
            for value in pose:
                openrb.write(str(value).encode())
                openrb.write('\n'.encode())
            for i in range(pose.size + 1):
                received_data = openrb.readline()
                
                print(f'Message {i}: ',
                      received_data.decode('utf-8').strip())
            
            # Set axes views
            axes[0].set_title('Front view')
            axes[0].set_proj_type('ortho')
            axes[0].view_init(0, 90)
            
            axes[1].set_title('Side view')
            axes[1].set_proj_type('ortho')
            axes[1].view_init(0, 0)
            
            axes[2].set_title('Top view')
            axes[2].set_proj_type('ortho')
            axes[2].view_init(90, -90, -90)
            
            axes[3].view_init(azim=50)
            plt.pause(0.1)
            
    print('Connection terminated.')