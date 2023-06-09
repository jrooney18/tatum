import matplotlib.pyplot as plt
import numpy as np
import serial
import time

import config_arm_prototype as config


def read_serial(port):
    ''' Reads line at open port, converts to a string, and removes limiters '''
    message = port.readline()
    message = str(message, 'utf-8')
    message = message.rstrip()
    return message

def get_joint_positions(plat_center, plat_radius, joint_angles):
    ''' Finds the position relative to center of each platform joint.
    Arguments:
        plat_center (array): 3x1 array with platform xyz coordinates.
        plat_radius (float): distance from platform center to joints.
        joint_angles(array): angle from center to each joint, in degrees.
    Returns:
        plat_joints (array): 3x6 array with joint xyz coordinates in
                             indices [0, :], [1, :], and [2, :]
    '''
    # Get the platform joint locations relative to the center position
    joint_angles = np.deg2rad(joint_angles)
    plat_joints = np.zeros([3, 6])
    plat_joints[0] = plat_radius * np.cos(joint_angles) + plat_center[0]
    plat_joints[1] = plat_radius * np.sin(joint_angles) + plat_center[1]
    plat_joints[2] = plat_center[2]
    return plat_joints

def quaternion(axis, angle):
    ''' Creates a quaternion from the axis-angle rotation representation '''
    quat = np.array([np.cos(angle/2),
                     np.sin(angle/2) * axis[0],
                     np.sin(angle/2) * axis[1],
                     np.sin(angle/2) * axis[2]
                     ])
    return quat

def quat_mult(q, p):
    ''' Multiplies 2 quaternions, q and p, in format (a, bi, cj, dk) '''
    q0, q1, q2, q3 = q
    p0, p1, p2, p3 = p
    qp = np.array([
        q0*p0 - q1*p1 - q2*p2 - q3*p3,
        q0*p1 + q1*p0 - q2*p3 + q3*p2,
        q0*p2 + q1*p3 + q2*p0 - q3*p1,
        q0*p3 - q1*p2 + q2*p1 + q3*p0
        ])
    return qp

def rotate_axis_angle(points, origin, axis, angle):
    ''' Rotates points about the origin rotating by angle about axis '''
    # Create quaternions to perform the rotation
    quat = quaternion(axis, angle)
    quat_inv = np.copy(quat)
    quat_inv[1:] = -quat_inv[1:]
    
    # Convert the platform coordinates to quaternion notation
    _origin = np.insert(origin, 0, 0, axis=0)
    _points = np.insert(points, 0, 0, axis=0)
    
    # Rotate the platform joint points
    points_rot = quat_mult(quat, _points - _origin)
    points_rot = quat_mult(points_rot, quat_inv) + _origin
    
    # Convert coordinates back from quaternion notation
    points_rot = np.delete(points_rot, 0, axis=0)
    return points_rot
    
def orient_platform(arm_vector, plat_center, plat_joints):
    ''' Find coordinates for the platform joints with the platform aligned
        along an input unit vector.
    Arguments: 
        arm_vector (array): unit vector from (0, 0, 0).
        plat_center (array): 3x1 array of coordinates for platform.
        plat_joints (array): 3x6 array of coordinates for platform joints.
    Returns:
        plat_joints_rot (array): 3x6 array of coordinates for platform
                                 joints oriented to align with arm_vector.
    '''
    # Define the platform's Z-axis.
    Z = np.array([0, 0, 1])
    
    # Find the rotation axis to align the Z-axis with the random vector
    axis = np.cross(arm_vector, Z)
    axis = axis / np.linalg.norm(axis)
    
    # Find the required rotation angle about that axis
    theta = np.arccos(np.dot(arm_vector, Z))
    
    plat_joints_rot = rotate_axis_angle(plat_joints, plat_center, axis, theta)
    
    return plat_joints_rot

def plot_platform(ax,
                  arm_vector,
                  plat_center,
                  basepoints,
                  plat_joints_rot,
                  color):
    artists = []
    # Plot unit vector for alignment
    artists += ax.plot([0, 50*arm_vector[0]] + plat_center[0],
                       [0, 50*arm_vector[1]] + plat_center[1],
                       [0, 50*arm_vector[2]] + plat_center[2],
                       color='green')
    
    # # Plot global Z-axis at platform center
    # Z = np.array([0, 0, 1])
    # artists += ax.plot([0, 50*Z[0]] + plat_center[0],
    #                    [0, 50*Z[1]] + plat_center[1],
    #                    [0, 50*Z[2]] + plat_center[2],
    #                    color='red')
    
    # Plot rotated platform legs
    for _i in range(6):
        artists += ax.plot([basepoints[0, _i], plat_joints_rot[0, (_i-1)%6]],
                            [basepoints[1, _i], plat_joints_rot[1, (_i-1)%6]],
                            [basepoints[2, _i], plat_joints_rot[2, (_i-1)%6]],
                            color=color)
    
    # Plot rotated platform outline
    for _i in range(6):
        artists += ax.plot([plat_joints_rot[0, _i], plat_joints_rot[0, (_i+1)%6]],
                           [plat_joints_rot[1, _i], plat_joints_rot[1, (_i+1)%6]],
                           [plat_joints_rot[2, _i], plat_joints_rot[2, (_i+1)%6]],
                           color=color)
    
    # Plot base points
    artists += ax.plot(basepoints[0],
                       basepoints[1],
                       basepoints[2],
                       'o', color='firebrick')
    
    # Plot platform center point
    artists += ax.plot(plat_center[0],
                       plat_center[1],
                       plat_center[2],
                       'o', color='darkorange')
    
    # Plot rotated platform joint points
    artists += ax.plot(plat_joints_rot[0],
                       plat_joints_rot[1],
                       plat_joints_rot[2],
                       'o', color=color)
    return artists


if __name__ == '__main__':
# Open serial communication with OpenRB
    with serial.Serial(port='COM7', baudrate=115200, timeout=0.5) as openrb:
        ax = plt.figure(figsize=(12,10)).add_subplot(projection='3d')
        
        # while True:
        #     message = read_serial(openrb)
        #     if message:
        #         if message.endswith('...'):
        #             print(message, end='')
        #         else:
        #             print(message)
        #         if message == 'Ready.':
        #             break
        #     time.sleep(.05)
        
        while True:
            # Generate random coordinates for the platform center
            plat_center = np.zeros([3, 1])
            plat_center[0] = np.random.uniform(low=config.h_min,
                                               high=config.h_max)
            plat_center[1] = np.random.uniform(low=config.h_min,
                                               high=config.h_max)
            plat_center[2] = np.random.uniform(low=config.v_min,
                                               high=config.v_max)
            
            # Generate a random unit vector to align to
            arm_vector = np.random.rand(3)
            arm_vector[0:2] = (arm_vector[0:2] * 2) - 1
            arm_vector = arm_vector / np.linalg.norm(arm_vector)
            
            # Find the un-oriented location of the platform joints
            plat_joints = get_joint_positions(plat_center,
                                              config.plat_radius,
                                              config.joint_angles)
            
            # Orient the platform along the given unit vector
            plat_joints_rot = orient_platform(arm_vector,
                                              plat_center,
                                              plat_joints)
            
            
            # Calculate leg lengths, and initialize final spot for checksum
            lengths = np.zeros(config.num_legs)
            for i in range(config.num_legs):
                lengths[i] = np.sqrt(
                    (plat_joints_rot[0, (i-3)%6] - config.basepoints[0, i])**2+
                    (plat_joints_rot[1, (i-3)%6] - config.basepoints[1, i])**2+
                    (plat_joints_rot[2, (i-3)%6] - config.basepoints[2, i])**2
                    )
            lengths = lengths.round(1)
            
            # Wait for user input to proceed
            command = input()
            if command == 'x':
                break
            
            # Plot platform position
            ax.cla()
            plot_platform(ax,
                          arm_vector,
                          plat_center,
                          config.basepoints,
                          plat_joints_rot,
                          color='slateblue')
            ax.set_zlim([0, 230])
            ax.set_aspect('equal')
            plt.pause(0.05)
            
            # Transmit data only if all lengths are achievable
            if ((lengths > config.min_length).all()
            and (lengths < config.max_length).all()):
                
                # Add checksum to leg lengths
                lengths = np.append(lengths, np.sum(lengths))
                
                for length in lengths:
                    openrb.write(str(length).encode())
                    openrb.write('\n'.encode())
                for i in range(lengths.size + 1):
                    received_data = openrb.readline()
                    print(f'Message {i}: ',
                          received_data.decode('utf-8').strip())
            else:
                print('Out of bounds')
            
    print('Connection terminated.')