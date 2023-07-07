import matplotlib.pyplot as plt
import numpy as np
import serial
import time


def read_serial(port):
    message = port.readline()
    message = str(message, 'utf-8')
    message = message.rstrip()
    return message

def quat_mult(q, p):
    q0 = q[0]; q1 = q[1]; q2 = q[2]; q3 = q[3]
    p0 = p[0]; p1 = p[1]; p2 = p[2]; p3 = p[3]
    qp = np.array([
        q0*p0 - q1*p1 - q2*p2 - q3*p3,
        q0*p1 + q1*p0 - q2*p3 + q3*p2,
        q0*p2 + q1*p3 + q2*p0 - q3*p1,
        q0*p3 - q1*p2 + q2*p1 + q3*p0
        ])
    return qp

# Number of platform legs
num_legs = 6

# Platform arm length limits
min_length = 226.8
max_length = 310

# Platform position limmits
h_min = -30
h_max = 30
v_min = 210 # Hard lower limit: 197.534 mm
v_max = 280

# Lower u-joint axis locations
basepoints = np.array([[-14, -127.017, 0],
                       [ 14, -127.017, 0],
                       [ 117, 51.384,  0],
                       [ 103, 75.633,  0],
                       [-103, 75.633,  0],
                       [-117, 51.384,  0]]
                      ).T

# World frame coordinate axes
X = np.array([1, 0, 0])
Y = np.array([0, 1, 0])
Z = np.array([0, 0, 1])


ax = plt.figure(figsize=(12,10)).add_subplot(projection='3d')

# Open serial communication with OpenRB
with serial.Serial(port='COM7', baudrate=115200, timeout=0.5) as openrb:
    while True:
        message = read_serial(openrb)
        if message:
            if message.endswith('...'):
                print(message, end='')
            else:
                print(message)
            if message == 'Ready.':
                break
        time.sleep(.05)
    
    while True:
        # Generate random coordinates for the platform center
        plat_center = np.zeros([4, 1])
        plat_center[1] = np.random.uniform(low=h_min, high=h_max)
        plat_center[2] = np.random.uniform(low=h_min, high=h_max)
        plat_center[3] = np.random.uniform(low=v_min, high=v_max)

        # Get the platform joint locations relative to the center position
        joint_angles = np.array([77.281,
                                 102.720,
                                 197.281,
                                 222.72,
                                 317.281,
                                 342.720])
        joint_angles = np.deg2rad(joint_angles)
        plat_radius = 27.927
        plat_joints = np.zeros([4, 6])
        plat_joints[1] = plat_radius * np.cos(joint_angles) + plat_center[1]
        plat_joints[2] = plat_radius * np.sin(joint_angles) + plat_center[2]
        plat_joints[3] = plat_center[3]
        
        # Generate a random unit vector to align to
        arm_vector = np.random.rand(3)
        arm_vector[0:2] = (arm_vector[0:2] * 2) - 1
        arm_vector = arm_vector / np.linalg.norm(arm_vector)
        
        # Find the rotation axis to align the Z-axis with the random vector
        axis = np.cross(arm_vector, Z)
        axis = axis / np.linalg.norm(axis)
        
        # Find the required rotation angle about that axis
        theta = np.arccos(np.dot(arm_vector, Z))
        
        # Create quaternions to perform the rotation
        quat = np.array([np.cos(theta/2),
                         np.sin(theta/2) * axis[0],
                         np.sin(theta/2) * axis[1],
                         np.sin(theta/2) * axis[2]
                         ])
        quat_inv = np.copy(quat)
        quat_inv[1:] = -quat_inv[1:]
        
        # Rotate the platform joint points
        plat_joints_rot = quat_mult(quat, plat_joints - plat_center)
        plat_joints_rot = quat_mult(plat_joints_rot, quat_inv) + plat_center
        
        # Calculate leg lengths, and initialize final spot for checksum
        lengths = np.zeros(num_legs + 1)
        for _i in range(num_legs):
            lengths[_i] = np.sqrt(
                (plat_joints_rot[1, (_i-3)%6] - basepoints[0, _i]) ** 2 +
                (plat_joints_rot[2, (_i-3)%6] - basepoints[1, _i]) ** 2 +
                (plat_joints_rot[3, (_i-3)%6] - basepoints[2, _i]) ** 2
                )
        lengths = lengths.round(1)
        lengths[-1] = lengths[:-1].sum()
        
        command = input()
        if command == 'x':
            break
        
        ax.cla()
        # Plot unit vector for alignment
        ax.plot([0, 50*arm_vector[0]] + plat_center[1],
                [0, 50*arm_vector[1]] + plat_center[2],
                [0, 50*arm_vector[2]] + plat_center[3],
                color='fuchsia')
        
        # Plot global Z-axis at platform center
        ax.plot([0, 50*Z[0]] + plat_center[1],
                [0, 50*Z[1]] + plat_center[2],
                [0, 50*Z[2]] + plat_center[3],
                color='red')
        
        # Plot pre-rotation platform legs
        for _i in range(6):
            ax.plot([basepoints[0, _i], plat_joints[1, (_i-3)%6]],
                    [basepoints[1, _i], plat_joints[2, (_i-3)%6]],
                    [basepoints[2, _i], plat_joints[3, (_i-3)%6]],
                    color='paleturquoise')
            
        # Plot rotated platform legs
        for _i in range(6):
            ax.plot([basepoints[0, _i], plat_joints_rot[1, (_i-3)%6]],
                    [basepoints[1, _i], plat_joints_rot[2, (_i-3)%6]],
                    [basepoints[2, _i], plat_joints_rot[3, (_i-3)%6]],
                    color='slateblue')
        
        # Plot pre-rotation platform outline
        for _i in range(6):
            ax.plot([plat_joints[1, _i], plat_joints[1, (_i+1)%6]],
                    [plat_joints[2, _i], plat_joints[2, (_i+1)%6]],
                    [plat_joints[3, _i], plat_joints[3, (_i+1)%6]],
                    color='paleturquoise')
        
        # Plot rotated platform outline
        for _i in range(6):
            ax.plot([plat_joints_rot[1, _i], plat_joints_rot[1, (_i+1)%6]],
                    [plat_joints_rot[2, _i], plat_joints_rot[2, (_i+1)%6]],
                    [plat_joints_rot[3, _i], plat_joints_rot[3, (_i+1)%6]],
                    color='slateblue')
        
        # Plot base points
        ax.plot(basepoints[0],
                basepoints[1],
                basepoints[2],
                'o', color='firebrick')
        
        # Plot platform center point
        ax.plot(plat_center[1],
                plat_center[2],
                plat_center[3],
                'o', color='darkorange')
        
        # Plot pre-rotation platform joint points
        ax.plot(plat_joints[1],
                plat_joints[2],
                plat_joints[3],
                'o', color='paleturquoise')
        
        # Plot rotated platform joint points
        ax.plot(plat_joints_rot[1],
                plat_joints_rot[2],
                plat_joints_rot[3],
                'o', color='slateblue')
        
        
        ax.set_zlim([0, 230])
        ax.set_aspect('equal')
        plt.pause(0.05)
        
        # Transmit data only if all lengths are achievable
        if ((lengths[:-1] > min_length).all()
            and (lengths[:-1] < max_length).all()):
            
            for length in lengths:
                openrb.write(str(length).encode())
                openrb.write('\n'.encode())
            for i in range(lengths.size + 1):
                received_data = openrb.readline()
                print(f'Message {i}: ', received_data.decode('utf-8').strip())
        else:
            print('Out of bounds')
        
        time.sleep(0.1)
        
print('Connection terminated.')