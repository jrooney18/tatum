import numpy as np
''' Config file for physical arm parameters. '''

# Number of platform legs
num_legs = 6

# Radius of platform and angle to each upper arm joint
plat_radius = 27.927
joint_angles = np.array([17.281,
                         42.720,
                         137.281,
                         162.72,
                         257.281,
                         282.720])

# Calculate joint positions using radius and angles
joint_angles = np.deg2rad(joint_angles)
joints = np.zeros([6, 3])
joints[:, 0] = plat_radius * np.cos(joint_angles)
joints[:, 1] = plat_radius * np.sin(joint_angles)
joints[:, 2] = 0

# Platform arm length limits
min_length = 226.8
max_length = 310

# Platform position limmits
h_min = -40
h_max = 40
v_min = 210 # Hard lower limit: 197.534 mm
v_max = 270

# Length of the human forearm wrist-elbow and robot forearm wrist-end
scale_factor = 1
len_forearm = 235 * scale_factor  # mm
len_robot   = 130 * scale_factor  # mm

# Lower u-joint axis locations
basepoints = np.array([[0, -14, -127.017],
                       [0,  14, -127.017],
                       [0,  117, 51.384 ],
                       [0,  103, 75.633 ],
                       [0, -103, 75.633 ],
                       [0, -117, 51.384 ]]
                      )

angle = np.deg2rad(75)
cos = np.cos(angle)
sin = np.sin(angle)
z_rot = np.array([[cos, -sin, 0],
                  [sin,  cos, 0],
                  [0  ,  0  , 1]]
                 )

basepoints = np.matmul(z_rot, basepoints.T)
basepoints = basepoints.T

offset = np.array([190, -200, 0])
basepoints = basepoints + offset