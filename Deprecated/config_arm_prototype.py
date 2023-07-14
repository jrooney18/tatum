import numpy as np
''' Config file for physical arm parameters. '''

# Number of platform legs
num_legs = 6

# Radius of platform and angle to each upper arm joint
plat_radius = 27.927
joint_angles = np.array([77.281,
                         102.720,
                         197.281,
                         222.72,
                         317.281,
                         342.720])

# Platform arm length limits
min_length = 226.8
max_length = 310

# Platform position limmits
h_min = -40
h_max = 40
v_min = 210 # Hard lower limit: 197.534 mm
v_max = 270

# Lower u-joint axis locations
basepoints = np.array([[-14, -127.017, 0],
                       [ 14, -127.017, 0],
                       [ 117, 51.384,  0],
                       [ 103, 75.633,  0],
                       [-103, 75.633,  0],
                       [-117, 51.384,  0]]
                      ).T