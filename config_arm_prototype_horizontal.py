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

# Platform arm length limits
min_length = 226.8
max_length = 310

# Platform position limmits
h_min = -40
h_max = 40
v_min = 210 # Hard lower limit: 197.534 mm
v_max = 270

# Lower u-joint axis locations
basepoints = np.array([[0, -14, -127.017],
                       [0,  14, -127.017],
                       [0,  117, 51.384 ],
                       [0,  103, 75.633 ],
                       [0, -103, 75.633 ],
                       [0, -117, 51.384 ]]
                      ).T