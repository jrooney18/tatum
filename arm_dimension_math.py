# -*- coding: utf-8 -*-
"""
Created on Wed May 17 10:27:31 2023

@author: Jonathan
"""

import matplotlib.pyplot as plt
import numpy as np

# l1 = np.arange(100, 200, step=10)
# l2 = 100
# h = np.arange(200, 400, step=10)
l1 = 150
l2 = 60
h = 150

# numrows = l1.size
# numcols = h.size
# d1 = np.zeros((numrows, numcols))
# d2 = np.zeros((numrows, numcols))
# d3 = np.zeros((numrows, numcols))
# d4 = np.zeros((numrows, numcols))
# theta1 = np.zeros((numrows, numcols))
# theta2 = np.zeros((numrows, numcols))
# theta3 = np.zeros((numrows, numcols))
# theta4 = np.zeros((numrows, numcols))

# for i in range(numrows):
#     d1[i] = np.sqrt((l1[i]/2 - l2/2)**2 + h**2)
#     d2[i] = np.sqrt((l1[i]/2)**2 + (h - l2/2)**2)
#     d3[i] = d1[i]
#     d4[i] = np.sqrt((l1[i]/2)**2 + (h + l2/2)**2)
#     theta1[i] = np.arcsin(h/d1[i])
#     theta2[i] = np.arcsin(l1[i]/(2*d2[i]))
#     theta3[i] = theta1[i]
#     theta4[i] = np.arcsin(l1[i]/(2*d4[i]))

d1 = np.sqrt((l1/2 - l2/2)**2 + h**2)
d2 = np.sqrt((l1/2)**2 + (h - l2/2)**2)
d3 = d1
d4 = np.sqrt((l1/2)**2 + (h + l2/2)**2)
theta1 = np.arcsin(h/d1)
theta2 = np.arcsin(l1/(2*d2))
theta3 = theta1
theta4 = np.arcsin(l1/(2*d4))

theta1 = np.rad2deg(theta1)
theta2 = -np.rad2deg(theta2)
theta3 = np.rad2deg(theta3)
theta4 = 180 - np.rad2deg(theta4)

