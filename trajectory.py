# import keypoint_connections as conn
# import keypoint_utils as kpt
import numpy as np


class Trajectory:
    def __init__(self,
                 filename,
                 robot_length,
                 forearm_length):
        self.filename = filename
        
        # Read in keypoints from file
        self._points = np.load(filename)
        self._num_frames = self._points.shape[0]
        # self._points, self._num_frames = kpt.read_kpts_3d_file(filename)

        # Orient keypoints with shoulder at origin and scale
        self._points *= 1000
        x_angle = np.deg2rad(-90)
        z_angle = np.deg2rad(-90)
        x_rot = np.array([[1, 0,                0              ],
                          [0, np.cos(x_angle), -np.sin(x_angle)],
                          [0, np.sin(x_angle),  np.cos(x_angle)]])
        z_rot = np.array([[np.cos(z_angle), -np.sin(z_angle), 0],
                          [np.sin(z_angle),  np.cos(z_angle), 0],
                          [0,                0,               1]])
        self._points = np.matmul(self._points, x_rot)
        self._points = np.matmul(self._points, z_rot)
        self._points = self._points - self._points[:, [1], :]
        # self._points = kpt.orient_kpts(self._points)
        # self._points = kpt.scale_kpts(self._points, ref_segment, ref_length)
        
        # dist_elbow2platform = forearm_length - robot_length
        elbow = 3
        wrist = 4
        forearm_lengths = np.sqrt(
            (self._points[:, [wrist], 0] - self._points[:, [elbow], 0])**2 +
            (self._points[:, [wrist], 1] - self._points[:, [elbow], 1])**2 +
            (self._points[:, [wrist], 2] - self._points[:, [elbow], 2])**2)
        dist_elbow2platform = (forearm_lengths *
                               (1 - robot_length/forearm_length))
        self._vector = ((self._points[:, wrist] - self._points[:, elbow])
                        / forearm_length)
        self._location = (self._vector *
                          dist_elbow2platform +
                          self._points[:, elbow])
        # self._location = self._location.T
        # self._vector = self._vector.T

    def offset(self, offset):
        self._location -= offset
        
    @property
    def points(self):
        return self._points
        
    @property
    def vector(self):
        return self._vector
    
    @property
    def location(self):
        return self._location
    
    @property
    def num_frames(self):
        return self._num_frames