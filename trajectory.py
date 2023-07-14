import keypoint_connections as conn
import keypoint_utils as kpt


class Trajectory:
    def __init__(self,
                 filename,
                 ref_segment,
                 ref_length,
                 robot_length,
                 forearm_length):
        self.filename = filename
        
        # Read in keypoints from file
        self.points, self._num_frames = kpt.read_kpts_3d_file(filename)

        # Orient keypoints with shoulder at origin and scale
        self.points = kpt.orient_kpts(self.points)
        self.points = kpt.scale_kpts(self.points, ref_segment, ref_length)
        
        dist_elbow2platform = forearm_length - robot_length
        self._vector = ((self.points[conn.wrist_r] - self.points[conn.elbow_r])
                        / forearm_length)
        self._location = (self._vector *
                          dist_elbow2platform +
                          self.points[conn.elbow_r])
        self._location = self._location.T
        self._vector = self._vector.T

    def offset(self, offset):
        self._location -= offset
        
    @property
    def vector(self):
        return self._vector
    
    @property
    def location(self):
        return self._location
    
    @property
    def num_frames(self):
        return self._num_frames
    

    # def filter(self, savgol_window, savgol_degree):
    #     forearm_point = savgol_filter(forearm_point, savgol_window, savgol_degree)