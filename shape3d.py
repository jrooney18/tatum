import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

import quaternion


class Shape3d:
    ''' This class stores a 3d shape defined by vertices, edges, and an origin.
    
    This class contains defining information for a shape in 3D space, and 
    simplifies transforms of that shape as a single object. The class uses,
    and expects, all points or sets of points to be stored as n x 3 Numpy
    arrays, where n is the number of points and each row stores a single
    [x, y, z] coordinate. For single points (e.g. the shape's origin), 
    a 1-dimensional Numpy array of 3 elements is used. 
    
    Note that for tranformation purposes, points_local and axes_local are a 
    mix of local and global coordinates. axes_local stores the three orthogonal
    unit vectors that make up the local frame's axes, as currently rotated in
    the global frame. Similarly, points_local stores point offsets from the 
    local frame's origin, but rotated globally to match the local frame's
    rotation.
    
    Class attributes:
        points_local (n x 3 array)
            Coordinates of the shape's vertices relative to its own frame.
        points_global (n x 3 array)
            Coordinates of the shape's vertices in the global frame.
        origin (1D array)
            Coordinates of the shape's origin in the global frame.
        axes_local (3 x 3 array)
            Stack of 3 row vectors defining the shape's local axes.
        axes_global (3 x 3 array)
            Stack of 3 row vectors defining the axes in the global frame.
        edges (list)
            List of endpoint pairs, as either lists or tuples
            (e.g. [(0, 1), (1, 2)] stores 2 edges, between points 0-1 and 1-2).
        edge_colors (list)
            List of color strings for plotting shape, indices matching edges.
        axes_colors (list)
            3-element list of color strings for coloring X, Y, and Z axes.
    '''
    
    def __init__(self, points_local, edges, origin=np.zeros(3)):
        ''' Create a shape3d object.
        
        Arguments:
            points_local (n x 3 array)
                Vertex coordinates relative to the shape's local frame.
            edges (list)
                List of endpoint pairs, as either lists or tuples.
            origin (1D array)
                XYZ coordinates in the world frame. Defaults to [0, 0, 0].
        '''
        self._origin = origin
        self._points_local = points_local
        self._axes_local = np.eye(3)
        self._quat = quaternion.Quaternion(np.zeros(3), 0)
        self.locate_points()
        self._edges = edges
        
        self._edge_colors = [None] * len(edges)
        self.set_axes_colors(['red', 'green', 'blue'])
    
    def locate_points(self):
        self._points_local = quaternion.rotate_via_quat(self._points_local,
                                                        self._quat)
        self._axes_local = quaternion.rotate_via_quat(self._axes_local,
                                                      self._quat)
        self._points_global = self._points_local + self._origin
        self._axes_global = self._axes_local + self._origin
        self.reset_rotation()
    
    def reset_rotation(self):
        self._quat = quaternion.Quaternion(np.zeros(3), 0)
    
    @property
    def origin(self):
        return self._origin
    
    @origin.setter
    def origin(self, point):
        self._origin = point
        self.locate_points()
    
    @property
    def local_axes(self):
        return self._axes_local
    
    @property
    def global_points(self):
        return self._points_global
    
    @property
    def edge_lengths(self):
        self._lengths = np.zeros(len(self._edges))
        for i, edge in enumerate(self._edges):
            self._lengths[i] = np.sqrt(
                (self._points_local[edge[0]][0] -
                 self._points_local[edge[1]][0]) ** 2 +
                (self._points_local[edge[0]][1] -
                 self._points_local[edge[1]][1]) ** 2 +
                (self._points_local[edge[0]][2] -
                 self._points_local[edge[1]][2]) ** 2 )
        return self._lengths
        
    def translate(self, offset):
        self._origin += offset
        self.locate_points
    
    def align_with_vector(self, vector):
        # Normalize the input vector
        _vector = vector / np.linalg.norm(vector)
        
        # Find the quaternion that aligns the local Z-axis with the vector
        axis = np.cross(_vector, self._axes_local[2])
        if np.sum(abs(axis)) != 0:
            axis /= np.linalg.norm(axis)
        # Dot product rounded to 5 decimals to avoid floating point errors
        angle = np.arccos(np.dot(_vector, self._axes_local[2]).round(5))
        self._quat = quaternion.Quaternion(axis, angle)
        self.locate_points()
    
    def rotate_about_z(self, angle):
        ''' Rotate shape about its local z-axis by angle (radians)'''
        self._quat = quaternion.Quaternion(self._axes_local[2], angle)
        self.locate_points()
    
    def set_edge_colors(self, colors):
        if type(colors)== list:
            self._edge_colors = colors
        elif type(colors) == str:
            self._edge_colors = [colors]
            self._edge_colors.extend([colors] * (len(self._edges) - 1))
    
    def set_axes_colors(self, colors):
        self._axes_colors = colors
    
    def plot(self, ax):
        ''' Plot the shape on the input 3d axes. '''
        artists = []
        for i, edge in enumerate(self._edges):
            artists += ax.plot(
                [self._points_global[edge[0]][0], self._points_global[edge[1]][0]],
                [self._points_global[edge[0]][1], self._points_global[edge[1]][1]],
                [self._points_global[edge[0]][2], self._points_global[edge[1]][2]],
                color=self._edge_colors[i]
                )
        return artists
    
    def plot_axes(self, ax, scale=1):
        ''' Plot the shape's axes on the input 3d axes. '''
        artists = []
        _axes = self._axes_global - self._origin
        _axes *= scale
        _axes += self._origin
        for i in range(3):
            artists += ax.plot(
                [self._origin[0], _axes[i, 0]],
                [self._origin[1], _axes[i, 1]],
                [self._origin[2], _axes[i, 2]],
                color=self._axes_colors[i]
                )
        return artists
        

if __name__ == '__main__':
    # Define shape origin in world frame, vertices in local frame, and
    # endpoints of edges
    origin = np.zeros(3)
    points = np.array([[ 1,  1,  1],
                       [-1,  1,  1],
                       [-1, -1,  1],
                       [ 1, -1,  1],
                       [ 1,  1, -1],
                       [-1,  1, -1],
                       [-1, -1, -1],
                       [ 1, -1, -1]]
                      )
    edges=[[0, 1], [1, 2], [2, 3], [3, 0],
           [4, 5], [5, 6], [6, 7], [7, 4],
           [0, 4], [1, 5], [2, 6], [3, 7]]
    
    # Create shape and set its color
    cube = Shape3d(points_local=points,
                   edges=edges,
                   origin=origin
                   )
    cube.set_edge_colors('steelblue')
    
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    artists = []
    
    for i in np.arange(0, 3, 0.05):
        # Choose a new centerpoint and alignment vector for shape
        new_point = np.array([i, i, i/2])
        vector = np.array([np.sin(i*3), 0, np.cos(i*3)])
        
        # Transform cube
        cube.origin = new_point
        cube.align_with_vector(vector)
        cube.rotate_about_z(np.deg2rad(5))
        
        # Plot shape
        frame_artists = []
        frame_artists += ax.plot([0, vector[0]],
                                 [0, vector[1]],
                                 [0, vector[2]],
                                 color='tab:orange')
        frame_artists += ax.plot(0, 0, 0, 'ok')
        frame_artists += cube.plot(ax)
        frame_artists += cube.plot_axes(ax, scale=2)
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.set_zlim(-5, 5)
        ax.set_aspect('equal')
        
        artists.append(frame_artists)
    ani = animation.ArtistAnimation(fig=fig, artists=artists, interval=100)
    plt.show()
