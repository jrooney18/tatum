import numpy as np

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