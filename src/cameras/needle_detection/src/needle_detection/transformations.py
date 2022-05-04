import numpy as np

class Transformations:

    @staticmethod
    def basic_rotation_x(x):
        sx = np.sin(x); cx = np.cos(x)
        return np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
         
    @staticmethod
    def basic_rotation_y(x):
        sx = np.sin(x); cx = np.cos(x)
        return np.array([[cx, 0.0, sx], [0.0, 1.0, 0.0], [-sx, 0.0, cx]])
    
    @staticmethod
    def basic_rotation_z(x):
        sx = np.sin(x); cx = np.cos(x)
        return np.array([[cx, -sx, 0.0], [sx, cx, 0.0], [0.0, 0.0, 1.0]])

    @staticmethod
    def rotation(eul):
        rot_z = Transformations.basic_rotation_z(eul[0][0])
        rot_y = Transformations.basic_rotation_y(eul[1][0])
        rot_x = Transformations.basic_rotation_x(eul[2][0])
        
        return np.matmul(np.matmul(rot_z, rot_y), rot_x)

    @staticmethod
    def transformation_matrix(pos, rot_mat):
        
        t_mat = np.zeros((4, 4))

        # Assign rotation matrix
        t_mat[0:3, 0:3] = rot_mat

        # Assign position vector
        t_mat[0:3, 3:] = pos

        # Assign unit element
        t_mat[3, 3] = 1.0

        return t_mat