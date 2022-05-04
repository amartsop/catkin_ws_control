import cv2
import numpy as np
import json

class CameraModel:

    def __init__(self, filename):

        # Read camera properties
        with open(filename) as jsonFile:
            jsonObject = json.load(jsonFile)
            jsonFile.close()

        ##################### Extrinsic Parameters ######################3
        # Camera origin position wrt to the inertial frame in meters
        camera_pos = jsonObject['camera_position(m)']
        self._rok_F_F = np.array([ [camera_pos["x"]], [camera_pos["y"]],
            [camera_pos["z"]] ])

        # Camera orientation wrt to the inertial frame
        camera_or = jsonObject['camera_orientation']

        self._rot_fk_F = np.array([
            [camera_or["r11"], camera_or["r12"], camera_or["r13"]],
            [camera_or["r21"], camera_or["r22"], camera_or["r23"]],
            [camera_or["r31"], camera_or["r32"], camera_or["r33"]] ])

        self._rot_F_fk = (self._rot_fk_F).transpose()

        # Camera to world homogeneous mapping
        self._h_fk_F = self.pose_mapping(self._rok_F_F, self._rot_fk_F)

        # World to camera homogeneous mapping
        self._h_F_fk = self.pose_mapping(-np.matmul(self._rot_F_fk, self._rok_F_F),
            self._rot_F_fk)

        ##################### Intrinsic Parameters ######################3
        # Normalized focal lenghts in x and y directions
        self._fx_bar = jsonObject['focal_length_bar_x(px)']
        self._fy_bar = jsonObject['focal_length_bar_y(px)']

        # Optic axis position wrt sensor frame
        self._xh_fs = jsonObject['central_point_x(px)']
        self._yh_fs = jsonObject['central_point_y(px)']

        # Camera matrix bar
        self._k_bar = self.camera_matrix_bar(self._fx_bar, self._fy_bar, 
            self._xh_fs, self._yh_fs)

        # Camera matrix
        self._k = self.camera_matrix()

        # Sensor to origin mapping
        self._p_F_fs = np.matmul(self._k_bar, self._h_F_fk)
    
    # Get camera matrix
    def get_camera_matrix(self):
        return self._k

    # Get camera to world rotation matrix
    def get_camera_to_world_rotation_matrix(self):
        return self._rot_fk_F

    # Get camera to world position vector
    def get_camera_to_world_vector(self):
        return self._rok_F_F

    # Return projection matrix
    def get_projection_matrix(self):
        return self._p_F_fs

    # Pose mapping
    def pose_mapping(self, pos, rot):

        # Initialze homogeneous transformation matrix
        h_mat = np.zeros((4, 4))

        # Assign rotation matrix
        h_mat[0:3, 0:3] = rot

        # Assign position vector
        h_mat[0:3, 3:] = pos

        # Assign unit element
        h_mat[3, 3] = 1.0

        return h_mat

    # Camera matrix bar
    def camera_matrix_bar(self, fx_bar, fy_bar, xh_fs, yh_fs):
        
        # Initialze camera matrix
        k_bar = np.zeros((3, 4))

        # Assign values
        k_bar[0, 0] = fx_bar
        k_bar[1, 1] = fy_bar
        k_bar[0, 2] = xh_fs
        k_bar[1, 2] = yh_fs
        k_bar[2, 2] = 1.0
        
        return k_bar

    # Camera matrix 
    def camera_matrix(self):
        
        # Initialze camera matrix
        k_mat = np.zeros((3, 3))

        # Assign values
        k_mat[0, 0] = self._k_bar[0, 0]
        k_mat[1, 1] = self._k_bar[1, 1]
        k_mat[0, 2] = self._k_bar[0, 2]
        k_mat[1, 2] = self._k_bar[1, 2]
        k_mat[2, 2] = 1.0
        
        return k_mat

    # Object to sensor mapping
    def object_to_sensor(self, rop_F_F):

        # Transform vector to homogeneous equivalent
        rop_F_F_tilde = np.concatenate((rop_F_F, np.array([[1.0]])))

        # Perform mapping
        rsp_fs_fs_tilde = np.matmul(self._p_F_fs, rop_F_F_tilde)

        # Homogeneous to normal (rounded to the nearest integer)
        rsp_fs_fs = cv2.convertPointsFromHomogeneous(rsp_fs_fs_tilde.transpose())
        rsp_fs_fs_comp = rsp_fs_fs[0][0]
        rsp_fs_fs = np.array([[rsp_fs_fs_comp[0]], [rsp_fs_fs_comp[1]]])
        rsp_fs_fs = np.rint(rsp_fs_fs)

        return rsp_fs_fs