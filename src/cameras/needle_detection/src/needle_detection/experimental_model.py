import numpy as np
import cv2
from needle_detection.transformations import Transformations

class ExperimentModel:

    def __init__(self):

        # Define robot to inertial frame transformation
        r_or_F_F = np.array([[-0.332649], [0.3], [-0.022]])
        rot_fr_F = np.eye(3)
        self.t_fr_F = Transformations.transformation_matrix(r_or_F_F, rot_fr_F)
        
        # Define rotation matrix of point p wrt the end-effector
        self._rot_f_fe = np.eye(3);

        # Define the position of the "e" point wrt the end-effector
        self.r_ee_fe_fe = np.array([[0.0], [0.0], [0.0]])

        # Define the position of the base point wrt the end-effector
        self.r_ea_fe_fe = np.array([[0.2234], [0.0], [-0.04515]])

    # Get inertial position
    def get_inertial_position(self, rep_fe_fe, ee_state_vector):

        # Calculate end-effectors transformation matrix wrt to robot's base
        r_re_fr_fr = ee_state_vector[0:3]
        rot_fe_fr = Transformations.rotation(ee_state_vector[3:6])
        t_fe_fr = Transformations.transformation_matrix(r_re_fr_fr, rot_fe_fr)
        
        # Point tilde 
        rep_fe_fe_tilde = np.concatenate((rep_fe_fe, np.array([[1.0]])))

        # Get inertial position of interest point  
        rop_F_F_tilde = np.matmul(self.t_fr_F, np.matmul(t_fe_fr, rep_fe_fe_tilde))
        
        rop_F_F = cv2.convertPointsFromHomogeneous(rop_F_F_tilde.transpose())

        return np.array([[rop_F_F[0][0][0]], [rop_F_F[0][0][1]],
            [rop_F_F[0][0][2]]])
    

    # Get base rotation matrix
    def get_base_rotation_matrix(self, ee_state_vector):

        # Calculate end-effectors transformation matrix wrt to robot's base
        rot_fe_fr = Transformations.rotation(ee_state_vector[3:6])

        return np.matmul(rot_fe_fr, self._rot_f_fe)
