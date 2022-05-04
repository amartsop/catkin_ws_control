#!/usr/bin/env python

# import sys
# from matplotlib.pyplot import pause
# from scipy import signal


import rospkg
import os

import cv2
import numpy as np


from needle_detection.camera_model import CameraModel
from needle_detection.experimental_model import ExperimentModel
from needle_detection.roi_model import RoiModel

# from camera_model import CameraModel
# from experimental_model import ExperimentModel
# from roi_model import RoiModel
# from single_camera_pose_estimation import SingleCameraPoseEstimation
# from needle_model_fitting import NeedleModelFitting

class DataProcessing:

    def __init__(self):

        # Get package name
        ros_package_path = rospkg.RosPack().get_path("needle_detection")

        # Define camera1 relative configuration file
        camera1_rel_config_file = "config/camera1_config.json"

        # Define camera2 relative configuration file
        camera2_rel_config_file = "config/camera2_config.json"
        
        # Define camera1 absolute configuration file
        camera1_abs_config_file = os.path.join(ros_package_path,
            camera1_rel_config_file)

        # Define camera1 absolute configuration file
        camera2_abs_config_file = os.path.join(ros_package_path,
            camera2_rel_config_file)

        # Create camera1 model
        self._camera1_model = CameraModel(camera1_abs_config_file)

        # Create camera2 model
        self._camera2_model = CameraModel(camera2_abs_config_file)

        # Define experimental model
        self._exp_model = ExperimentModel()

        # Interest point
        r_ea_fe_fe = self._exp_model.r_ea_fe_fe

        # ROI model camera 1
        self._roi_model_camera1 = RoiModel(0.3, 0.3, self._camera1_model, 1)

        # ROI model camera 2
        self._roi_model_camera2 = RoiModel(0.3, 0.3, self._camera2_model, 2)

        # List of seperated images for camera 1
        self._seperated_images_cam1 = []

        # List of seperated images for camera 2
        self._seperated_images_cam2 = []

        # Number of corresponding points 
        self._corresponding_points_num = 20        

    def process_data(self, frame_camera1, frame_camera2, ee_state):

        # Inertial position of point A
        roa_F_F = self._exp_model.get_inertial_position(
            self._exp_model.r_ea_fe_fe, ee_state)
        
        # Rotation matrix of base frame with respect to the inertial frame
        rot_f_F = self._exp_model.get_base_rotation_matrix(ee_state)

        # Get sensor position of point a for camera 1
        rs1p1_fs1_fs1_cam1 = self._camera1_model.object_to_sensor(roa_F_F)

        # Get sensor position of point a for camera 2
        rs1p1_fs1_fs1_cam2 = self._camera2_model.object_to_sensor(roa_F_F)
        
        self.plot_frames(frame_camera1, rs1p1_fs1_fs1_cam1, frame_camera2, 
            rs1p1_fs1_fs1_cam2)

        # ###################### Camera 1 ######################

        # # Get indices of pixels of interest for camera 1 (poi)
        # rsp_fs_fs_vec_cam1 = self.get_poi_indices(roa_F_F, frame_camera1,
        #     self._roi_model_camera1, "Image1")

        # # Get seperated image for camera 1
        # seperated_img_cam1 = self.generate_seperated_image(frame_camera1,
        #     rsp_fs_fs_vec_cam1)

        

        # # Convert needle to 1D for camera 1
        # (seperated_img_cam1_1D, rsp_fs_fs_cam1_1D_pc) = \
        #     self.convert_needle_pixels_to_1D( seperated_img_cam1)

        ###################### Camera 2 ######################

        # Get indices of pixels of interest for camera 2 (poi)
        rsp_fs_fs_vec_cam2 = self.get_poi_indices(roa_F_F, frame_camera2, 
            self._roi_model_camera2, "Image2")

        # Get seperated image for camera 2
        seperated_img_cam2 = self.generate_seperated_image(frame_camera2,
            rsp_fs_fs_vec_cam2)
            
            
        # # Convert needle to 1D for camera 2
        # (seperated_img_cam2_1D, rsp_fs_fs_cam2_1D_pc) = \
        #     self.convert_needle_pixels_to_1D( seperated_img_cam2)

        # ################### Corresponding Points ###################

        # # Corresponding points
        # crp_points_num = 10

        # (rsp_fs_fs_crp_cam1, rsp_fs_fs_crp_cam2) = \
        #     self.generate_corresponding_points(rsp_fs_fs_cam1_1D_pc, \
        #         rsp_fs_fs_cam2_1D_pc, crp_points_num)

        # # Visualize corresponding points
        # self.visualize_correspnding_points(rsp_fs_fs_crp_cam1, \
        #     rsp_fs_fs_crp_cam2, seperated_img_cam1_1D, seperated_img_cam2_1D)
            


    # Plot seperated image
    def generate_seperated_image(self, frame, rsp_fs_fs_vec):

        # Initialize seperated image
        seperated_image = np.zeros((frame.shape[0],frame.shape[1]), np.uint8)

        # Generate seperated image
        for rsp_fs_fs in rsp_fs_fs_vec:
            seperated_image[rsp_fs_fs[1], rsp_fs_fs[0]] = 255

        return seperated_image

    # Get indices of pixels of interest (poi)
    def get_poi_indices(self, roa_F_F, frame, roi_model, name):
        
        # Get region of interest (crop frame)
        frame_cropped = roi_model.calculate_roi(roa_F_F, frame)

        # Get cropped image origin
        cropped_origin = roi_model.get_cropped_image_origin()

        # Extract the background
        res = self.background_extraction(frame_cropped)

        # Get binary indices (in cropped image)
        binary_indices = np.where(res==255)           

        # Generate binary indices rows and cols (in uncroppedcropped image)
        indices_x =  binary_indices[1] + cropped_origin[0]
        indices_y =  binary_indices[0] + cropped_origin[1]

        # Initialize rsp_fs_fs_vec
        rsp_fs_fs_vec = np.zeros((len(indices_x), 2, 1))

        # Loop through all points
        for i in range(len(indices_x)):
            rsp_fs_fs_vec[i] = np.array([ [indices_x[i]], [indices_y[i]] ])

        return rsp_fs_fs_vec.astype(int)
        
    # Extract background image
    def background_extraction(self, frame):
        # Copy img
        img = frame.copy()

        # If the image is rgb convert to gray
        if (len(frame.shape) == 3):
        
            # Convert image to gray-scale
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Blurred image (denoise)
        gray_blurred = cv2.medianBlur(img, 15)

        # Threshold image(100)
        ret,th2 = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)

        return th2

    # Plot frames 
    def plot_frames(self, frame_camera1, rs1p1_fs1_fs1_cam1, frame_camera2,
        rs1p1_fs1_fs1_cam2):

        cv2.circle(frame_camera1, (int(rs1p1_fs1_fs1_cam1[0]),
            int(rs1p1_fs1_fs1_cam1[1])), 3, (255,0,0), -1)
        cv2.circle(frame_camera2, (int(rs1p1_fs1_fs1_cam2[0]),
            int(rs1p1_fs1_fs1_cam2[1])), 3, (255,0,0), -1)

        cv2.imshow("Camera 1", frame_camera1)
        cv2.imshow("Camera 2", frame_camera2)
        cv2.waitKey(2)

    # Convert needle pixels to 1D lines
    def convert_needle_pixels_to_1D(self, img):

        # Genrate an empty image 
        img_1D = 0 * np.copy(img)

        # Get the number of columns of img
        rows = img.shape[0]
        cols = img.shape[1]

        # Define indices array
        rows_indices = np.arange(0, rows, 1)
        rows_indices = np.reshape(rows_indices, (rows_indices.shape[0], 1))

        # Average index vector
        avg_idx_vec = []

        # Point cloud of needle indices
        rsp_fs_fs_dir1 = []
        rsp_fs_fs_dir2 = []

        for i in range(cols):
            
            # Get column i
            col_i = img[:, i]
            col_i = np.reshape(col_i, (col_i.shape[0], 1))

            # Get the nonzero elements of column i
            (non_zero_indices, a) = np.nonzero(col_i)

            # If it returns a non empty list find average
            if non_zero_indices.size != 0:

                avg_col_i = int(np.average(non_zero_indices))

                img_1D[avg_col_i, i] = 255 

                rsp_fs_fs_dir2.append(avg_col_i)
                rsp_fs_fs_dir1.append(i)
        
        rsp_fs_fs_pc = np.zeros((len(rsp_fs_fs_dir1), 2))
        rsp_fs_fs_pc[:, 0] = rsp_fs_fs_dir1
        rsp_fs_fs_pc[:, 1] = rsp_fs_fs_dir2

        return (img_1D, rsp_fs_fs_pc)

    # Generate corrsponding points
    def generate_corresponding_points(self, rsp_fs_fs_cam1_1D_pc, \
        rsp_fs_fs_cam2_1D_pc, crp_points_num):

        # Calculate lenths of crp points
        length_percentage = np.linspace(0, 1, crp_points_num)

        # Corresponding points image 1
        rsp_fs_fs_crp_cam1 = self.corresponding_points_per_image(\
            length_percentage, rsp_fs_fs_cam1_1D_pc, 1)

        print(rsp_fs_fs_crp_cam1)

        # Corresponding points image 2
        rsp_fs_fs_crp_cam2 = self.corresponding_points_per_image(\
            length_percentage, rsp_fs_fs_cam2_1D_pc, 2)

        return (rsp_fs_fs_crp_cam1.astype(int), rsp_fs_fs_crp_cam2.astype(int))

    # Get correspnding points per image
    def corresponding_points_per_image(self, length_percentage, pc, camera_id):

        # Calculate needle length and accumulative length (in pixels)
        (needle_length, needle_accum_length) = self.needle_pixel_length(pc)
        
        # Calculate corrensponding points length
        crp_points_length = needle_length * length_percentage        

        # Indices of corresponding points
        indices_crp = np.searchsorted(needle_accum_length, crp_points_length, \
            side="right")

        # Reverse the order of indices for the first camera
        if camera_id == 1:
            indices_crp = indices_crp[::-1]
        
        return pc[indices_crp]
        

    # Calculate needle length (in pixels)
    def needle_pixel_length(self, pc_1D):
        
        # Initialize length
        length = 0

        # Initialize accumulative length
        accumulative_length = []

        for i in range(1, pc_1D.shape[0]):

            # x coordinates difference
            x_coord_diff = pc_1D[i, 0] - pc_1D[i-1, 0]

            # y coordinates difference
            y_coord_diff = pc_1D[i, 1] - pc_1D[i-1, 1]

            # Calculate length
            length += np.sqrt( pow(x_coord_diff, 2.0) + pow(y_coord_diff, 2.0) ) 

            # Append length
            accumulative_length.append(length)

        return (length, accumulative_length)

    # Visualize corresponding points
    def visualize_corresponding_points(self, rsp_fs_fs_crp_cam1, \
        rsp_fs_fs_crp_cam2, seperated_img_cam1_1D, seperated_img_cam2_1D):
        
        # Number of corresponding points
        crp_points_num = rsp_fs_fs_crp_cam1.shape[0]

        # Convert images to rgb for visualization
        seperated_img_cam1_1D = cv2.cvtColor(seperated_img_cam1_1D, cv2.COLOR_GRAY2RGB)
        seperated_img_cam2_1D = cv2.cvtColor(seperated_img_cam2_1D, cv2.COLOR_GRAY2RGB)

        for i in range(0, crp_points_num):
            
            # Random color
            color = list(np.random.choice(range(256), size=3).astype(int))
            color_tuple = (int(color[0]), int(color[1]), int(color[2]))
            
            cv2.circle(seperated_img_cam1_1D, (rsp_fs_fs_crp_cam1[i][0], \
                rsp_fs_fs_crp_cam1[i][1]), 3, color_tuple, -1)

            # Corresponding points image 2
            cv2.circle(seperated_img_cam2_1D, (rsp_fs_fs_crp_cam2[i][0], \
                rsp_fs_fs_crp_cam2[i][1]), 3, (color_tuple), -1)

        cv2.imshow("Image 1", seperated_img_cam1_1D)
        cv2.imshow("Image 2", seperated_img_cam2_1D)
        cv2.waitKey(0)

