import numpy as np
import cv2

class RoiModel:

    def __init__(self, h, w, camera_model, camera_id):

        if (camera_id == 1):
            # Create roi points relative to frame f
            self._r_ar11_f_f = np.array([[0.0], [0.0], [h/2.0]])
            self._r_ar21_f_f = np.array([[w], [0.0], [h/2.0]])
            self._r_ar31_f_f = np.array([[w], [0.0], [-h/2.0]])
            self._r_ar41_f_f = np.array([[0.0], [0.0], [-h/2.0]])

        else:
            # Create roi points relative to frame f
            self._r_ar11_f_f = np.array([[0.0], [h/2.0], [0.0]])
            self._r_ar21_f_f = np.array([[w], [h/2.0], [0.0]])
            self._r_ar31_f_f = np.array([[w], [-h/2.0], [0.0]])
            self._r_ar41_f_f = np.array([[0.0], [-h/2.0], [0.0]])


        # Cropped image origin
        self._rs1r21_fs1_fs1 = []
        self._rs1r11_fs1_fs1 = []

        # Camera model
        self._camera_model = camera_model

        # Camera id 
        self._camera_id = camera_id

    def calculate_roi(self, roa_F_F, frame):

        # ROI points relative to frame without rotation (R_f_F = I3)
        r_or11_F_F = roa_F_F + self._r_ar11_f_f
        r_or21_F_F = roa_F_F + self._r_ar21_f_f
        r_or31_F_F = roa_F_F + self._r_ar31_f_f
        r_or41_F_F = roa_F_F + self._r_ar41_f_f

        # Object to sensor
        self._rs1r11_fs1_fs1 = self._camera_model.object_to_sensor(r_or11_F_F)
        self._rs1r21_fs1_fs1 = self._camera_model.object_to_sensor(r_or21_F_F)
        rs1r31_fs1_fs1 = self._camera_model.object_to_sensor(r_or31_F_F)
        rs1r41_fs1_fs1 = self._camera_model.object_to_sensor(r_or41_F_F)

        # Crop frame
        x1 = self.sature_pixels(int(self._rs1r21_fs1_fs1[0]))
        y1 = self.sature_pixels(int(self._rs1r21_fs1_fs1[1]))
        x2 = self.sature_pixels(int(rs1r41_fs1_fs1[0]))
        y2 = self.sature_pixels(int(rs1r41_fs1_fs1[1]))
        
        if (self._camera_id == 1):

            return frame[y1:y2, x1:x2]
        else: 
            return frame[y1:y2, x2:x1]
        
    # Saturate negative pixels
    def sature_pixels(self, x):

        if (x < 0):
            return 0
        else:
            return x

    # Return cropped image origin
    def get_cropped_image_origin(self):
        if (self._camera_id == 1):
            return self._rs1r21_fs1_fs1
        else:
            return self._rs1r11_fs1_fs1
