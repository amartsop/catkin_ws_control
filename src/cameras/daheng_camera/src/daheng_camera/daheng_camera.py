#!/usr/bin/env python

from matplotlib.pyplot import gray
import rospy
import gxipy as gx
import numpy
import cv2
import os
import csv 

class DahengCamera:

    def __init__(self, camera_gain=24.0, des_fps=25, camera_index=1, gray_flag=False):

        # Set images gain, exposure time(us), and index
        self._camera_gain = camera_gain;
        self._des_fps = des_fps
        self._camera_index = camera_index
        self._gray_flag = gray_flag

        # Calculate required exposure time
        self._exposure_time = 1.0e6 / self._des_fps

        # Fps tracker
        self._fps_counter = 0
        self._required_fps = 500

        # Constant camera parameters
        self._width = 1440
        self._height = 1080
        self._offset_x = 0
        self._offset_y = 0

        # Setup camera
        self.camera_setup()


####################################################################

    # Convert raw images to CV images
    def raw_to_cv(self, raw_image, gray_flag):

        # get RGB image from raw image
        rgb_image = raw_image.convert("RGB")
        if rgb_image is None:
            print("Conversion to RGB image failed.")

        # improve image quality
        rgb_image.image_improvement(self.color_correction_param,
            self.contrast_lut, self.gamma_lut)

        # create numpy array with data from raw image
        numpy_image = rgb_image.get_numpy_array()

        if numpy_image is None:
            print("Conversion to numpy image failed.")
        
        # Initialize image
        np_img = None

        # Check color scheme
        if gray_flag:
            np_img = cv2.cvtColor(numpy.asarray(numpy_image), cv2.COLOR_BGR2GRAY)
        else:
            np_img = cv2.cvtColor(numpy.asarray(numpy_image), cv2.COLOR_BGR2RGB)

        return (np_img)


####################################################################

    # Camera Settings Initialization
    def camera_setup(self):

        # create a device manager
        self.device_manager = gx.DeviceManager()
        self.dev_num, self.dev_info_list = self.device_manager.update_device_list()

        if self.dev_num is 0:
            print("Number of enumerated devices is 0")
            return
        
        # open the first device
        self.cam = self.device_manager.open_device_by_index(self._camera_index)

        # set Width
        self.cam.Width.set(self._width)

        # set Height
        self.cam.Height.set(self._height)

        # offset width
        self.cam.OffsetX.set(self._offset_x)

        # offset width
        self.cam.OffsetY.set(self._offset_y)
    
        # set continuous acquisition
        self.cam.TriggerMode.set(gx.GxSwitchEntry.OFF)

        # set exposure
        self.cam.ExposureTime.set(self._exposure_time)

        # set gain
        self.cam.Gain.set(self._camera_gain)

        # set auto white balance
        self.cam.BalanceWhiteAuto.set(1)

        # User Set Selector
        self.cam.UserSetSelector.set(1)
        self.cam.UserSetSave.send_command()

        # get param of improving image quality
        if self.cam.GammaParam.is_readable():
            gamma_value = self.cam.GammaParam.get()
            self.gamma_lut = gx.Utility.get_gamma_lut(gamma_value)
        else:
            self.gamma_lut = None

        if self.cam.ContrastParam.is_readable():
            contrast_value = self.cam.ContrastParam.get()
            self.contrast_lut = gx.Utility.get_contrast_lut(contrast_value)
        else:
            self.contrast_lut = None

        if self.cam.ColorCorrectionParam.is_readable():
            self.color_correction_param = self.cam.ColorCorrectionParam.get()
        else:
            self.color_correction_param = 0

        # set the acq buffer count
        self.cam.data_stream[0].set_acquisition_buffer_number(2)


####################################################################

    # Get exposure time
    def get_exposure_time_us(self):
        return self._exposure_time


####################################################################

    # Activate stream
    def activate_stream(self):
        self.cam.stream_on()


####################################################################

    # Deactivate stream
    def deactivate_stream(self):
        self.cam.stream_off()


####################################################################

    # Get image width
    def get_image_width(self):
        return self._width


####################################################################

    # Get image height
    def get_image_height(self):
        return self._height


####################################################################

    # Get cv image
    def get_cv_image(self):

        # Get raw image
        raw_image = self.cam.data_stream[0].get_image()

        if raw_image is None:
            print("Getting image failed.")

        # Get CV image
        return self.raw_to_cv(raw_image, self._gray_flag)


####################################################################

    # Online camera stream
    def online_stream(self, name="Image"):

        # Open stream
        self.cam.stream_on()

        try:
            while True:

                # Get cv image
                pimg = self.get_cv_image()

                # Show cv image
                cv2.imshow(name, pimg)
                cv2.waitKey(10)

        except KeyboardInterrupt:
            pass

        # Close stream
        self.cam.stream_off()


####################################################################

    # Get one single frame (standalone function)
    def get_frame(self):

        # Activate stream
        self.cam.stream_on()

        # Get cv image 
        pimg = self.get_cv_image()

        # Deactivate stream
        self.cam.stream_off()

        return pimg


####################################################################

    # Save frame
    def save_frame(self, pimg, filename):
        cv2.imwrite(filename, pimg)


####################################################################

    def __del__(self):
        # close device
        self.cam.close_device()
        cv2.destroyAllWindows()  