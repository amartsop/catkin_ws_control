#!/usr/bin/env python

import numpy as np
from scipy.fft import dst
import rospy
import os 
import cv2
import csv 

import message_filters
from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from image_msgs.msg import ImagesSynchronizedCompressed
from cartesian_msgs.msg import CartesianPose

# from needle_detection.src.needle_detection.data_processing import DataProcessing
from needle_detection.data_processing import DataProcessing

class ImageController:

    def __init__(self):
        
        ##################### Synchronized subscriber ###################
        # Bridge handle
        self.cv_bridge = CvBridge()

        # Name of camera1 topic
        self.camera1_topic = "/compressed_camera1_images"
        
        # Name of camera2 topic
        self.camera2_topic = "/compressed_camera2_images"

        # Ee state topic
        self.ee_state_topic = "/lwa4p_end_effector_state"

        # Image encoding format
        self.img_format = "jpeg"

        # Subscribe to camera1 message
        camera1_sub = message_filters.Subscriber(self.camera1_topic,
            CompressedImage)

        # Subscribe to camera2 message
        camera2_sub = message_filters.Subscriber(self.camera2_topic,
            CompressedImage)

        # Subscribe to end-effector state message
        ee_sub = message_filters.Subscriber(self.ee_state_topic,
            CartesianPose)

        # Time sychronizer for the two cameras
        ts = message_filters.ApproximateTimeSynchronizer([camera1_sub,
            camera2_sub, ee_sub], queue_size=10, slop=0.01)

        # Frames subscription flag
        self.frames_sub_flag = False

        ts.registerCallback(self.get_frames_callback)

        ##################### Syncronized images publisher ###################

        # Sychronized image publisher
        self.synced_images_pub = rospy.Publisher("/synced_compressed_cameras_images",
            ImagesSynchronizedCompressed, queue_size=10)

        # Image from camera 1
        self.img_camera1_cv2 = None

        # Image from camera 2
        self.img_camera2_cv2 = None

        # Compressed images
        self.img_camera1_cmp = None
        self.img_camera2_cmp = None
        
        # Synced images visualization publishing interval
        des_fps = 30.0
        self.synced_images_pub_interval = 1.0 / des_fps
        
        # Data processing algorithm
        self.dp = DataProcessing()

    # Get frames callback function
    def get_frames_callback(self, camera1_msg, camera2_msg, ee_pose_msg):
        
        # Update subscription flag        
        self.frames_sub_flag = True

        # Copy compressed images
        self.img_camera1_cmp = camera1_msg.data        
        self.img_camera2_cmp = camera2_msg.data

        # Get camera 1 data
        self.img_camera1_cv2 = \
            cv2.imdecode(np.frombuffer(camera1_msg.data, dtype=np.int8),
            cv2.IMREAD_UNCHANGED)
        
        # Get camera 2 data
        self.img_camera2_cv2 = \
            cv2.imdecode(np.frombuffer(camera2_msg.data, dtype=np.int8),
            cv2.IMREAD_UNCHANGED)

        # End-effector pose
        ee_pose = np.array([[ee_pose_msg.rop_F_F.x],
            [ee_pose_msg.rop_F_F.y], [ee_pose_msg.rop_F_F.z], 
            [ee_pose_msg.euler.x], [ee_pose_msg.euler.y], 
            [ee_pose_msg.euler.z]])

        # # Do some processing....
        # self.dp.process_data(self.img_camera1_cv2, self.img_camera2_cv2, ee_pose)


    # Publish synced images
    def publish_synced_images(self, event=None):
        
        # Initialize messgae
        msg = ImagesSynchronizedCompressed()

        if (self.frames_sub_flag):
            
            # Set message contents
            msg.header.stamp.secs = rospy.Time.now().secs
            msg.header.stamp.nsecs = rospy.Time.now().nsecs
        
            # Images format
            msg.camera1.format = self.img_format
            msg.camera2.format = self.img_format

            # Images data
            msg.camera1.data = self.img_camera1_cmp
            msg.camera2.data = self.img_camera2_cmp
        
            # Publish msg
            self.synced_images_pub.publish(msg)


    # Get the publishing interval of the synced images
    def get_synced_images_publishing_interval(self):
        return self.synced_images_pub_interval

if __name__ == '__main__':
    try:
        # Initialize node        
        rospy.init_node("image_controller")

        # Initialize image controller handle
        image_controller = ImageController()

        # Create timer for publishing synced images (visualization)
        rospy.Timer(rospy.Duration(
            image_controller.get_synced_images_publishing_interval()),
            image_controller.publish_synced_images)

        # Spin
        rospy.spin()

    except rospy.ROSInterruptException:
        pass