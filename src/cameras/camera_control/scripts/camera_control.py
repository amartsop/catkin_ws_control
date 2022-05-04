#!/usr/bin/env python
import rospy
import sys
import cv2
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from daheng_camera.daheng_camera import DahengCamera


class CameraController:

    def __init__(self, topic_name, desired_fps, usb_port):

        # Create a ROS publisher
        self.image_publisher = rospy.Publisher(topic_name,
            CompressedImage, queue_size=10)

        # Bridge hande
        self.bridge = CvBridge()    

        # Desired fps
        self.des_fps = desired_fps

        # Gray flag
        gray_flag = True

        # Camera 1 handle
        self.camera = DahengCamera(24, desired_fps, usb_port, gray_flag)

        # Define encoding
        self.encoding = "mono8"
        
        # Image encoding format
        self.img_format = "jpeg"

        # Activate stream
        self.camera.activate_stream()
    
        # Encoding image
        self.enc_img = None
        
        # Encoding result (success or fail)
        self.enc_result = None
        

    # Get desired fps
    def get_desired_fps(self):
        return self.des_fps

    # Read callback
    def read_images(self, event=None):

        # Get the cv image
        cv_img = self.camera.get_cv_image() 
        
        if (cv_img is None):
            pass
        else:
            # Compress image
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
            self.enc_result, self.enc_img = \
                cv2.imencode('.jpeg', cv_img, encode_param)

        
    # Publish callback
    def publish_images(self, event=None):
        
        # Initialize message
        ci = CompressedImage()
        ci.header.stamp.secs = rospy.Time.now().secs
        ci.header.stamp.nsecs = rospy.Time.now().nsecs

        if (self.enc_img is None):
            pass
        else:
            # Generate data
            ci.data = self.enc_img.tobytes()
            ci.format = self.img_format

            # Publish the message
            self.image_publisher.publish(ci)

    # Desturctor
    def __del__(self):
        # close device
        self.camera.deactivate_stream()
    

def camera_control_node(node_name, topic_name, desired_fps, usb_port):

    # Initialize node        
    rospy.init_node(node_name)
    
    # Initialize camera controller
    cc = CameraController(topic_name, float(desired_fps), int(usb_port))

    # Images acquisition interval
    read_period = 1.0 / float(desired_fps)

    # Publishing interval
    pub_period = 1.0 / float(desired_fps)
    
    # Create a ROS Timer for reading data
    rospy.Timer(rospy.Duration(read_period), cc.read_images)       

    # Create timer for publishing data
    rospy.Timer(rospy.Duration(pub_period), cc.publish_images)
    
        
if __name__ == '__main__':

    try:
        if (len(sys.argv) < 5): 
            print("Usage: Need to specify node name, topic_name, \
                desired_fps and camera usb port")
        else:
            args = rospy.myargv(argv=sys.argv)
            # Initialize node
            camera_control_node(args[1], args[2], args[3], args[4])
            
            # Spin
            rospy.spin()

    except rospy.ROSInterruptException:
        pass