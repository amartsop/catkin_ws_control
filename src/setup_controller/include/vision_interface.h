#pragma once

#include <iostream>
#include "ros/ros.h"
#include "camera_window.h"
#include <image_msgs/ImagesSynchronizedCompressed.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

class VisionInterface
{
public:
    // Constructor
    VisionInterface(ros::NodeHandle* nh,
        std::shared_ptr<CameraWindow> camera1_window_ptr, 
        std::shared_ptr<CameraWindow> camera2_window_ptr);
private:

    /******************** Syncronized cameras subscriber ****************/
    // Synced cameras topic name
    const std::string m_synced_cameras_topic_name =
        "synced_compressed_cameras_images";
    
    //Synced cameras subscrimer
    ros::Subscriber m_synced_cameras_sub; 

    // Synced cameras callback function
    void synced_cameras_callback(const image_msgs::ImagesSynchronizedCompressedConstPtr&
        msg);

    // Check if we have subscribed to the topic
    bool m_synced_cameras_sub_flag = false;

private:
    
    // Camera 1 window pointer
    std::shared_ptr<CameraWindow> m_camera1_window_ptr;
    
    // Camera 2 window pointer
    std::shared_ptr<CameraWindow> m_camera2_window_ptr;

    // Initial time
    ros::Time m_time_init;

    // Streaming frequency (Hz / FPS)
    double m_streaming_frequency = 30;
    double m_streaming_interval = 1.0 / m_streaming_frequency;

    bool m_pending_start = true;
};