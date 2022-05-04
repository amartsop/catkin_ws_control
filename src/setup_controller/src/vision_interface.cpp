#include "../include/vision_interface.h"

VisionInterface::VisionInterface(ros::NodeHandle* nh,
    std::shared_ptr<CameraWindow> camera1_window_ptr, 
    std::shared_ptr<CameraWindow> camera2_window_ptr)
{
    // Store vision pointers to member variables
    m_camera1_window_ptr = camera1_window_ptr;
    m_camera2_window_ptr = camera2_window_ptr;
    
    /******************* Synced cameras Topic Subscriber *****************/
    // Subscribe to synced cameras topic    
    m_synced_cameras_sub = nh->subscribe(m_synced_cameras_topic_name, 1,
        &VisionInterface::synced_cameras_callback, this);

    m_time_init = ros::Time::now();
}

// Synced cameras callback function
void VisionInterface::synced_cameras_callback(const
    image_msgs::ImagesSynchronizedCompressedConstPtr& msg)
{
    // Update flag
    m_synced_cameras_sub_flag = true;

    if (m_pending_start)
    {
        ROS_INFO("Succesfully subscribed to synced cameras state topic");

        // Update pending start flag
        m_pending_start = false;
    } 
        
    // Get camera1 img
    cv::InputArray camera1_img_array = cv::InputArray(msg->camera1.data);
    cv::Mat camera1_img = cv::imdecode(camera1_img_array, cv::IMREAD_UNCHANGED);
    
    // Get camera2 img
    cv::InputArray camera2_img_array = cv::InputArray(msg->camera2.data);
    cv::Mat camera2_img = cv::imdecode(camera2_img_array, cv::IMREAD_UNCHANGED);

    // Set camera1 image
    m_camera1_window_ptr->set_frames(camera1_img);

    // Set camera2 image
    m_camera2_window_ptr->set_frames(camera2_img);
}
