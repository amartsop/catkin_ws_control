#pragma once

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "cartesian_msgs/CartesianPose.h"
#include <dynamics_math_eigen/dynamics_math_eigen.h>

class NveFTProcessing
{
public:
    NveFTProcessing(ros::NodeHandle *nh);

private:

    /********************* Axia ft data subscriber *******************/
    // FT sensor topic name
    std::string m_ft_sensor_topic_name = "axia_ft_data";

    // FT sensor subscriber
    ros::Subscriber m_ft_data_sub;

    // FT sensor callback function
    void ft_sensor_callback(const geometry_msgs::WrenchStampedConstPtr& msg);

    // Check if we have subscribed to the topic
    bool m_ft_data_sub_flag = false;

    // Ft data (with respect to ft frame)
    dme::FTData m_ft_data_ft;    

    /********************* End-Effector State Subscriber *******************/
    // Lwa4p subscriber ee topic name
    std::string m_ee_state_topic_name = "lwa4p_end_effector_state";

    // End-effector state subscriber
    ros::Subscriber m_ee_state_sub;

    // End-effector state callback function
    void ee_state_callback(const cartesian_msgs::CartesianPoseConstPtr& msg);

    // Check if we have subscribed to the topic
    bool m_ee_state_sub_flag = false;

    // End effector state (with respect to fr frame)
    dme::Cartesian m_ee_state;


private:

    /************* Handle FT Data Publisher *****************/
    // Publisher for handle's ft data
    ros::Publisher m_handle_ft_data_pub;

    // Handle ft data topic name
    const std::string m_handle_ft_data_topic_name = "handle_ft_data";

    // Handle ft data publishing interval
    double m_handle_ft_data_pub_frequency = 100.0;
    double m_handle_ft_data_pub_interval = 1.0 / m_handle_ft_data_pub_frequency;

    // Handle ft data timer
    ros::Timer m_handle_ft_data_timer;

    // Publish axia ft data
    void publish_handle_ft_data(const ros::TimerEvent& event);

    // Handle ft data (with respect to fc frame)
    dme::FTData m_hanlde_ft_data;    

private:
    
    // The rotation matrix of the fm frame with respect to the fe frame 
    Eigen::Matrix3d m_rot_fm_fe = Eigen::Matrix3d::Identity(3, 3);

    // The rotation matrix of the fm frame with respect to the ft frame 
    Eigen::Matrix3d m_rot_fm_ft = Eigen::Matrix3d::Zero(3, 3);

    // The position vector of the fm frame with respect to the fe frame 
    // expressed in the fe frame
    Eigen::Vector3d m_rem_fe_fe = {0.065, 0.0, -0.04515};

    // The position vector of the Am point with respect to the fm frame 
    // expressed in the fm frame
    Eigen::Vector3d m_rmam_fm_fm = {0.005397, 0.0, 0.0};

    // The position vector of the At point with respect to the fm frame 
    // expressed in the fm frame
    Eigen::Vector3d m_rmat_fm_fm = {-0.004603, 0.0, 0.0};

    // Support mass (kg)
    double m_mm = 60.0 / 1000.0;
    
    // Support inertia tensor
    Eigen::Matrix3d m_im = Eigen::Matrix3d::Zero(3, 3);

    // Weight vector of the support body
    Eigen::Vector3d m_wm_fr;

private:

    // The rotation matrix of the the ft frame with respect to the fe frame 
    Eigen::Matrix3d m_rot_ft_fe = Eigen::Matrix3d::Zero(3, 3);

    // The position vector of the At point with respect to the ft frame 
    // expressed in the ft frame
    Eigen::Vector3d m_rtat_ft_ft = {0.0, 0.0, 0.0};

private:

    // Gravity constant (m / s^2)
    double m_grav = 9.80665;

private:
    // Pending start in ft sensor callback
    bool m_ft_pending_start = true;

    // Force support offset
    dme::FTData m_ft_support_offset;

};
