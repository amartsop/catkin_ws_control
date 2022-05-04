#pragma once
#include <iostream>
#include <memory>

#include "ros/ros.h"
#include <dynamics_math_eigen/dynamics_math_eigen.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "geometry_msgs/WrenchStamped.h"
#include "cartesian_msgs/CartesianPose.h"
#include "low_pass_filter_eigen.h"

class AxiaFTProcessing
{

public:
    // Constructor
    AxiaFTProcessing(ros::NodeHandle *nh);
    
    // Get FT sensor topic name
    std::string get_ft_sensor_topic_name(void) { return m_ft_sensor_topic_name; }
    
    // Get ee state topic name
    std::string get_ee_state_topic_name(void) { return m_ee_state_topic_name; }
    
    // Callback function for ft data and ee state
    void ft_ee_callback(const geometry_msgs::WrenchStampedConstPtr& ft_data,
        const cartesian_msgs::CartesianPoseConstPtr& ee_state);
    
private:

    /********************* FT Sensor Subscriber *******************/
    // FT sensor topic name
    std::string m_ft_sensor_topic_name = "ft_sensor/netft_data";
    
    // FT sensor data        
    dme::FTData m_ft_data;

    // FT sensor subscriber
    ros::Subscriber m_ft_data_sub;

    // FT sensor callback function
    void ft_sensor_callback(const geometry_msgs::WrenchStampedConstPtr& msg);

    // Check if we have subscribed to the topic
    bool m_ft_data_sub_flag = false;

    // Axia raw ft data (with respect to ft frame)
    dme::FTData m_axia_raw_ft_data;    

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

    /************* Axia Measurements Publisher *****************/
    // Publisher handle for axia ft data
    ros::Publisher m_axia_ft_data_pub;

    // Axia ft data topic name
    const std::string m_axia_ft_data_topic_name = "axia_ft_data";

    // Axia ft data publishing interval
    double m_axia_ft_data_pub_frequency = 100.0;
    double m_axia_ft_data_pub_interval = 1.0 / m_axia_ft_data_pub_frequency;

    // Axia ft data timer
    ros::Timer m_axia_ft_data_timer;

    // Publish axia ft data
    void publish_axia_ft_data(const ros::TimerEvent& event);

    // Filtered ft data (with respect to ft frame)
    dme::FTData m_filtered_ft_data;    

private:

    // Taring routine counter
    int m_taring_routine_counter = 0;    

    // Taring routine threshold
    int m_taring_routine_thresh = 100;

    // Forces and moments bias/offset (with respect to ft frame)
    dme::FTData m_ft_bias;

    // Pending start flag
    bool m_pending_start_flag = true;

private:

    /************************ Forces Filter ********************/

    // Filter cut-off frequency for forces (Hz)
    double m_forces_filter_cutoff_freq = 2.0;

    // Filter order for forces 
    int m_forces_filter_order = 2;

    // Filter for end-effector's velocity 
    std::shared_ptr<LowPassFilterEigen> m_forces_filter;

    /************************ Moments Filter ********************/

    // Filter cut-off frequency for forces (Hz)
    double m_moments_filter_cutoff_freq = 2.0;

    // Filter order for forces 
    int m_moments_filter_order = 2;

    // Filter for end-effector's velocity 
    std::shared_ptr<LowPassFilterEigen> m_moments_filter;

private:

    // Remove sensor inertia
    dme::FTData remove_sensor_inertia(const dme::Cartesian& ft_frame_state, 
        const dme::FTData& net_ft_data);

    // The position vector of the the ft frame with respect to the fe frame 
    // expressed in the fe frame
    Eigen::Vector3d m_ret_fe_fe = {0.0604, 0.05, -0.04515};

    // The rotation matrix of the the ft frame with respect to the fe frame 
    Eigen::Matrix3d m_rot_ft_fe = Eigen::Matrix3d::Zero(3, 3);

private:

    // Sensors mass (kg)
    double m_sensor_mass = 0.1512;

    // Sensors inertial tensor (kg * m^2)
    Eigen::Matrix3d m_sensor_inertia = Eigen::Matrix3d::Zero(3, 3);
    
    // Gravity constant (m / s^2)
    double m_grav = 9.80665;
    
    // Sensor's weight in the inertial frmae
    Eigen::Vector3d m_sensor_weight_fr;
    
    // Distance of the sensor's center of mass Ct wrt to the measuring centre T (m)
    Eigen::Vector3d m_rtct_ft_ft = {0.0, 0.0, -0.089};

    Eigen::Vector3d m_a = {0.0, 0.0, 0.0};
};