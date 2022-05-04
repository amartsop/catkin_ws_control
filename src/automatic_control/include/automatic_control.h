#pragma once

#include <iostream>

#include "ros/ros.h"

#include "smch_msgs/State.h"
#include "cartesian_msgs/CartesianPose.h"
#include "cartesian_msgs/CartesianGoal.h"
#include "parameters_msgs/NeedleVibrationExperiment.h"

#include "needle_experiment_trajectory.h"
#include <dynamics_math_eigen/dynamics_math_eigen.h>

class AutomaticControl
{

public:
    AutomaticControl(ros::NodeHandle *nh);

private: 
    // Update system
    void update(double delta_time);

private:
    
    /************************ System State Subscriber *********************/

    // System state subscriber    
    std::string m_system_state_topic_name = "system_state";

    // System state callback function
    void system_state_callback(const smch_msgs::StateConstPtr& msg);

    // Define subscriber to system state topic
    ros::Subscriber m_system_state_sub;

    // System state 
    int m_state_id;

    // Check if we are pending to start
    bool m_system_state_sub_pending_start = false;

private:

    /******************* End-Effector State Topic Subscriber *****************/
    // Lwa4p subscriber ee topic name
    std::string m_ee_state_topic_name = "lwa4p_end_effector_state";
    
    // Lwa4p ee state subscriber
    ros::Subscriber m_ee_state_sub; 

    // End-effector state callback function
    void ee_state_callback(const
        cartesian_msgs::CartesianPoseConstPtr& msg);

    // Check if we have subscribed to the topic
    bool m_ee_state_sub_flag = false;

    // End-effector pose
    dme::Cartesian m_ee_pose;

private:

    /************* Desired End-Effector Pose Topic Publisher *****************/

    // Publisher handle for the desired lwa4p ee pose
    ros::Publisher m_lwa4p_des_ee_pose_pub;

    // Desired lwa4p ee state topic name
    const std::string m_lwa4p_des_ee_pose_topic_name =
        "lwa4p_autonomous_desired_end_effector_state";

    // Desired lwa4p ee pose publishing interval
    const double m_des_ee_pose_pub_interval = 1.0 / 300.0;

    // Desired joint configuration timer
    ros::Timer m_des_ee_pose_timer;

    // Publish autonomous desired ee state pose
    void publish_autonomous_desired_ee_pose(const ros::TimerEvent& event);

    // Desired end effector pose
    dme::Cartesian m_des_ee_pose;

private:

    // Needle experiment trajectory
    NeedleExperimentTrajectory m_needle_experiment_trajectory;

    // Needle experiment pending start
    bool m_needle_experiment_pending_start = true;

    // Needle experiment initial time
    ros::Time m_needle_experiment_initial_time;

    // Needle experiment duration
    double m_needle_experiment_duration;

    // Current time counter
    double m_needle_experiment_time = 0;

    int m_counter = 0;

private:

    /******* Needle Vibration Experiment Parameters Topic Subscriber ********/
    // Nve parameter topic name
    const std::string m_nve_params_topic_name =
        "needle_vibration_experiment_parameters";
    
    // Nve subscriber
    ros::Subscriber m_nve_sub; 

    // Nve callback function
    void nve_prameters_callback(const
        parameters_msgs::NeedleVibrationExperimentConstPtr& msg);

    // Check if we have subscribed to the topic
    bool m_nve_sub_flag = false;

    // Position amplitude (m), frequency (Hz), phase (rad)
    Eigen::Vector3d m_pos_a;
    Eigen::Vector3d m_pos_f;
    Eigen::Vector3d m_pos_p;

    // Rotation amplitude (m), frequency (Hz), phase (rad)
    Eigen::Vector3d m_rot_a;
    Eigen::Vector3d m_rot_f;
    Eigen::Vector3d m_rot_p;

private:
    /*************** Axia sensor debugging ********************/

    // Distance of the sensor's center of mass Ct wrt to the fe frame expressed 
    // in the fe frame
    Eigen::Vector3d m_rect_fe_fe;

    // The position vector of the the ft frame with respect to the fe frame 
    // expressed in the fe frame
    Eigen::Vector3d m_ret_fe_fe = {0.0604, 0.0, -0.04515};

    // Distance of the sensor's center of mass Ct wrt to the measuring centre T (m)
    Eigen::Vector3d m_rtct_ft_ft = {0.0, 0.0, 0.0};

    // The rotation matrix of the ft frame with respect to the fe frame 
    Eigen::Matrix3d m_rot_ft_fe = Eigen::Matrix3d::Zero(3, 3);

};