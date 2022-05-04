#pragma once 

#include "ros/ros.h"
#include <iostream>
#include <memory>
#include "std_msgs/Bool.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

#include "cartesian_msgs/CartesianPose.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "low_pass_filter_eigen.h"
#include <dynamics_math_eigen/dynamics_math_eigen.h>


class Lwa4pStateHandler
{
public:
    // Constructor
    Lwa4pStateHandler(ros::NodeHandle *nh,
        robot_model::RobotModelPtr kinematic_model);

    // Publish end-effector (ee) state
    void publish_ee_state(const ros::TimerEvent& event);

    // Return the ee publishing rate 
    double get_ee_state_publishing_rate(void) { return m_ee_pub_interval; }

private:

    // Arm joint state topic
    std::string m_arm_joint_state_topic = "lwa4p_arm/arm_controller/state";

    // Define subscriber to the arm's state topic
    ros::Subscriber m_joint_state_sub;

    // Joint state callback function
    void joint_state_callback(const
        control_msgs::JointTrajectoryControllerStateConstPtr& msg);

    // Check if we have subscribed to the topic
    bool m_joint_state_sub_flag = false;

    // Joint positions
    std::vector<double> m_joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Joint velocities
    std::vector<double> m_joint_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Joint accelerations
    std::vector<double> m_joint_accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

private:

    // Publisher handle for ee state
    ros::Publisher m_ee_state_pub;

    // Topic name 
    const std::string m_ee_state_topic_name = "lwa4p_end_effector_state";

    // Publishing interval
    const double m_ee_pub_frequency = 130.0;
    const double m_ee_pub_interval = 1.0 / m_ee_pub_frequency;

    // Recording state
    cartesian_msgs::CartesianPose m_ee_state;

private:

    // Kinematic state ptr handle 
    std::shared_ptr<robot_state::RobotState> m_kinematic_state;

    // Group joint model
    robot_state::JointModelGroup* m_joint_model_group;

    // Joint names
    std::vector<std::string> m_joint_names;

    // Link names
    std::vector<std::string> m_link_names;

    // End effector name
    std::string m_ee_name;

private:
    // Current and previous time (for time derivative)
    ros::Time time_prev, time_current;

    // Previous linear velocity (for time derivative)
    Eigen::Vector3d rop_F_F_dot_prev = {0.0, 0.0, 0.0};

    // Previous angular velocity 
    Eigen::Vector3d omega_f_F_F_prev = {0.0, 0.0, 0.0};

    // Previous derivative of ksi (for time derivative)
    Eigen::VectorXd m_ksi_prev = Eigen::VectorXd::Zero(6);

private:

    // Convert an std::vector<double> to Eigen::VectorXd
    Eigen::VectorXd vec_to_eigen(const std::vector<double>& a);

private:
    /************************ Acceleration Filter ********************/

    // Filter cut-off frequency for end-effector's acceleration (Hz)
    double m_ee_acceleration_filter_cutoff_freq = 2.0;

    // Filter order end-effector's acceleration 
    int m_ee_acceleration_filter_order = 2;

    // Filter for end-effector's velocity 
    std::shared_ptr<LowPassFilterEigen> m_ee_acceleration_filter;
};