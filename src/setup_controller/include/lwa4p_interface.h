#pragma once 

#include <iostream>
#include <memory>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include "control_msgs/JointTrajectoryControllerState.h"
#include "cartesian_msgs/CartesianPose.h"
#include "cartesian_msgs/CartesianGoal.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"

#include "control_panel_window.h"
#include "joystick_interface.h"
#include "utils.h"
#include <dynamics_math_eigen/dynamics_math_eigen.h>

class Lwa4pInterface
{

public:
    Lwa4pInterface(ros::NodeHandle *nh,
        std::shared_ptr<ControlPanelWindow> cp_window,
        JoystickInterface* joystick_interface);

    // Update 
    void update(int state);

public:

    // Get lwa4p joint angles
    std::vector<double> get_joint_angles(void) { return m_joint_angles; }

    // Get lwa4p end-effector state
    std::vector<double> get_ee_pose(void) { return m_ee_pose; }

private:

    // Control panel window ptr
    std::shared_ptr<ControlPanelWindow> m_cp_window_ptr;

    // Joystick interface
    JoystickInterface* m_joystick_interface;

private:

    // Robot model loader    
    robot_model_loader::RobotModelLoaderPtr m_robot_model_loader;

    // Kinematics base pointer
    kinematics::KinematicsBasePtr m_kinematics_base_ptr;

    // Joint names
    std::vector<std::string> m_joint_names;

    // Link names
    std::vector<std::string> m_link_names;

    // End effector name
    std::string m_ee_name;

    /**
     * @brief Lwa4p inverse kinematics routine
     * 
     * @param desired_ee_state Vector of cartesian ee state.
     * @return dme::Joint Caluclated joint angles and their derivatives
     */
    dme::Joint inverse_kinematics(const dme::Cartesian& des_ee_state);

private:

    /*********************** Joint State Topic Subscriber *****************/
    // Lwa4p subscriber joint state topic name
    std::string m_joint_state_topic_name = "lwa4p_arm/arm_controller/state";
    
    // Lwa4p joint state subscriber
    ros::Subscriber m_joint_state_sub; 

    // Joint state callback function
    void joint_state_callback(const
        control_msgs::JointTrajectoryControllerStateConstPtr& msg);

    // Check if we have subscribed to the topic
    bool m_joint_state_sub_flag = false;

    // Joint state
    dme::Joint m_joint_state;

    // A local copy of joint angles
    std::vector<double> m_joint_angles;

    // Number of joints angles
    const int m_joints_num = 6;
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

    // End-effector state    
    dme::Cartesian m_ee_state; 

    // A local copy of end-effector's pose (x, y, z, phi, theta, psi)
    std::vector<double> m_ee_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

private:

    /************* Desired End-Effector State Topic Subscriber *****************/
    // Lwa4p subscriber desired ee state topic name
    std::string m_des_ee_state_topic_name =
        "lwa4p_autonomous_desired_end_effector_state";
    
    // Lwa4p desired ee state subscriber
    ros::Subscriber m_des_ee_state_sub; 

    // Desired end-effector state callback function
    void des_ee_state_callback(const
        cartesian_msgs::CartesianGoalConstPtr& msg);

    // Check if we have subscribed to the topic
    bool m_des_ee_state_sub_flag = false;
    
    // Desired end effector state
    dme::Cartesian m_des_ee_state;

private:

    /****************** Desired Joint State Topic Publisher *****************/

    // Publisher handle for the desired joint state
    ros::Publisher m_des_joint_pub;

    // Desired joint configuration topic name
    const std::string m_des_joint_traj_topic_name =
        "lwa4p_desired_joint_configuration";

    // Desired joint configuration publishing interval
    const double m_des_joint_traj_pub_interval = 1.0 / 120.0;

    // Desired joint configuration timer
    ros::Timer m_des_joint_timer;

    // Publish desired joint trajectory configuration
    void publish_desired_joint_trajectory(const ros::TimerEvent& event);

    // Desired joint state
    dme::Joint m_des_joint_state;

private:
    // Manual joint control joystick index count
    int m_idx_count_manual_joint_joy = 0;

    // Manual joint control joystick index count dummy
    int m_idx_count_manual_joint_joy_dummy = 0;

    // Mnaual joint control joystick threshold for dummy variable
    int m_idx_count_manual_joint_joy_dummy_thresh = 12;

    // Manual joint control joystick step
    double m_manual_joint_joy_step = 0.005;

    // Manual ee control joystick step
    double m_manual_ee_pos_joy_step = 0.05;
    double m_manual_ee_eul_joy_step = 0.1;

private:
    // Lwa4p home position
    std::vector<double> m_lwa4p_home_position = { 0.0 * M_PI / 180.0,
        20.0 * M_PI / 180.0, 110.0 * M_PI / 180.0, 0.0 * M_PI / 180.0,
        90.0 * M_PI / 180.0, 0.0 * M_PI / 180.0};

    // Lwa4p home position
    std::vector<double> m_lwa4p_zero_position = { 0.0 * M_PI / 180.0,
        20.0 * M_PI / 180.0, 110.0 * M_PI / 180.0, 0.0 * M_PI / 180.0,
        0.0 * M_PI / 180.0, 0.0 * M_PI / 180.0};

    // Pending start flags (Gui joint input, Gui ee input, des_ee_state)
    std::vector<bool> m_pending_start = {true, true, true};
};