#pragma once

#include <iostream>
#include <memory>
#include "ros/ros.h"

#include "control_msgs/FollowJointTrajectoryAction.h"
#include <actionlib/client/simple_action_client.h>

typedef control_msgs::FollowJointTrajectoryGoal fktg;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TrajClient;

class RobotControl
{
public:
    RobotControl(ros::NodeHandle *nh, std::shared_ptr<TrajClient> traj_client);

private:

    // Desired robot joint configuration topic name
    std::string m_des_joint_config_topic_name =
        "lwa4p_desired_joint_configuration";
    
    // Desired robot joint configuration callback
    void des_joint_config_callback(const
        control_msgs::FollowJointTrajectoryGoalPtr& msg);

    // Define subscriber to the desired joint configuration
    ros::Subscriber m_des_joint_config_sub;

    // Check if we have subscribed to the topic
    bool m_des_joint_config_flag = false;

    // Trajectory client shared pointerj 
    std::shared_ptr<TrajClient> m_traj_client;    
};