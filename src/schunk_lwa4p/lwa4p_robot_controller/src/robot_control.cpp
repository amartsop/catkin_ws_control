#include "../include/robot_control.h"

RobotControl::RobotControl(ros::NodeHandle *nh,
    std::shared_ptr<TrajClient> traj_client)
{
    // Copy trajectory client
    m_traj_client = traj_client; 

    // Subscribe to the desired joint configuration
    m_des_joint_config_sub = nh->subscribe(m_des_joint_config_topic_name, 1000,
        &RobotControl::des_joint_config_callback, this);

    // Wait to subscribe to topic
    while(!m_des_joint_config_flag) {
        ROS_INFO("Waiting to subscribe to desired joint configuration topic");
    };

    ROS_INFO("Succesfully subscribed to desired joint configuration topic");
}

// Callback to the desired robot joint configuration
void RobotControl::des_joint_config_callback(const
    control_msgs::FollowJointTrajectoryGoalPtr& msg)
{
    // Change subscribe flag
    m_des_joint_config_flag = true;

    m_traj_client->sendGoal(*msg);
}