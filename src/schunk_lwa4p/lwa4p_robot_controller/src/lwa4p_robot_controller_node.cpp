#include <iostream>
#include <memory>
#include "ros/ros.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "../include/robot_control.h"

#include "control_msgs/FollowJointTrajectoryAction.h"
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TrajClient;

int main(int argc, char** argv)
{
    // Initiate sync_test node 
    ros::init(argc, argv, "lwa4p_robot_controller"); 

    // Create ros node handle
    ros::NodeHandle node_handle;

    // Initiate asychronous callback check
    ros::AsyncSpinner spinner(0);
    spinner.start();

    /***************************** Robot control **************************/
     // Create follow joint trajectroy action
    std::string traj_client_name = "lwa4p_arm/arm_controller/follow_joint_trajectory";
    std::shared_ptr<TrajClient> traj_client = 
        std::make_shared<TrajClient>(traj_client_name, true);
    
    // Wait to connect to the server
    while(!traj_client->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the joint_trajectory_action server");
    }   

    // Define robot control
    RobotControl robot_control(&node_handle, traj_client);

    /************************* Main loop **************************/
    while(ros::ok())
    {
        ros::spinOnce();
    }
    
    // Shutdown node
    ros::shutdown();

    return 0;
}