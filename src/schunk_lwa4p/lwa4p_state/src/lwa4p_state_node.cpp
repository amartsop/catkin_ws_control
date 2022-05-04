#include <iostream>
#include "ros/ros.h"
#include "../include/lwa4p_state_handler.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    // Initiate sync_test node 
    ros::init(argc, argv, "lwa4p_state"); 

    // Create ros node handle
    ros::NodeHandle node_handle;

    // Initiate asychronous callback check
    ros::AsyncSpinner spinner(0);
    spinner.start();

    /***************************** Robot state **************************/
    // Create kinematic robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    // Initialize state subscriber and publisher 
    Lwa4pStateHandler lwa4p_state_handler(&node_handle, kinematic_model);

    // Set interval for writting the end effector state
    ros::Timer timer = node_handle.createTimer(
        ros::Duration(lwa4p_state_handler.get_ee_state_publishing_rate()),
        &Lwa4pStateHandler::publish_ee_state, &lwa4p_state_handler);

    /************************* Main loop **************************/
    while(ros::ok())
    {
        ros::spinOnce();
    }
    
    // Shutdown node
    ros::shutdown();

    return 0;
}