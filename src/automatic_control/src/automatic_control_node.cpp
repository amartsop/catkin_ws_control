#include <iostream>
#include "ros/ros.h"

#include <cstdlib>
#include <iostream>
#include <armadillo>
#include <ostream>
#include <vector>
#include <chrono>
#include <thread>

#include "../include/automatic_control.h"

int main(int argc, char** argv)
{
    // Initiate sync_test node 
    ros::init(argc, argv, "automatic_control_node"); 

    // Create ros node handle
    ros::NodeHandle node_handle;

    // Initiate asychronous callback check
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Initialize automatic control
    AutomaticControl automatic_control(&node_handle);

    /************************* Main loop **************************/
    while(ros::ok())
    {
        // Update ROS
        ros::spinOnce();
    }
    
    // Shutdown node
    ros::shutdown();

    return 0;
}