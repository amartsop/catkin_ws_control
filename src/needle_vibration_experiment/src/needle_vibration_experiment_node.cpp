#include <iostream>
#include "ros/ros.h"
#include "../include/nve_application.h"

int main(int argc, char** argv)
{
    // Initiate sync_test node 
    ros::init(argc, argv, "needle_vibration_experiment"); 

    // Create ros node handle
    ros::NodeHandle node_handle;

    // Initiate asychronous callback check
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Initialize nve application
    NveApplication nve_app(&node_handle);

    // Main ros loop
    while(ros::ok())
    {
        ros::spinOnce();
    }
    
    // Shutdown node
    ros::shutdown();

    return 0;
}










    // // FT Sensor Subscriber
    // message_filters::Subscriber<geometry_msgs::WrenchStamped>
    //     ft_data_sub(node_handle, ft_proc.get_ft_sensor_topic_name(), 1000);

    // // End-Effector State Subscriber
    // message_filters::Subscriber<cartesian_msgs::CartesianPose>
    //     ee_state_sub(node_handle, ft_proc.get_ee_state_topic_name(), 1000);
    
    // // Create approximate time sychronizer
    // typedef message_filters::sync_policies::ApproximateTime<
    //     geometry_msgs::WrenchStamped, cartesian_msgs::CartesianPose> sync_policy;

    // message_filters::Synchronizer<sync_policy> sync(sync_policy(10),
    //     ft_data_sub, ee_state_sub);

    // // Register callback function
    // sync.registerCallback(&FTProcessing::ft_ee_callback, &ft_proc);

