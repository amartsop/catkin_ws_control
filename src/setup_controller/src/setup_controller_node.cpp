#include <iostream>
#include "ros/ros.h"
#include "ros/package.h"

#include <cstdlib>
#include <iostream>
#include <armadillo>
#include <ostream>
#include <vector>
#include <chrono>
#include <thread>

#include "../include/application_interface.h"
#include "../include/state_handler.h"



int main(int argc, char** argv)
{
    std::cout << ros::package::getPath("setup_controller");
    // Initiate sync_test node 
    ros::init(argc, argv, "setup_controller_node"); 

    // Create ros node handle
    ros::NodeHandle node_handle;

    // Initiate asychronous callback check
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Initialize application interface
    ApplicationInterface app;

    // Initialize application
    app.initialize(&node_handle);

    // Clock
    auto previous_time = std::chrono::steady_clock::now();
    double real_time = 0.0;

    /************************* Main loop **************************/
    while(ros::ok() && !app.is_done())
    {
        // Current time
        auto current_time = std::chrono::steady_clock::now();   

        // Elapsed time
        std::chrono::duration<double> elapsed_time = current_time - previous_time;

        // Real time 
        real_time += elapsed_time.count();
        
        // Render application 
        app.update(real_time, elapsed_time.count());
        
        // Update time 
        previous_time = current_time;
        
        // Update ROS
        ros::spinOnce();
    }
    
    // Shutdown node
    ros::shutdown();

    return 0;
}