#pragma once

#include <iostream>
#include <memory>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "control_panel_window.h"
#include "parameters_msgs/NeedleVibrationExperiment.h"
#include "parameters_msgs/Vision.h"

// Parameters interface
class ParametersInterface
{

public:
    // Constructor
    ParametersInterface(ros::NodeHandle *nh,
        std::shared_ptr<ControlPanelWindow> cp_window);
    
private:
    
    // Control panel window ptr
    std::shared_ptr<ControlPanelWindow> m_cp_window_ptr;
    
private:

    /****************** Nve parameters publisher *****************/
    // Publisher handle for the nve parameters
    ros::Publisher m_nve_params_pub;

    // Nve parameter topic name
    const std::string m_nve_params_topic_name =
        "needle_vibration_experiment_parameters";

    // Nve parameters publishing interval
    const double m_nve_params_pub_interval = 1.0 / 30.0;

    // Nve parameters publishing timer
    ros::Timer m_nve_params_timer;

    // Publish nve parameters
    void publish_nve_parameters(const ros::TimerEvent& event);

private:
    /****************** Vision parameters publisher *****************/
    // Publisher handle for the vision parameters
    ros::Publisher m_vision_params_pub;

    // Vision parameter topic name
    const std::string m_vision_params_topic_name = "vision_parameters";

    // Nve parameters publishing interval
    const double m_vision_params_pub_interval = 1.0 / 30.0;

    // Vision parameters publishing timer
    ros::Timer m_vision_params_timer;

    // Publish vision parameters
    void publish_vision_parameters(const ros::TimerEvent& event);

};