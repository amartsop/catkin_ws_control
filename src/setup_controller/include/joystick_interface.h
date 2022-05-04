#pragma once
  
#include <iostream>
#include <map>

#include "ros/ros.h"

#include <sensor_msgs/Joy.h>


class JoystickInterface
{
public:
    JoystickInterface() {};

    // Initialize interface
    void initialize(ros::NodeHandle *nh);

    // Get joystick values
    std::map<std::string, double> get_joystick_values(void) {
        return m_joy_values;
    }

private:

    /******************* Joystick Topic Subscriber *****************/
    // Joystick topic name
    std::string m_joy_topic_name = "joy";
    
    // Joystick subscriber
    ros::Subscriber m_joy_sub; 

    // Joystick callback function
    void joystick_callback(const sensor_msgs::JoyConstPtr& msg);

    // Check if we have subscribed to the topic
    bool m_joy_sub_flag = false;
    
private:

    std::map<std::string, double> m_joy_values = 
    {
        {"arrows_horizontal", 0.0},
        {"arrows_vertical", 0.0},
        {"left_joy_horizontal", 0.0},
        {"left_joy_vertical", 0.0},
        {"right_joy_horizontal", 0.0},
        {"right_joy_vertical", 0.0},
        {"button_A", 0.0},
        {"button_B", 0.0},
        {"button_X", 0.0},
        {"button_Y", 0.0},
        {"button_R1", 0.0},
        {"button_R2", 0.0},
        {"button_L1", 0.0},
        {"button_L2", 0.0},
        {"button_START", 0.0},
        {"button_BACK", 0.0},
        {"button_LEFT_JOY", 0.0},
        {"button_RIGHT_JOY", 0.0},
    };
};




