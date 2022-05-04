#include "../include/joystick_interface.h"

void JoystickInterface::initialize(ros::NodeHandle *nh)
{
    /*********************** Joystick Topic Subscriber *****************/
    // Subscribe to joystick topic
    m_joy_sub = nh->subscribe(m_joy_topic_name, 1000,
        &JoystickInterface::joystick_callback, this);
}

// Joystick callback function
void JoystickInterface::joystick_callback(const sensor_msgs::JoyConstPtr& msg)
{
    if (msg->axes.size() > 0)
    {
        // Arrows horizontal 
        m_joy_values["arrows_horizontal"] = (double) msg->axes.at(0);

        // Arrows vertical 
        m_joy_values["arrows_vertical"] = (double) msg->axes.at(1);

        // Left joystick horizontal
        m_joy_values["left_joy_horizontal"] = (double) msg->axes.at(4);

        // Left joystick vertical
        m_joy_values["left_joy_vertical"] = (double) msg->axes.at(5);

        // Right joystick horizontal
        m_joy_values["right_joy_horizontal"] = (double) msg->axes.at(2);

        // Right joystick vertical
        m_joy_values["right_joy_vertical"] = (double) msg->axes.at(3);
    }
    
    if (msg->buttons.size() > 0)
    {
        // Button A
        m_joy_values["button_A"] = (double) msg->buttons.at(1);

        // Button B
        m_joy_values["button_B"] = (double) msg->buttons.at(2);

        // Button X
        m_joy_values["button_X"] = (double) msg->buttons.at(0);

        // Button Y
        m_joy_values["button_Y"] = (double) msg->buttons.at(3);

        // Button R1
        m_joy_values["button_R1"] = (double) msg->buttons.at(5);

        // Button R2
        m_joy_values["button_R2"] = (double) msg->buttons.at(4);

        // Button L1
        m_joy_values["button_L1"] = (double) msg->buttons.at(7);

        // Button L2
        m_joy_values["button_L2"] = (double) msg->buttons.at(6);

        // Button START
        m_joy_values["button_START"] = (double) msg->buttons.at(9);

        // Button BACK
        m_joy_values["button_BACK"] = (double) msg->buttons.at(8);

        // Button LEFT JOYSTICK
        m_joy_values["button_LEFT_JOY"] = (double) msg->buttons.at(10);

        // Button RIGHT JOYSTICK
        m_joy_values["button_RIGHT_JOY"] = (double) msg->buttons.at(11);
    }
}