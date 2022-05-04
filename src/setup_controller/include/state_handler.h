#pragma once 

#include <iostream>
#include <memory>
#include "ros/ros.h"

#include <user_interface.h>
#include <control_panel_window.h>
#include <smch_msgs/State.h>

/**
 * @brief This class handles the states of the setup. The states are the 
 * following.
 * State 0: Schunk Lwa4p manual joint control mode from GUI.
 * State 1: Schunk Lwa4p manual end_effector control mode from GUI.
 * State 2: Schunk Lwa4p manual joint_control control mode from joystick.
 * State 3: Schunk Lwa4p manual end_effector control mode from joystick.
 * State 4: Schunk Lwa4p manual tool control mode from joystick.
 * State 5: Schunk Lwa4p homing routine (home position).
 * State 6: Schunk Lwa4p homing routine (zero position).
 * State 7: Needle experiment initialization.
 * State 8: Needle experiment execution.
 */
class StateHandler
{
public:
    // Constructor
    StateHandler(){};

    // Initialize
    void initialize(UserInterface* gui, ros::NodeHandle *nh);

    // Update
    void update(void);

    // Get state
    int get_state(void) { return m_state_id; }

private:

    // Pointer to user interface
    UserInterface* m_gui;

    // Control panel window ptr
    std::shared_ptr<ControlPanelWindow> m_cp_window_ptr;

private:

    // The current state of the setup
    int m_state_id = 0; 

private:
    /******************** System State Publisher ***********************/
    // Publisher handle for ee state
    ros::Publisher m_system_state_pub;

    // Topic name 
    const std::string m_system_state_topic_name = "system_state";

    // Publishing interval
    const double m_system_state_pub_interval = 1.0 / 130.0;

    // Publishing timer
    ros::Timer m_system_state_pub_timer;

    // Publish system state
    void publish_system_state(const ros::TimerEvent& event);
};