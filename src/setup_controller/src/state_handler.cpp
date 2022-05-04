#include "../include/state_handler.h"

// Constructor
void StateHandler::initialize(UserInterface* gui, ros::NodeHandle *nh)
{
    // System state publisher 
    m_system_state_pub = nh->advertise<smch_msgs::State>(m_system_state_topic_name,
        1000);

   // Set gui pointer to member variable 
    m_gui = gui;

    // Get control panel window ptr
    m_cp_window_ptr = gui->get_control_panel_window_ptr();

    try
    {
        // Publishing timer
        m_system_state_pub_timer = nh->createTimer(ros::Duration(
            m_system_state_pub_interval),
            &StateHandler::publish_system_state, this);
    }
    catch(std::runtime_error& ex) {
        ROS_ERROR("Exception state handler: [%s]", ex.what());
    }
}

// Update state handler
void StateHandler::update(void)
{
    // Get the current operational mode
    int operational_mode = m_cp_window_ptr->get_operational_mode();

    // Get lwa4p manual control mode
    int lwa4p_manual_control_mode = m_cp_window_ptr->get_lwa4p_manual_control_mode();

    // Schunk Lwa4p
    if (operational_mode == 0)
    {
        if (lwa4p_manual_control_mode == 0) { m_state_id = 0; }
        else if (lwa4p_manual_control_mode == 1) { m_state_id = 1; }
        else if (lwa4p_manual_control_mode == 2) { m_state_id = 2; }
        else if (lwa4p_manual_control_mode == 3) { m_state_id = 3; }
        else { m_state_id = 4; }
    }
    // Lwa4p homing
    else if (operational_mode == 1)
    {
        if (m_cp_window_ptr->get_lwa4p_go_home_status())
        {
            m_state_id = 5;
        }
        if (m_cp_window_ptr->get_lwa4p_go_zero_status())
        {
            m_state_id = 6;
        }
    }
    // Needle experiment
    else if (operational_mode == 2)
    {
        if (m_cp_window_ptr->get_needle_experiment_initialization_status())    
        {
            m_state_id = 7;
        }
        if (m_cp_window_ptr->get_needle_experiment_execution_status())
        {
            m_state_id = 8;
        }
    }

    // to be continued...
}

// Publish system state
void StateHandler::publish_system_state(const ros::TimerEvent& event)
{
    // Initialize message
    smch_msgs::State system_state;
    
    // Set time
    system_state.header.stamp.sec = ros::Time::now().sec;
    system_state.header.stamp.nsec = ros::Time::now().nsec;        
    
    // Set state
    system_state.state.data = m_state_id;

    // Advertise state
    m_system_state_pub.publish(system_state);
}