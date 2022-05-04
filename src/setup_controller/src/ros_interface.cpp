#include "../include/ros_interface.h"

// Initialize ROS interface
void ROSInterface::initialize(UserInterface* gui,
    AnimationInterface* anim_interface, StateHandler* state_handler,
    ros::NodeHandle *nh)
{
   // Set gui pointer to member variable 
    m_gui = gui;

    // Set animation interface pointer to member variable
    m_anim_interface = anim_interface;

    // Set state handler point to member variable
    m_state_handler = state_handler;

    // Get control panel window ptr
    m_cp_window_ptr = m_gui->get_control_panel_window_ptr();

    // Initialize joystick interface
    m_joystick_interface.initialize(nh);

    // Make a lwa4p interface
    m_lwa4p_interface = std::make_shared<Lwa4pInterface>(nh, m_cp_window_ptr, 
        &m_joystick_interface);

    // Set initial values for lwa4p desired joint angles
    std::vector<double> lwa4p_initial_joint_angles = m_lwa4p_interface->get_joint_angles();
    m_cp_window_ptr->set_lwa4p_initial_desired_joint_angles(lwa4p_initial_joint_angles);
    
    // Set initial values for lwa4p desired ee pose
    std::vector<double> lwa4p_initial_ee_pose = m_lwa4p_interface->get_ee_pose();
    m_cp_window_ptr->set_lwa4p_initial_desired_ee_pose(lwa4p_initial_ee_pose);

    // Initialize state
    m_state = m_state_handler->get_state();

    // Make a parameters interface
    m_params_interface = std::make_shared<ParametersInterface>(nh, m_cp_window_ptr);

    // Make a vision interface
    m_vision_interface = std::make_shared<VisionInterface>(nh, 
        m_gui->get_camera1_window_ptr(), m_gui->get_camera2_window_ptr()) ;
}

// Update ROS interface
void ROSInterface::update(double real_time)
{
    // Get current state
    m_state = m_state_handler->get_state();

    // Update lwa4p arm    
    update_lwa4p();
}

// Update lwa4p arm
void ROSInterface::update_lwa4p(void)
{
    /********************* Update control panel *************************/
    // Get joint angles
    std::vector<double> joint_angles = m_lwa4p_interface->get_joint_angles();

    // Get ee pose
    std::vector<double> ee_pose = m_lwa4p_interface->get_ee_pose();

    // Set joint angles in control panel
    m_cp_window_ptr->set_lwa4p_joint_angles(joint_angles);

    // Get and set end-effector pose
    m_cp_window_ptr->set_lwa4p_ee_pose(ee_pose);

    // Update lwa4p interface    
    m_lwa4p_interface->update(m_state);

    /************************* Update Animation *************************/
    // Calculate forward kinematics for each joint
    m_lwa4p_fk.calculate(joint_angles);

    // Get joints positions 
    std::vector<glm::vec3> links_positions = m_lwa4p_fk.get_links_positions();

    // Get joints orientations
    std::vector<glm::vec3> links_orientations = m_lwa4p_fk.get_links_orientations();
    
    // Set lwa4p links positions
    m_anim_interface->set_lwa4p_links_positions(m_lwa4p_fk.get_links_positions());

    // Set lwa4p links orientations
    m_anim_interface->set_lwa4p_links_orientations(m_lwa4p_fk.get_links_orientations());
}