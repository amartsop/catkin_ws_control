#include "../include/application_interface.h"

// Initialize application
void ApplicationInterface::initialize(ros::NodeHandle *nh)
{
    // Initialize display 
    m_display = std::make_shared<Display>(m_window_title.c_str());

    // Initialize experiment window    
    m_experiment_window.initialize();

    // Initialize animation 
    m_animation.initialize(&m_anim_interface, m_display);

    // Initialzie user interface
    m_gui.initialize(m_display);
    
    // Initialize state handler
    m_state_handler.initialize(&m_gui, nh);

    // Initialize ros interface
    m_ros_interface.initialize(&m_gui, &m_anim_interface, &m_state_handler, nh);
}

// Update application
void ApplicationInterface::update(double real_time, double deltaTime)
{
    // Poll events
    glfwPollEvents();

    // Update user interface
    m_gui.update();
    
    // Update ROS interface
    m_ros_interface.update(real_time);

    // Update state handler
    m_state_handler.update();

    // Update experiment scene    
    update_experiment_scene(real_time, deltaTime);

    // Render camera 1 window    
    m_gui.render_camera1_window();

    // Update camera 2
    m_gui.render_camera2_window();

    // Update control panel
    m_gui.render_control_panel();

    // Render gui
    m_gui.render();

    // Clear experiment window
    m_experiment_window.clear();

    // Clear display background
    m_display->clear(m_background_color[0], m_background_color[1],
        m_background_color[2]);

    // Update display
    m_display->update();
}

// Update experiment scene
void ApplicationInterface::update_experiment_scene(double real_time,
    double deltaTime)
{
    // Update animation scene    
    m_animation.update(real_time, deltaTime);

    // Frame buffer update
    m_experiment_window.update_frame_buffer();

    // Render experiment window
    m_gui.render_experiment_window(m_experiment_window.get_texture_color_buffer());
}
