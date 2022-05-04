#pragma once
#include <iostream>
#include <memory>


#include <filesystem.h>
#include <display.h>
#include <callback_setter.h>
#include <callback_handler.h>

#include <animation_interface.h>
#include <experiment_window.h>
#include <camera_window.h>
#include <user_interface.h>
#include <scene.hpp>
#include <ros_interface.h>
#include <state_handler.h>
#include <joystick_interface.h>

class ApplicationInterface
{
public:
    ApplicationInterface(/* args */){};

    // Initialize application interface
    void initialize(ros::NodeHandle *nh);

    // Update application
    void update(double real_time, double deltaTime);

    // Check if animation is done
    bool is_done(void) { return m_display->is_closed(); }

private:
    
    // Animation interface
    AnimationInterface m_anim_interface;

    // Define animation
    Scene<AnimationInterface> m_animation;

    // Window properties
    const std::string m_window_title = "Transperineal Prostate Biopsy";
    const float m_background_color[3] = {0.82f, 0.8f, 0.9f}; // RGB
        
    // Display object ptr
    std::shared_ptr<Display> m_display;

private:
    
    /// User interface
    UserInterface m_gui;

    // System state
    StateHandler m_state_handler;

    /// ROS interface
    ROSInterface m_ros_interface;

    /// Experiment window
    ExperimentWindow m_experiment_window;

private:

    /// Update animation window
    void update_experiment_scene(double real_time, double delta_time);
};
