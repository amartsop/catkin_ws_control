#pragma once

#include <iostream>
#include <memory>
#include "ros/ros.h"

#include "user_interface.h"
#include "animation_interface.h"
#include "control_panel_window.h"
#include "state_handler.h"
#include "lwa4p_forward_kinematics.h"
#include "lwa4p_interface.h"
#include "parameters_interface.h"
#include "vision_interface.h"

class ROSInterface
{
public:
    ROSInterface(){};

    // Initialize 
    void initialize(UserInterface* gui, AnimationInterface* anim_interface,
        StateHandler* state_handler, ros::NodeHandle *nh);

    // Update
    void update(double real_time);

private:
    // Pointer to user interface
    UserInterface* m_gui;

    // Pointer to animation interface
    AnimationInterface* m_anim_interface;

    // Pointer to state handler
    StateHandler* m_state_handler;

    // Control panel window ptr
    std::shared_ptr<ControlPanelWindow> m_cp_window_ptr;

    /// Joystick interface
    JoystickInterface m_joystick_interface;

    // Current state
    int m_state;

private:

    // Update lwa4p arm
    void update_lwa4p(void);

    // Lwa4p handler pointer
    std::shared_ptr<Lwa4pInterface> m_lwa4p_interface;

    // Lwa4p forward kinematics handle 
    Lwa4pForwardKinematics m_lwa4p_fk;

    std::vector<double> m_joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

private:
    // Parameters interface
    std::shared_ptr<ParametersInterface> m_params_interface;

    // Vision interface
    std::shared_ptr<VisionInterface> m_vision_interface;
};

