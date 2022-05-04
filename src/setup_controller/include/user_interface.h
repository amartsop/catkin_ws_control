#pragma once

#include <iostream>
#include <vector>
#include <memory>

#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

#include "display.h"
#include <control_panel_window.h>
#include <camera_window.h>

class UserInterface
{
    public:
        UserInterface(void){};
        
        /// Initialize 
        void initialize(std::shared_ptr<Display> display);
        
        // Update gui
        void update(void);

        // Render gui
        void render(void);

        // Render experiment window
        void render_experiment_window(unsigned int texture_color_buffer);

        // Render camera 1 window
        void render_camera1_window(void);

        // Render camera 2 window
        void render_camera2_window(void);

        // Render control panel
        void render_control_panel(void);

        // Destructor
        ~UserInterface();

    public:

        // Get control panel window ptr
        std::shared_ptr<ControlPanelWindow> get_control_panel_window_ptr(void)
        { return m_cp_window_ptr; }
    
        // Get camera 1 window ptr
        std::shared_ptr<CameraWindow> get_camera1_window_ptr(void)
        { return m_camera1_window_ptr; }

        // Get camera 2 window ptr
        std::shared_ptr<CameraWindow> get_camera2_window_ptr(void)
        { return m_camera2_window_ptr; }

    private:
        
        // Display pointer
        std::shared_ptr<Display> m_display;

        // Display width and height
        int m_display_width, m_display_height;
    
        // Set ImGui Style
        void set_ImGui_style(void);
    
    private:

        // Experiment window size to screen size ratio
        float m_ew_ratio = 0.7;
        
        // Experiment window width and height
        ImVec2 m_ew_dims;
        
        // Experiment window name 
        std::string m_ew_name = "Experiment Setup";

        // Set experiment window position
        ImVec2 m_ew_pos = ImVec2(0.0f, 0.0f);

    private:

        // Camera 1 window size to screen size ratio
        float m_cam1_ratio = 0.3;
        
        // Camera 1 width and height
        ImVec2 m_cam1_dims;
        
        // Camera 1 window name 
        std::string m_cam1_name = "Camera 1";

        // Set camera 1 position
        ImVec2 m_cam1_pos = ImVec2(0.0f, 0.0f);

        // Camera 1 window pointer
        std::shared_ptr<CameraWindow> m_camera1_window_ptr;

    private:

        // Camera 2 window size to screen size ratio
        float m_cam2_ratio = 0.3;
        
        // Camera 2 window width and height
        ImVec2 m_cam2_dims;
        
        // Camera 2 window name 
        std::string m_cam2_name = "Camera 2";

        // Set camera 2 position
        ImVec2 m_cam2_pos = ImVec2(0.0f, 0.0f);

        // Camera 2 window pointer
        std::shared_ptr<CameraWindow> m_camera2_window_ptr;

    private:

        // Control panel size to screen size ratio
        float m_cp_ratio = 0.3;

        // Control panel width and height
        ImVec2 m_cp_dims;
        
        // Control panel name 
        std::string m_cp_name = "Control panel";

        // Set control panel position
        ImVec2 m_cp_pos = ImVec2(0.0f, 0.0f);

        // Control panel window
        std::shared_ptr<ControlPanelWindow> m_cp_window_ptr;
};