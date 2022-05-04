#pragma once
#include "ros/ros.h"

#include <iostream>
#include <memory>

#include <opencv4/opencv2/opencv.hpp>

#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>


class CameraWindow
{

public:
    // Constructor
    CameraWindow(void);
    
    // Render frames
    void render_frames(void);

    // Set frames
    void set_frames(cv::Mat& frame);

    // Get texture id
    unsigned int get_texture_id(void) { return m_texture_id; }

    // Get canvas size
    ImVec2 get_canvas_size(void) { return m_canvas_size; }

private:

    // Texture id
    unsigned int m_texture_id;

    // Frame height and width
    int m_frame_height = 1080;
    int m_frame_width = 1440;

    // Frame
    cv::Mat m_frame;

    // Frame data pointer
    unsigned char *m_frame_data_ptr;

    // Canvas size
    ImVec2 m_canvas_size;

    // Init time
    ros::Time m_time_init;

    // Rendering frequency (FPS)
    double m_rendering_freq = 25.0;
    double m_rendering_interval = 1.0 / m_rendering_freq;
    
    bool m_pending_start = true;
};