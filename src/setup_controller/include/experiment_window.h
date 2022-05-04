#pragma once

#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

class ExperimentWindow
{

public:

    ExperimentWindow(/* args */) {};

    // Initialize experiment window
    void initialize(void);

    // Update frame buffer
    void update_frame_buffer(void);

    // Clear window
    void clear(void);

    // Get texture color buffer
    unsigned int get_texture_color_buffer(void) { return m_texture_color_buffer; }

private:

    // Frame buffer 
    unsigned int m_fbo;

    // Texture color buffer
    unsigned int m_texture_color_buffer;

    // Frame buffer width and height
    int m_fbo_width = 1920;
    int m_fbo_height = 1080;
};