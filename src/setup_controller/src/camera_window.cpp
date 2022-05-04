#include "../include/camera_window.h"

CameraWindow::CameraWindow(/* args */)
{
    
}

void CameraWindow::render_frames(void)
{
    // Pending start routine
    if (m_pending_start)
    {
        // Set initial time
        m_time_init = ros::Time::now();
    
        // Update pending start flag
        m_pending_start = false;
    }
    
    // Render every rendering time secs
    if ( (ros::Time::now() - m_time_init).toSec() >= m_rendering_interval)
    {
        // Set frame data pointer
        m_frame_data_ptr = m_frame.ptr();
        
        // Set width and height
        m_frame_width = m_frame.cols;
        m_frame_height = m_frame.rows;

        glGenTextures(1, &m_texture_id);
        glBindTexture(GL_TEXTURE_2D, m_texture_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_frame_width, m_frame_height,
            0, GL_LUMINANCE, GL_UNSIGNED_BYTE, m_frame_data_ptr);

        // Update initial time
        m_time_init = ros::Time::now();
    }
    
    m_canvas_size = ImVec2(m_frame_width, m_frame_height);
}


// Set frames
void CameraWindow::set_frames(cv::Mat& frame)
{
    // Resize image
    cv::resize(frame, frame, cv::Size(576, 365), 0.0, 0.0, cv::INTER_AREA);
    
    // Clone frame
    m_frame = frame.clone();
}