#include "../include/parameters_interface.h"

ParametersInterface::ParametersInterface(ros::NodeHandle *nh,
    std::shared_ptr<ControlPanelWindow> cp_window)
{
    // Store control panel window ptr to member variable
    m_cp_window_ptr = cp_window;

    /****************** Nve parameters publisher *****************/
    // Nve parameters publishing timer
    m_nve_params_timer = nh->createTimer(ros::Duration(m_nve_params_pub_interval), 
        &ParametersInterface::publish_nve_parameters, this);

    // Publisher handle for the nve parameters
    m_nve_params_pub = nh->advertise<parameters_msgs::NeedleVibrationExperiment>(
        m_nve_params_topic_name, 1000);

    /****************** Vision parameters publisher *****************/
    // Vision parameters publishing timer
    m_vision_params_timer = nh->createTimer(
        ros::Duration(m_vision_params_pub_interval), 
        &ParametersInterface::publish_vision_parameters, this);

    // Publisher handle for vision parameters
    m_vision_params_pub = nh->advertise<parameters_msgs::Vision>(
        m_vision_params_topic_name, 1000);
}
    
// Nve parameters callback function
void ParametersInterface::publish_nve_parameters(const ros::TimerEvent& event)
{
    // Get parameters from control panel
    auto translation_params = m_cp_window_ptr->get_nve_translation_params();
    auto rotation_params = m_cp_window_ptr->get_nve_rotation_params();

    auto rtct_ft_ft =
        m_cp_window_ptr->get_axia_com_wrt_measurement_reference_frame();

    // Initialize parameters msg
    parameters_msgs::NeedleVibrationExperiment nve_msg;
    
    // Set translation amplitude
    nve_msg.translation_amplitude.x = translation_params.at(0).at(0);
    nve_msg.translation_amplitude.y = translation_params.at(0).at(1);
    nve_msg.translation_amplitude.z = translation_params.at(0).at(2);
    
    // Set translation frequency
    nve_msg.translation_frequency.x = translation_params.at(1).at(0);
    nve_msg.translation_frequency.y = translation_params.at(1).at(1);
    nve_msg.translation_frequency.z = translation_params.at(1).at(2);

    // Set translation phase
    nve_msg.translation_phase.x = translation_params.at(2).at(0);
    nve_msg.translation_phase.y = translation_params.at(2).at(1);
    nve_msg.translation_phase.z = translation_params.at(2).at(2);

    // Set rotation amplitude
    nve_msg.rotation_amplitude.x = rotation_params.at(0).at(0);
    nve_msg.rotation_amplitude.y = rotation_params.at(0).at(1);
    nve_msg.rotation_amplitude.z = rotation_params.at(0).at(2);
    
    // Set rotation frequency
    nve_msg.rotation_frequency.x = rotation_params.at(1).at(0);
    nve_msg.rotation_frequency.y = rotation_params.at(1).at(1);
    nve_msg.rotation_frequency.z = rotation_params.at(1).at(2);

    // Set rotation phase
    nve_msg.rotation_phase.x = rotation_params.at(2).at(0);
    nve_msg.rotation_phase.y = rotation_params.at(2).at(1);
    nve_msg.rotation_phase.z = rotation_params.at(2).at(2);

    // Set sensor com with repsect to the reference measurement frame
    nve_msg.rtct_ft_ft.x = rtct_ft_ft.at(0);
    nve_msg.rtct_ft_ft.y = rtct_ft_ft.at(1);
    nve_msg.rtct_ft_ft.z = rtct_ft_ft.at(2);

    m_nve_params_pub.publish(nve_msg);
}

// Vision parameters callback function
void ParametersInterface::publish_vision_parameters(const ros::TimerEvent& event)
{
    // Initialize vision message    
    parameters_msgs::Vision vision_msg;

    // Set parameters for camera 1
    vision_msg.camera1_stream_status = m_cp_window_ptr->get_camera1_stream_status();
    vision_msg.camera1_cp_status = m_cp_window_ptr->get_camera1_cp_status();
    vision_msg.camera1_bp_status = m_cp_window_ptr->get_camera1_bp_status();
    vision_msg.camera1_type = m_cp_window_ptr->get_camera1_image_type();
    vision_msg.camera1_binary_threshold_number =
        m_cp_window_ptr->get_camera1_binary_threshold_value();
    
    std::vector<double> rok1_F_F = m_cp_window_ptr->get_camera1_inertial_position();
    vision_msg.rok1_F_F.x = rok1_F_F[0];
    vision_msg.rok1_F_F.y = rok1_F_F[1];
    vision_msg.rok1_F_F.z = rok1_F_F[2];

    std::vector<double> camera1_euler = m_cp_window_ptr->get_camera1_orientation();
    vision_msg.camera1_euler.x = camera1_euler[0];
    vision_msg.camera1_euler.y = camera1_euler[1];
    vision_msg.camera1_euler.z = camera1_euler[2];

    // Set parameters for camera 2
    vision_msg.camera2_stream_status = m_cp_window_ptr->get_camera2_stream_status();
    vision_msg.camera2_cp_status = m_cp_window_ptr->get_camera2_cp_status();
    vision_msg.camera2_bp_status = m_cp_window_ptr->get_camera2_bp_status();
    vision_msg.camera2_type = m_cp_window_ptr->get_camera2_image_type();
    vision_msg.camera2_binary_threshold_number =
        m_cp_window_ptr->get_camera2_binary_threshold_value();
    
    std::vector<double> rok2_F_F = m_cp_window_ptr->get_camera2_inertial_position();
    vision_msg.rok2_F_F.x = rok2_F_F[0];
    vision_msg.rok2_F_F.y = rok2_F_F[1];
    vision_msg.rok2_F_F.z = rok2_F_F[2];

    std::vector<double> camera2_euler = m_cp_window_ptr->get_camera2_orientation();
    vision_msg.camera2_euler.x = camera2_euler[0];
    vision_msg.camera2_euler.y = camera2_euler[1];
    vision_msg.camera2_euler.z = camera2_euler[2];

    // Publish message
    m_vision_params_pub.publish(vision_msg);
}