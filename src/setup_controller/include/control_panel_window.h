#pragma once

#include <iostream>
#include <vector>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

class ControlPanelWindow
{

public:
    ControlPanelWindow(/* args */){};

    // Render window
    void render(void);

public:

    // Get operational mode
    int get_operational_mode(void) { return m_operational_mode; }

    // Get lwa4p manual control mode
    int get_lwa4p_manual_control_mode(void) { return m_lwa4p_manual_control_mode; }

    // Get lwa4p go home status
    bool get_lwa4p_go_home_status(void) { return m_lwa4p_go_home; }

    // Get lwa4p go zero status
    bool get_lwa4p_go_zero_status(void) { return m_lwa4p_go_zero; }

    // Get needle experiment initialization status
    bool get_needle_experiment_initialization_status(void) 
    { return m_needle_experiment_init; }

    // Get needle experiment execution status
    bool get_needle_experiment_execution_status(void)
    { return m_needle_experiment_execute; }

public:

    // Set lwa4p joint angles
    void set_lwa4p_joint_angles(std::vector<double>& joint_angles);

    // Set lwa4p ee pose    
    void set_lwa4p_ee_pose(std::vector<double>& ee_pose);
    
    // Set initial desired lwa4p joints angles
    void set_lwa4p_initial_desired_joint_angles(std::vector<double>& joint_angles);
    
    // Set initial desired lwa4p ee pose
    void set_lwa4p_initial_desired_ee_pose(std::vector<double>& ee_pose);
    
    // Get desired lwa4p joint angles
    std::vector<double> get_lwa4p_desired_joint_angles(void);

    // Get desired lwa4p ee pose
    std::vector<double> get_lwa4p_desired_ee_pose(void);
    
public:

    // Get sensor's centre of mass with respect to the measurement origin frame
    std::vector<double> get_axia_com_wrt_measurement_reference_frame(void);
    
public:

    // Get needle vibration experiment translation parameters
    std::vector<std::vector<double>> get_nve_translation_params(void);

    // Get needle vibration experiment rotation parameters
    std::vector<std::vector<double>> get_nve_rotation_params(void);

public:

    // Get camera 1 stream status
    bool get_camera1_stream_status(void);

    // Get camera 2 stream status
    bool get_camera2_stream_status(void);

    // Get camera 1 corresponding points status
    bool get_camera1_cp_status(void);

    // Get camera 2 stream status
    bool get_camera2_cp_status(void);

    // Get camera 1 background point status
    bool get_camera1_bp_status(void);

    // Get camera 2 stream status
    bool get_camera2_bp_status(void);

    // Get camera 1 image type
    int get_camera1_image_type(void);

    // Get camera 2 image type
    int get_camera2_image_type(void);

    // Get camera 1 binary threshold value
    double get_camera1_binary_threshold_value(void);

    // Get camera 2 binary threshold value
    double get_camera2_binary_threshold_value(void);

    // Get camera 1 inertial position
    std::vector<double> get_camera1_inertial_position(void);

    // Get camera 2 inertial position
    std::vector<double> get_camera2_inertial_position(void);

    // Get camera 1 orientation position
    std::vector<double> get_camera1_orientation(void);

    // Get camera 2 orientation position
    std::vector<double> get_camera2_orientation(void);

private:
    // Home tab
    void home_tab(void);

    // Setup elements status
    void setup_elements_status(void);

    // Lwa4p Status Flag
    bool m_lwa4p_status = true;

    // Cyton status
    bool m_cyton_status = false;

    // Axia status
    bool m_axia_status = false;

    // Camera 1 status
    bool m_camera1_status = true;

    // Camera 2 status
    bool m_camera2_status = true;

    // Status strings 
    std::vector<std::string> m_status_string = {"Not Ready", "Ready"};

    // Status color 
    std::vector<int> m_red_color = {255, 0, 0, 255};
    std::vector<int> m_green_color = {0, 255, 0, 255};
    std::vector<std::vector<int>> m_status_color = {m_red_color, m_green_color};

    // Tables names 
    std::vector<std::string> m_home_table_headers = {"System State",
        "Operational Mode", "Configuration"};

    
    // List of operational modes
    std::vector<std::string> m_operational_modes_names = {
        "Schunk Lwa4p Manual Control", "Schunk Lwa4p Homing",
        "Needle Experiment", "Automatic Needle Insertion"};

private:

    // Operational mode
    int m_operational_mode = 0;

private:

    // Lwa4p manual control modes names
    std::vector<std::string> m_lwa4p_manual_control_modes_names = {
        "Joint Control (GUI)", 
        "End-Effector Control (GUI)", 
        "Joint Control (Joystick)", 
        "End-Effector Control (Joystick)",
        "Tool Frame Control (Joystick)"};

    // Lwa4p manual control mode
    int m_lwa4p_manual_control_mode = 0;

private:
    
    // Lwa4p tab 
    void lwa4p_tab(void);

    // Lwa4p manual control gui tab    
    void lwa4p_manual_control_gui_tab(void);

    // Tables names 
    std::vector<std::string> m_lwa4p_table_headers = {
        "Current Joint Configuration (deg)",
        "Current End-Effector Pose",
        "Desired Configuration"};

    // Joint names
    std::vector<std::string> m_lwa4p_joint_names = {"Joint 1 (rad)",
        "Joint 2 (rad)", "Joint 3 (rad)", "Joint 4 (rad) ", "Joint 5 (rad)",
        "Joint 6 (rad)"};

    // End-effector names    
    std::vector<std::string> m_lwa4p_ee_names = {"x (m)", "y (m)", "z (m)", 
        "Roll (rad)", "Pitch (rad)", "Yaw (rad)"};
    
    // Lwa4p joints num
    const int m_lwa4p_joints_num = 6;

    // Lwa4p ee pose num
    const int m_lwa4p_ee_pose_num = 6;

    // Lwa4p joint values
    float m_lwa4p_joint_angles[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Lwa4p ee pose values
    float m_lwa4p_ee_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Lwa4p desired joint values
    float m_lwa4p_des_joint_angles[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Lwa4p desired ee pose
    float m_lwa4p_des_ee_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Lwa4p joint state step
    float m_lwa4p_joint_state_step = 0.01f;

    // Lwa4p ee state step
    float m_lwa4p_ee_state_step[6] = {0.005f, 0.005f, 0.005f, 0.01f, 0.01f, 0.01f};

private:
    // Lw4p go home button
    bool m_lwa4p_go_home = false;

    // Lw4p go zero button
    bool m_lwa4p_go_zero = false;

private:
    // Initialize needle experiment button
    bool m_needle_experiment_init = false;

    // Execute needle experiment button
    bool m_needle_experiment_execute = false;

    // Needle experiment scalar step
    float m_needle_experiment_scalar_step = 0.01f;

    // Headers translation amplitude
    std::vector<std::string> m_ne_ta = {"Amplitude x", "Amplitude y", "Amplitude z"};

    // Headers translation frequency
    std::vector<std::string> m_ne_tf = {"Frequency x", "Frequency y", "Frequency z"};

    // Headers translation phase
    std::vector<std::string> m_ne_tp = {"Phase x", "Phase y", "Phase z"};

    // Translation amplitude (m), Frequency (Hz) and Phase (rad)
    float m_translation_a[3] = {0.0f, 0.0f, 0.0f};
    float m_translation_f[3] = {0.5f, 0.5f, 0.5f};
    float m_translation_p[3] = {0.0f, 0.0f, 0.0f};

    // Headers rotation amplitude
    std::vector<std::string> m_ne_ra = {"Amplitude roll", "Amplitude pitch", "Amplitude yaw"};

    // Headers rotation frequency
    std::vector<std::string> m_ne_rf = {"Frequency roll", "Frequency pitch", "Frequency yaw"};

    // Headers rotatio phase
    std::vector<std::string> m_ne_rp = {"Phase roll", "Phase pitch", "Phase yaw"};

    // Rotation amplitude (m), Frequency (Hz) and Phase (rad)
    float m_rotation_a[3] = {0.0f, 0.0f, 0.0f};
    float m_rotation_f[3] = {0.5f, 0.5f, 0.5f};
    float m_rotation_p[3] = {0.0f, 0.0f, 0.0f};

private: 
    // Axia tab
    void axia_tab(void);

    // Sensor's centre of mass with respect to the measurement origin frame
    float m_rtct_ft_ft[3] = {0.0f, 0.0f, 0.0f};

    // Axia sensor com slider step
    float m_axia_com_slider_step = 0.01;

private:
    // Vision system tab
    void vision_system_tab(void);

    // Tables names for vision system
    std::vector<std::string> m_vs_table_headers = { "Camera 1", "Camera 2"};
    
    // Enable stream checkbox
    bool m_vs_enable_stream[2] = {true, true};
    
    // Image types
    std::vector<std::string> m_vs_image_types_names = {"Grayscale", "Binary", 
    "Binary ROI"};

    /**
     * @brief Image types
     *  Image type 0: Grayscale image.
     *  Image type 1: Binary Image
     *  Image type 1: Binary Image ROI (only region of interest)
     */ 
    std::vector<int> m_vs_image_type = {0, 0};

    // Binary image threshold value
    std::vector<float> m_binary_image_thresh_val = {0.0, 0.0};
    
    // Binary image threshold value step
    float m_binary_image_thrash_val_step = 1.0;

    // Camera position (rok_F_F)
    std::vector<std::vector<float>> m_camera_pos = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

    // Camera orientation    
    std::vector<std::vector<float>> m_camera_euler = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

    // Camera position/orientation slider step
    float m_camera_pos_rot_slider_step = 0.01;

    // Enable corresponding points checkbox
    bool m_vs_enable_cp[2] = {false, false};

    // Enable base point checkbox
    bool m_vs_enable_bp[2] = {false, false};

};
    