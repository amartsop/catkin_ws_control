#include "../include/control_panel_window.h"

void ControlPanelWindow::render(void)
{
    if (ImGui::BeginTabBar("MyTabBar", ImGuiTabBarFlags_None))
    {
        if (ImGui::BeginTabItem("Setup"))
        {
            // Render home tab
            home_tab();
            
            // End tab item
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Schunk Lwa4p"))
        {
            // Render lwa4p tab 
            lwa4p_tab();

            // End tab item
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Vision System"))
        {
            // Render vision system tab 
            vision_system_tab();
            
            // End tab item
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Gamma Cyton"))
        {
            
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Axia FT Sensor"))
        {
            // Render axia tab
            axia_tab();

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Dynamixel MX-12W"))
        {

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Needle"))
        {

            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }
    ImGui::Separator();
}

// Home tab    
void ControlPanelWindow::home_tab(void)
{
    // Table flags
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter |
        ImGuiTableFlags_BordersV  | ImGuiTableFlags_NoHostExtendX;
        
    // Number of rows and columns
    int rows_num = 1;
    int cols_num = 3; 

    // Generate table
    if (ImGui::BeginTable("home_table", cols_num, flags))
    {
        // Generate headers
        for (size_t i = 0; i < m_home_table_headers.size(); i++)
        {
            ImGui::TableSetupColumn(m_home_table_headers.at(i).c_str(),
                ImGuiTableColumnFlags_WidthFixed);
        }

        // Generate table rows and cols
        ImGui::TableHeadersRow();

        for (int row = 0; row < rows_num; row++)
        {
            ImGui::TableNextRow();

            for (int column = 0; column < cols_num; column++)
            {
                ImGui::TableSetColumnIndex(column);

                // Column "System State"
                if (column == 0)
                {
                    // Setup element status
                    setup_elements_status();
                }

                // Column "Operational Mode"
                if (column == 1)
                {
                    for (int i = 0; i < m_operational_modes_names.size(); i++)
                    {
                        if (ImGui::Selectable(m_operational_modes_names.at(i).c_str(),
                            m_operational_mode == i))
                        {
                            m_operational_mode = i;
                        }
                    }
                }
                 
                // Column "Configuration"
                if (column == 2)
                {
                    // Schunk Lwa4p manual control
                    if (m_operational_mode == 0)
                    {
                        for (int i = 0; i < m_lwa4p_manual_control_modes_names.size(); i++)
                        {
                            if (ImGui::Selectable(m_lwa4p_manual_control_modes_names.at(i).c_str(),
                                m_lwa4p_manual_control_mode == i))
                            {
                                m_lwa4p_manual_control_mode = i;
                            }
                        }
                    }

                    // Schunk Lwa4p robot homing selected
                    if (m_operational_mode == 1)
                    {
                        ImGui::AlignTextToFramePadding();
                        ImGui::BulletText("Schunk Lwa4p: ");
                        ImGui::SameLine();
                        m_lwa4p_go_home = ImGui::Button("Go to home position");
                        ImGui::BulletText("Schunk Lwa4p: ");
                        ImGui::SameLine();
                        m_lwa4p_go_zero = ImGui::Button("Go to zero position");
                        ImGui::Unindent();
                    }
                
                    // Needle Experiment Selected
                    if (m_operational_mode == 2)
                    {
                        
                       // Initialize needle experiment
                        m_needle_experiment_init =
                            ImGui::Button("Initialize experiment");
                        ImGui::SameLine();
                       // Execute needle experiment
                       m_needle_experiment_execute =
                        ImGui::Button("Execute experiment");

                        // Table flags
                        static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit |
                        ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter |
                        ImGuiTableFlags_BordersV  | ImGuiTableFlags_NoHostExtendX;

                        // Number of columns                        
                        int cols_num = 3;
                        int ros_num = 1;

                        // Table Title
                        ImGui::Text("Needle Sinusoidal Trajectory - Translation");
                        
                        // Generate table
                        if (ImGui::BeginTable(
                            "needle_experiment_table_translation", cols_num, flags))
                        {
                            // Generate headers
                            ImGui::TableSetupColumn("Amplitude (m)", ImGuiTableColumnFlags_WidthFixed);

                            // Generate headers
                            ImGui::TableSetupColumn("Frequency (Hz)", ImGuiTableColumnFlags_WidthFixed);

                            // Generate headers
                            ImGui::TableSetupColumn("Phase (rad)", ImGuiTableColumnFlags_WidthFixed);

                            // Generate table rows and cols
                            ImGui::TableHeadersRow();

                            for (int row = 0; row < rows_num; row++)
                            {
                                ImGui::TableNextRow();

                                for (int column = 0; column < cols_num; column++)
                                {
                                    ImGui::TableSetColumnIndex(column);
                                    
                                    if (column == 0)
                                    {
                                        for (size_t i = 0; i < m_ne_ta.size(); i++)
                                        {
                                            ImGui::InputScalar(m_ne_ta.at(i).c_str(),
                                            ImGuiDataType_Float, &m_translation_a[i],
                                                &m_needle_experiment_scalar_step);
                                        }
                                    }

                                    if (column == 1)
                                    {
                                        for (size_t i = 0; i < m_ne_tf.size(); i++)
                                        {
                                            ImGui::InputScalar(m_ne_tf.at(i).c_str(),
                                            ImGuiDataType_Float, &m_translation_f[i],
                                                &m_needle_experiment_scalar_step);
                                        }
                                    }

                                    if (column == 2)
                                    {
                                        for (size_t i = 0; i < m_ne_tp.size(); i++)
                                        {
                                            ImGui::InputScalar(m_ne_tp.at(i).c_str(),
                                            ImGuiDataType_Float, &m_translation_p[i],
                                                &m_needle_experiment_scalar_step);
                                        }
                                    }
                                }
                            }
                        }

                        ImGui::EndTable();

                        // Table Title
                        ImGui::Text("Needle Sinusoidal Trajectory - Rotation");

                        // Generate table
                        if (ImGui::BeginTable(
                            "needle_experiment_table_rotation", cols_num, flags))
                        {
                            // Generate headers
                            ImGui::TableSetupColumn("Amplitude (rad)", ImGuiTableColumnFlags_WidthFixed);

                            // Generate headers
                            ImGui::TableSetupColumn("Frequency (Hz)", ImGuiTableColumnFlags_WidthFixed);

                            // Generate headers
                            ImGui::TableSetupColumn("Phase (rad)", ImGuiTableColumnFlags_WidthFixed);

                            // Generate table rows and cols
                            ImGui::TableHeadersRow();

                            for (int row = 0; row < rows_num; row++)
                            {
                                ImGui::TableNextRow();

                                for (int column = 0; column < cols_num; column++)
                                {
                                    ImGui::TableSetColumnIndex(column);
                    
                                    if (column == 0)
                                    {
                                        for (size_t i = 0; i < m_ne_ra.size(); i++)
                                        {
                                            ImGui::InputScalar(m_ne_ra.at(i).c_str(),
                                            ImGuiDataType_Float, &m_rotation_a[i],
                                                &m_needle_experiment_scalar_step);
                                        }
                                    }

                                    if (column == 1)
                                    {
                                        for (size_t i = 0; i < m_ne_rf.size(); i++)
                                        {
                                            ImGui::InputScalar(m_ne_rf.at(i).c_str(),
                                            ImGuiDataType_Float, &m_rotation_f[i],
                                                &m_needle_experiment_scalar_step);
                                        }
                                    }

                                    if (column == 2)
                                    {
                                        for (size_t i = 0; i < m_ne_rp.size(); i++)
                                        {
                                            ImGui::InputScalar(m_ne_rp.at(i).c_str(),
                                            ImGuiDataType_Float, &m_rotation_p[i],
                                                &m_needle_experiment_scalar_step);
                                        }
                                    }
                                }
                            }
                        }

                        ImGui::EndTable();
                    }
                }
            }
        }
    }

    ImGui::EndTable();
}

// Setup elements status
void ControlPanelWindow::setup_elements_status(void)
{
    // Lwa4p Status
    {
        ImGui::Text("Lwa4p Status: ");
        ImGui::SameLine();
        std::vector<int> status_color = m_status_color.at((int) m_lwa4p_status);
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(status_color[0],
            status_color[1], status_color[2], status_color[3]));
        ImGui::Text(m_status_string.at((int) m_lwa4p_status).c_str());
        ImGui::PopStyleColor();
    }

    // Cyton Status
    {
        ImGui::Text("Cyton Status: ");
        ImGui::SameLine();
        std::vector<int> status_color = m_status_color.at((int) m_cyton_status);
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(status_color[0],
            status_color[1], status_color[2], status_color[3]));
        ImGui::Text(m_status_string.at((int) m_cyton_status).c_str());
        ImGui::PopStyleColor();
    }

    // Axia status
    {
        ImGui::Text("Axia Status: ");
        ImGui::SameLine();
        std::vector<int> status_color = m_status_color.at((int) m_axia_status);
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(status_color[0],
            status_color[1], status_color[2], status_color[3]));
        ImGui::Text(m_status_string.at((int) m_axia_status).c_str());
        ImGui::PopStyleColor();
    }

    // Camera 1 status
    {
        ImGui::Text("Camera 1 Status: ");
        ImGui::SameLine();
        std::vector<int> status_color = m_status_color.at((int) m_camera1_status);
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(status_color[0],
            status_color[1], status_color[2], status_color[3]));
        ImGui::Text(m_status_string.at((int) m_camera1_status).c_str());
        ImGui::PopStyleColor();
    }

    // Camera 2 status
    {
        ImGui::Text("Camera 2 Status: ");
        ImGui::SameLine();
        std::vector<int> status_color = m_status_color.at((int) m_camera2_status);
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(status_color[0],
            status_color[1], status_color[2], status_color[3]));
        ImGui::Text(m_status_string.at((int) m_camera2_status).c_str());
        ImGui::PopStyleColor();
    }

}

// Lwa4p tab 
void ControlPanelWindow::lwa4p_tab(void)
{
    // Table flags
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter |
        ImGuiTableFlags_BordersV  | ImGuiTableFlags_NoHostExtendX;
        
    // Number of rows and columns
    int rows_num = 1;
    int cols_num;

    if (m_lwa4p_manual_control_mode == 0 || m_lwa4p_manual_control_mode == 1)
    {
        cols_num = 3;
    }
    else 
    {
        cols_num = 2;
    }

    // Generate table
    if (ImGui::BeginTable("lwa4p_table", cols_num, flags))
    {
        // Generate headers
        for (size_t i = 0; i < cols_num; i++)
        {
            ImGui::TableSetupColumn(m_lwa4p_table_headers.at(i).c_str(),
                ImGuiTableColumnFlags_WidthFixed);
        }

        // Generate table rows and cols
        ImGui::TableHeadersRow();

        for (int row = 0; row < rows_num; row++)
        {
            ImGui::TableNextRow();

            for (int column = 0; column < cols_num; column++)
            {
                ImGui::TableSetColumnIndex(column);
                    
                // Column "Current Joint Configuration"
                if (column == 0)
                {
                   for (size_t i = 0; i < m_lwa4p_joints_num; i++) 
                   {
                       // Set text 
                       std::string value_text =
                        m_lwa4p_joint_names.at(i) + ": %f";

                       ImGui::Text(value_text.c_str(), m_lwa4p_joint_angles[i]);
                   }
                }
                    
                // Column "Current End-Effector Pose"
                if (column == 1)
                {
                   for (size_t i = 0; i < m_lwa4p_ee_pose_num; i++) 
                   {
                       // Set text 
                       std::string value_text =
                        m_lwa4p_ee_names.at(i) + ": %f";

                       ImGui::Text(value_text.c_str(), m_lwa4p_ee_pose[i]);
                   }
                }

                if (column == 2)
                {
                    if (m_lwa4p_manual_control_mode == 0)
                    {
                        for (size_t i = 0; i < m_lwa4p_joints_num; i++)
                        {
                            ImGui::InputScalar(m_lwa4p_joint_names.at(i).c_str(),
                                ImGuiDataType_Float, &m_lwa4p_des_joint_angles[i],
                                &m_lwa4p_joint_state_step);
                        }                       
                    }
                    else 
                    {
                        for (size_t i = 0; i < m_lwa4p_ee_pose_num; i++)
                        {
                            ImGui::InputScalar(m_lwa4p_ee_names.at(i).c_str(),
                                ImGuiDataType_Float, &m_lwa4p_des_ee_pose[i],
                                &m_lwa4p_ee_state_step[i]);
                        }                       
                    } 
                }
            }
        }
    }

    ImGui::EndTable();
}

// Axia tab
void ControlPanelWindow::axia_tab(void)
{
    ImGui::Text("Sensor's centre of mass with respect to the measurement origin frame");

    // Position in x
    ImGui::InputScalar("Position x(m)", ImGuiDataType_Float, &m_rtct_ft_ft[0],
        &m_axia_com_slider_step);

    // Position in y
    ImGui::InputScalar("Position y(m)", ImGuiDataType_Float, &m_rtct_ft_ft[1],
        &m_axia_com_slider_step);

    // Position in z
    ImGui::InputScalar("Position z(m)", ImGuiDataType_Float, &m_rtct_ft_ft[2],
        &m_axia_com_slider_step);
}

// Vision system tab
void ControlPanelWindow::vision_system_tab(void)
{
    if (ImGui::BeginTabBar("VisionTabBar", ImGuiTabBarFlags_None))
    {
        if (ImGui::BeginTabItem("Image Processing"))
        {
            // Table flags
            static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit |
                ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter |
                ImGuiTableFlags_BordersV  | ImGuiTableFlags_NoHostExtendX;
        
            // Number of rows and columns
            int rows_num = 1;
            int cols_num = 2;

            // Generate table
            if (ImGui::BeginTable("vision_system_table", cols_num, flags))
            {
                // Generate headers
                for (size_t i = 0; i < cols_num; i++)
                {
                    ImGui::TableSetupColumn(m_vs_table_headers.at(i).c_str(),
                        ImGuiTableColumnFlags_WidthFixed);
                }

                // Generate table rows and cols
                ImGui::TableHeadersRow();

                for (int row = 0; row < rows_num; row++)
                {
                    ImGui::TableNextRow();

                    for (int column = 0; column < cols_num; column++)
                    {
                        // Get column number to string                
                        std::string column_str = std::to_string(column);

                        ImGui::TableSetColumnIndex(column);

                        /************* Enable stream checkbox *****************/
                        // Add an extra id to differentiate strings (See ImGui doc)
                        std::string enable_stream_str = "Enable stream##" + column_str;

                        ImGui::Checkbox(enable_stream_str.c_str(),
                            &m_vs_enable_stream[column]);

                        // Add points to the image
                        if (m_vs_enable_stream[column])
                        {
                            // Enable corresponding points 
                            std::string enable_cp_str = "Enable corresponding points##"
                                + column_str;

                            ImGui::Checkbox(enable_cp_str.c_str(),
                                &m_vs_enable_cp[column]);
                        
                            // Enable base point
                            std::string enable_bp_str = "Enable base point##"
                                + column_str;

                            ImGui::Checkbox(enable_bp_str.c_str(),
                                &m_vs_enable_bp[column]);
                        }

                        ImGui::Separator();

                        /************* Image Type *****************/
                        if (m_vs_enable_stream[column])
                        {
                            ImGui::Text("Select Image Type");

                            // Select image type
                            for (int i = 0; i < m_vs_image_types_names.size(); i++)
                            {
                                // Add an extra id to differentiate strings (See ImGui)
                                std::string vs_image_types_name =
                                    m_vs_image_types_names.at(i) + "##" + std::to_string(column);

                                if (ImGui::Selectable(vs_image_types_name.c_str(),
                                    m_vs_image_type.at(column) == i))
                                {
                                    m_vs_image_type.at(column) = i;
                                }
                            }
                        }

                        // Threshold Value for binary images
                        if (m_vs_image_type[column] == 1 || m_vs_image_type[column] == 2)
                        {
                            ImGui::Text("Set Threshold Value");

                            // Threshold value label
                            std::string thresh_value_label = "Threshold Value##" + 
                                column_str;

                            ImGui::InputScalar(thresh_value_label.c_str(),
                                ImGuiDataType_Float, &m_binary_image_thresh_val[column],
                                &m_binary_image_thrash_val_step);
                        }

                        
                    }
                }

                ImGui::EndTable();
            }
            
            // End tab item
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Camera Positioning"))
        {

            // Table flags
            static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit |
                ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter |
                ImGuiTableFlags_BordersV  | ImGuiTableFlags_NoHostExtendX;
        
            // Number of rows and columns
            int rows_num = 1;
            int cols_num = 2;

            // Generate table
            if (ImGui::BeginTable("vision_system_positioning_table", cols_num, flags))
            {
                // Generate headers
                for (size_t i = 0; i < cols_num; i++)
                {
                    ImGui::TableSetupColumn(m_vs_table_headers.at(i).c_str(),
                        ImGuiTableColumnFlags_WidthFixed);
                }

                // Generate table rows and cols
                ImGui::TableHeadersRow();

                for (int row = 0; row < rows_num; row++)
                {
                    ImGui::TableNextRow();

                    for (int column = 0; column < cols_num; column++)
                    {
                        // Get column number to string                
                        std::string column_str = std::to_string(column);

                        ImGui::TableSetColumnIndex(column);
                        
                        // /************************** Camera pose *********************/
                        ImGui::Text("Set Camera Position and Orientation wrt Inertial");
                        ImGui::Separator();
                        ImGui::Text("Camera Position");
                        // Position in x
                        std::string position_x_label = "Position x(m)##" + column_str;
                        ImGui::InputScalar(position_x_label.c_str(), ImGuiDataType_Float,
                            &m_camera_pos[column][0], &m_camera_pos_rot_slider_step);

                        // Position in y
                        std::string position_y_label = "Position y(m)##" + column_str;
                        ImGui::InputScalar(position_y_label.c_str(), ImGuiDataType_Float,
                            &m_camera_pos[column][1], &m_camera_pos_rot_slider_step);
                   
                        // Position in z
                        std::string position_z_label = "Position z(m)##" + column_str;
                        ImGui::InputScalar(position_z_label.c_str(), ImGuiDataType_Float,
                        &m_camera_pos[column][2], &m_camera_pos_rot_slider_step);
                        
                        ImGui::Text("Camera Orientation");
                        ImGui::Separator();
                        // Orientation in x
                        std::string position_phi_label = "Phi (rad)##" + column_str;
                        ImGui::InputScalar(position_phi_label.c_str(), ImGuiDataType_Float,
                            &m_camera_euler[column][0], &m_camera_pos_rot_slider_step);

                        // Orientation in y
                        std::string position_theta_label = "Theta (rad)##" + column_str;
                        ImGui::InputScalar(position_theta_label.c_str(), ImGuiDataType_Float,
                            &m_camera_euler[column][1], &m_camera_pos_rot_slider_step);

                        // Orientation in z
                        std::string position_psi_label = "Psi (rad)##" + column_str;
                        ImGui::InputScalar(position_psi_label.c_str(), ImGuiDataType_Float,
                            &m_camera_euler[column][2], &m_camera_pos_rot_slider_step);
                    }
                }

                ImGui::EndTable();
            }
            
            // End tab item
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }
}



/*************************** Seters ******************************/
// Set lwa4p joints    
void ControlPanelWindow::set_lwa4p_joint_angles(std::vector<double>& joint_angles)
{
    for (size_t i = 0; i < m_lwa4p_joints_num; i++)
    {
        m_lwa4p_joint_angles[i] = (float) joint_angles.at(i);
    }
}

// Set lwa4p ee pose    
void ControlPanelWindow::set_lwa4p_ee_pose(std::vector<double>& ee_pose)
{
    for (size_t i = 0; i < m_lwa4p_ee_pose_num; i++)
    {
        m_lwa4p_ee_pose[i] = (float) ee_pose.at(i);
    }
}

// Set initial desired lwa4p joints angles
void ControlPanelWindow::set_lwa4p_initial_desired_joint_angles(
    std::vector<double>& joint_angles)
{
    for (size_t i = 0; i < m_lwa4p_joints_num; i++)
    {
        m_lwa4p_des_joint_angles[i] = (float) joint_angles.at(i);
    }
}

// Set initial desired lwa4p ee pose
void ControlPanelWindow::set_lwa4p_initial_desired_ee_pose(std::vector<double>&
    ee_pose)
{
    for (size_t i = 0; i < m_lwa4p_ee_pose_num; i++)
    {
        m_lwa4p_des_ee_pose[i] = (float) ee_pose.at(i);
    }
}

// Get lwa4p desired joint angles
std::vector<double> ControlPanelWindow::get_lwa4p_desired_joint_angles(void)
{
    // Initialize vector of desired joint angles
    std::vector<double> des_joint_angles(m_lwa4p_joints_num);

    for (size_t i = 0; i < m_lwa4p_joints_num; i++) 
    {
        des_joint_angles.at(i) = (double) m_lwa4p_des_joint_angles[i];
    }
    return des_joint_angles;
}

// Get desired lwa4p ee pose
std::vector<double> ControlPanelWindow::get_lwa4p_desired_ee_pose(void)
{
    // Initialize vector of desired ee pose
    std::vector<double> des_ee_pose(m_lwa4p_ee_pose_num);

    for (size_t i = 0; i < m_lwa4p_ee_pose_num; i++) 
    {
        des_ee_pose.at(i) = (double) m_lwa4p_des_ee_pose[i];
    }
    return des_ee_pose;
}

// Get needle vibration experiment translation parameters
std::vector<std::vector<double>> ControlPanelWindow::get_nve_translation_params(void)
{
    // // Initialize parameters
    // std::vector<std::vector<double>> params;

    // Translation amplitude
    std::vector<double> a = {(double)m_translation_a[0],
        (double)m_translation_a[1], (double)m_translation_a[2]};

    // Translation frequency
    std::vector<double> f = {(double)m_translation_f[0],
        (double)m_translation_f[1], (double)m_translation_f[2]};

    // Translation phase
    std::vector<double> p = {(double)m_translation_p[0],
        (double)m_translation_p[1], (double)m_translation_p[2]};

    return {a, f, p};
}

// Get needle vibration experiment rotation parameters
std::vector<std::vector<double>> ControlPanelWindow::get_nve_rotation_params(void)
{
    // Rotation amplitude
    std::vector<double> a = {(double)m_rotation_a[0],
        (double)m_rotation_a[1], (double)m_rotation_a[2]};

    // Rotation frequency
    std::vector<double> f = {(double)m_rotation_f[0],
        (double)m_rotation_f[1], (double)m_rotation_f[2]};

    // Roation phase
    std::vector<double> p = {(double)m_rotation_p[0],
        (double)m_rotation_p[1], (double)m_rotation_p[2]};

    return {a, f, p};
}

// Get sensor's centre of mass with respect to the measurement origin frame
std::vector<double> ControlPanelWindow::get_axia_com_wrt_measurement_reference_frame(void)
{
    return {(double) m_rtct_ft_ft[0], (double) m_rtct_ft_ft[1],
        (double) m_rtct_ft_ft[2]};
}

// Get camera 1 stream status
bool ControlPanelWindow::get_camera1_stream_status(void)
{
    return m_vs_enable_stream[0];
}

// Get camera 2 stream status
bool ControlPanelWindow::get_camera2_stream_status(void)
{
    return m_vs_enable_stream[1];
}

// Get camera 1 corresponding points status
bool ControlPanelWindow::get_camera1_cp_status(void)
{
    return m_vs_enable_cp[0];
}

// Get camera 2 stream status
bool ControlPanelWindow::get_camera2_cp_status(void)
{
    return m_vs_enable_cp[1];
}

// Get camera 1 background point status
bool ControlPanelWindow::get_camera1_bp_status(void)
{
    return m_vs_enable_bp[0];
}

// Get camera 2 stream status
bool ControlPanelWindow::get_camera2_bp_status(void)
{
    return m_vs_enable_bp[1];
}

// Get camera 1 image type
int ControlPanelWindow::get_camera1_image_type(void)
{
    return m_vs_image_type[0];
}

// Get camera 2 image type
int ControlPanelWindow::get_camera2_image_type(void)
{
    return m_vs_image_type[1];
}

// Get camera 1 binary threshold value
double ControlPanelWindow::get_camera1_binary_threshold_value(void)
{
    return (double)m_binary_image_thresh_val[0];
}

// Get camera 2 binary threshold value
double ControlPanelWindow::get_camera2_binary_threshold_value(void)
{
    return (double)m_binary_image_thresh_val[1];
}

// Get camera 1 inertial position
std::vector<double> ControlPanelWindow::get_camera1_inertial_position(void)
{
    return {(double)m_camera_pos[0][0], (double)m_camera_pos[0][1], 
        (double)m_camera_pos[0][2]};
}

// Get camera 2 inertial position
std::vector<double> ControlPanelWindow::get_camera2_inertial_position(void)
{
    return {(double)m_camera_pos[1][0], (double)m_camera_pos[1][1], 
        (double)m_camera_pos[1][2]};
}

// Get camera 1 orientation position
std::vector<double> ControlPanelWindow::get_camera1_orientation(void)
{
    return {(double)m_camera_euler[0][0], (double)m_camera_euler[0][1], 
        (double)m_camera_euler[0][2]};
}

// Get camera 2 orientation position
std::vector<double> ControlPanelWindow::get_camera2_orientation(void)
{
    return {(double)m_camera_euler[1][0], (double)m_camera_euler[1][1], 
        (double)m_camera_euler[1][2]};
}