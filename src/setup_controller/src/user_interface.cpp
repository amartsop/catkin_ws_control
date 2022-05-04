#include "../include/user_interface.h"


void UserInterface::initialize(std::shared_ptr<Display> display)
{
    // Get the display pointer
    m_display = display;

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    const char* glsl_version = "#version 130";

    // Setup Dear ImGui style
    set_ImGui_style();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(m_display->get_window_handle(), true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Generate control panel window ptr
    m_cp_window_ptr = std::make_shared<ControlPanelWindow>();

    // Generate camera 1 window ptr
    m_camera1_window_ptr = std::make_shared<CameraWindow>();

    // Generate camera 2 window ptr
    m_camera2_window_ptr = std::make_shared<CameraWindow>();
}

// Update gui state
void UserInterface::update(void)
{
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Get display width and height
    m_display_width = m_display->get_window_width();
    m_display_height = m_display->get_window_height();
}

// Render gui
void UserInterface::render(void)
{
    // Rendering
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

// Render animation window
void UserInterface::render_experiment_window(unsigned int texture_color_buffer)
{
   // Set animation window width and height 
    m_ew_dims.x = m_ew_ratio * m_display_width;
    m_ew_dims.y = m_ew_ratio * m_display_height;

    ImGui::Begin(m_ew_name.c_str());
    {
        ImGui::SetWindowSize(m_ew_dims);
        ImGui::SetWindowPos(m_ew_pos);
        ImVec2 wsize = ImGui::GetWindowSize();
        ImGui::Image((ImTextureID)texture_color_buffer, wsize,
            ImVec2(0, 1), ImVec2(1, 0));
    }

    ImGui::End();
}

// Render camera 1 window
void UserInterface::render_camera1_window(void)
{
   // Set animation window width and height 
    m_cam1_dims.x = m_cam1_ratio * m_display_width;
    m_cam1_dims.y = 0.5 * m_ew_dims.y;

    // Calculate position of camera 1 window
    float cam1_x = m_ew_pos.x + m_ew_dims.x;
    float cam1_y = m_ew_pos.y;
    m_cam1_pos = ImVec2(cam1_x , cam1_y);

    // Texture id
    unsigned int texture_id = m_camera1_window_ptr->get_texture_id();

    // Canvas size
    ImVec2 canvas_size = m_camera1_window_ptr->get_canvas_size();

    // Update frames
    m_camera1_window_ptr->render_frames();

    ImGui::Begin(m_cam1_name.c_str());
    {
        ImGui::SetWindowSize(m_cam1_dims);
        ImGui::SetWindowPos(m_cam1_pos);
        ImGui::ImageButton((void *)(intptr_t)texture_id, canvas_size,
            ImVec2(0, 0), ImVec2(1, 1), 0);
    }

    ImGui::End();
}

// Render camera 2 window
void UserInterface::render_camera2_window(void)
{
   // Set animation window width and height 
    m_cam2_dims = m_cam1_dims;

    // Calculate position of camera 1 window
    float cam2_x = m_ew_pos.x + m_ew_dims.x;
    float cam2_y = m_ew_pos.y + m_cam2_dims.y;
    m_cam2_pos = ImVec2(cam2_x , cam2_y);

    // Canvas size
    ImVec2 canvas_size = m_camera2_window_ptr->get_canvas_size();

    // Texture id
    unsigned int texture_id = m_camera2_window_ptr->get_texture_id();

    // Update frames
    m_camera2_window_ptr->render_frames();

    ImGui::Begin(m_cam2_name.c_str());
    {
        ImGui::SetWindowSize(m_cam2_dims);
        ImGui::SetWindowPos(m_cam2_pos);
        ImGui::ImageButton((void *)(intptr_t)texture_id, canvas_size,
            ImVec2(0, 0), ImVec2(1, 1), 0);
    }

    ImGui::End();
}

// Control panel window
void UserInterface::render_control_panel(void)
{
   // Set animation window width and height 
    m_cp_dims.x = m_display_width;
    m_cp_dims.y = m_cp_ratio * m_display_height;

    // Calculate position of camera 1 window
    float cp_x = m_ew_pos.x;
    float cp_y = m_ew_pos.y + m_ew_dims.y;
    m_cp_pos = ImVec2(cp_x, cp_y);

    ImGui::Begin(m_cp_name.c_str());
    {
        ImGui::SetWindowSize(m_cp_dims);
        ImGui::SetWindowPos(m_cp_pos);
        ImVec2 wsize = ImGui::GetWindowSize();
        
        // Render window
        m_cp_window_ptr->render();
    }
    
    ImGui::End();
}

UserInterface::~UserInterface()
{
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

// Set ImGui Style
void UserInterface::set_ImGui_style(void)
{
    ImGuiStyle* style = &ImGui::GetStyle();
    style->WindowPadding = ImVec2(15, 15);
	style->WindowRounding = 5.0f;
	style->FramePadding = ImVec2(5, 5);
	style->FrameRounding = 4.0f;
	style->ItemSpacing = ImVec2(12, 8);
	style->ItemInnerSpacing = ImVec2(8, 6);
	style->IndentSpacing = 25.0f;
	style->ScrollbarSize = 15.0f;
	style->ScrollbarRounding = 9.0f;
	style->GrabMinSize = 5.0f;
	style->GrabRounding = 3.0f;
 
	style->Colors[ImGuiCol_Text] = ImVec4(0.80f, 0.80f, 0.83f, 1.00f);
	style->Colors[ImGuiCol_TextDisabled] = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
	style->Colors[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
	style->Colors[ImGuiCol_PopupBg] = ImVec4(0.07f, 0.07f, 0.09f, 1.00f);
	style->Colors[ImGuiCol_Border] = ImVec4(0.80f, 0.80f, 0.83f, 0.88f);
	style->Colors[ImGuiCol_BorderShadow] = ImVec4(0.92f, 0.91f, 0.88f, 0.00f);
	style->Colors[ImGuiCol_FrameBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
	style->Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
	style->Colors[ImGuiCol_FrameBgActive] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
	style->Colors[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
	style->Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(1.00f, 0.98f, 0.95f, 0.75f);
	style->Colors[ImGuiCol_TitleBgActive] = ImVec4(0.07f, 0.07f, 0.09f, 1.00f);
	style->Colors[ImGuiCol_MenuBarBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
	style->Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
	style->Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.80f, 0.80f, 0.83f, 0.31f);
	style->Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
	style->Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
	style->Colors[ImGuiCol_CheckMark] = ImVec4(0.80f, 0.80f, 0.83f, 0.31f);
	style->Colors[ImGuiCol_SliderGrab] = ImVec4(0.80f, 0.80f, 0.83f, 0.31f);
	style->Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
	style->Colors[ImGuiCol_Button] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
	style->Colors[ImGuiCol_ButtonHovered] = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
	style->Colors[ImGuiCol_ButtonActive] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
	style->Colors[ImGuiCol_Header] = ImVec4(0.50f, 0.5f, 0.6f, 1.00f);
	style->Colors[ImGuiCol_HeaderHovered] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
	style->Colors[ImGuiCol_HeaderActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
	style->Colors[ImGuiCol_ResizeGrip] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	style->Colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
	style->Colors[ImGuiCol_ResizeGripActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
	style->Colors[ImGuiCol_PlotLines] = ImVec4(0.40f, 0.39f, 0.38f, 0.63f);
	style->Colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.25f, 1.00f, 0.00f, 1.00f);
	style->Colors[ImGuiCol_PlotHistogram] = ImVec4(0.40f, 0.39f, 0.38f, 0.63f);
	style->Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.25f, 1.00f, 0.00f, 1.00f);
	style->Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.25f, 1.00f, 0.00f, 0.43f);
}
