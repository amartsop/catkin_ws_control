#include "../include/animation_interface.h"

void AnimationInterface::initialize_animation(Scene<AnimationInterface>* scene)
{
    // Get simple shader path
    m_simple_shader_path = scene->get_simple_shader_path();
    
    // Get current path
    // m_current_path = std::filesystem::current_path();
    m_current_path = std::filesystem::path(ros::package::getPath("setup_controller"));
   
    // Generate background
    generate_background();

    // Generate table
    generate_table();

    // Generate Lwa4p
    generate_lwa4p();
}

// Update all scene objects
void AnimationInterface::update(double real_time)
{
    // Update lwa4p state
    for (size_t i = 0; i < m_lwa4p_ids.size(); i++)
    {
        m_objects.at(m_lwa4p_ids.at(i))->transform(m_lwa4p_links_positions.at(i),
            m_lwa4p_links_orientations.at(i));
    }

    // Render all objects
    for (size_t i = 0; i < m_objects.size(); i++)
    {
        // Draw objects
        m_objects.at(i)->draw();
    }
}

// Generate background
void AnimationInterface::generate_background(void)
{
    obj_type::ImportedSurfaceProperties iop;
    iop.id = m_objects.size();
    iop.pos = glm::vec3(1.5f, 0.0f, 1.41f);
    iop.euler = glm::vec3(0.0f, 0.0f, M_PI_2);
    iop.shader_filename = m_simple_shader_path;
    iop.mesh_filename = (m_current_path / m_bkg_rel_mesh_name).string();
    iop.texture_filename = (m_current_path / m_bkg_rel_texture_name).string();
    iop.shininess = 10.0f;
    iop.draw_type = GL_STATIC_DRAW;

    m_background_id = iop.id;

    m_objects.push_back(std::make_shared<ImportedSurface>());
    m_objects.at(iop.id)->initialize_imported_surface(iop);
}

// Generate table
void AnimationInterface::generate_table(void)
{
    obj_type::ImportedSurfaceProperties iop;
    iop.id = m_objects.size();
    iop.pos = glm::vec3(0.0f, 0.0f, 0.0f);
    iop.euler = glm::vec3(0.0f, 0.0f, 0.0f);
    iop.shader_filename = m_simple_shader_path;
    iop.mesh_filename = (m_current_path / m_table_rel_mesh_name).string();
    iop.texture_filename = (m_current_path / m_table_rel_texture_name).string();
    iop.shininess = 10.0f;
    iop.draw_type = GL_STATIC_DRAW;

    m_background_id = iop.id;

    m_objects.push_back(std::make_shared<ImportedSurface>());
    m_objects.at(iop.id)->initialize_imported_surface(iop);
}

// Generate lwa4p
void AnimationInterface::generate_lwa4p(void)
{
    // List of objects
    std::vector<obj_type::ImportedSurfaceProperties> iop(7);

    // Resize number of lwa4p ids
    m_lwa4p_ids.resize(7);

    // Setup links 
    for (size_t i = 0; i < m_lwa4p_ids.size(); i++) 
    {
        iop.at(i).id = m_objects.size();
        iop.at(i).pos = glm::vec3(0.0, 0.0, 0.0);
        iop.at(i).euler = glm::vec3(0.0, 0.0, 0.0);
        iop.at(i).shader_filename = m_simple_shader_path;
        iop.at(i).mesh_filename = (m_current_path /
            m_lwa4p_rel_mesh_names.at(i)).string();
        iop.at(i).texture_filename = (m_current_path /
            m_lwa4p_rel_texture_names.at(i)).string();
        iop.at(i).shininess = 60.0f;
        iop.at(i).draw_type = GL_DYNAMIC_DRAW;

        m_lwa4p_ids.at(i) = iop.at(i).id;

        m_objects.push_back(std::make_shared<ImportedSurface>());
        m_objects.at(iop.at(i).id)->initialize_imported_surface(iop.at(i));
    }


    // // Link 0
    // iop.at(0).id = m_objects.size();
    // iop.at(0).pos = glm::vec3(-0.332649f, 0.3f, -0.022f);
    // iop.at(0).euler = glm::vec3(M_PI_2, 0.0f, 0.0f);
    // iop.at(0).shader_filename = m_simple_shader_path;
    // iop.at(0).mesh_filename = (m_current_path / "objects/lwa4p/lwa4p_link0.obj").string();
    // iop.at(0).texture_filename = (m_current_path / "objects/lwa4p/lwa4p_metal.png").string();
    // iop.at(0).shininess = 60.0f;
    // iop.at(0).draw_type = GL_DYNAMIC_DRAW;

    // m_lwa4p_ids.at(0) = iop.at(0).id;

    // m_objects.push_back(std::make_shared<ImportedSurface>());
    // m_objects.at(iop.at(0).id)->initialize_imported_surface(iop.at(0));

    // // Link 1
    // iop.at(1).id = m_objects.size();
    // iop.at(1).pos = glm::vec3(-0.332649f, 0.3f, 0.183f);
    // iop.at(1).euler = glm::vec3(M_PI_2, 0.0f, 0.0f);
    // iop.at(1).shader_filename = m_simple_shader_path;
    // iop.at(1).mesh_filename = (m_current_path / "objects/lwa4p/lwa4p_link1.obj").string();
    // iop.at(1).texture_filename = (m_current_path / "objects/lwa4p/lwa4p_metal.png").string();
    // iop.at(1).shininess = 10.0f;
    // iop.at(1).draw_type = GL_DYNAMIC_DRAW;

    // m_lwa4p_ids.at(1) = iop.at(1).id;

    // m_objects.push_back(std::make_shared<ImportedSurface>());
    // m_objects.at(iop.at(1).id)->initialize_imported_surface(iop.at(1));

    // // Link 2
    // iop.at(2).id = m_objects.size();
    // iop.at(2).pos = glm::vec3(-0.332649f, 0.3f, 0.183f);
    // iop.at(2).euler = glm::vec3(M_PI_2, 0.0f, 0.0f);
    // iop.at(2).shader_filename = m_simple_shader_path;
    // iop.at(2).mesh_filename = (m_current_path / "objects/lwa4p/lwa4p_link2.obj").string();
    // iop.at(2).texture_filename = (m_current_path / "objects/lwa4p/lwa4p_plastic.png").string();
    // iop.at(2).shininess = 10.0f;
    // iop.at(2).draw_type = GL_DYNAMIC_DRAW;

    // m_lwa4p_ids.at(2) = iop.at(2).id;

    // m_objects.push_back(std::make_shared<ImportedSurface>());
    // m_objects.at(iop.at(2).id)->initialize_imported_surface(iop.at(2));

    // // Link 3
    // iop.at(3).id = m_objects.size();
    // iop.at(3).pos = glm::vec3(-0.332649f, 0.3f, 0.533f);
    // iop.at(3).euler = glm::vec3(M_PI_2, 0.0f, 0.0f);
    // iop.at(3).shader_filename = m_simple_shader_path;
    // iop.at(3).mesh_filename = (m_current_path / "objects/lwa4p/lwa4p_link3.obj").string();
    // iop.at(3).texture_filename = (m_current_path / "objects/lwa4p/lwa4p_metal.png").string();
    // iop.at(3).shininess = 10.0f;
    // iop.at(3).draw_type = GL_DYNAMIC_DRAW;

    // m_lwa4p_ids.at(3) = iop.at(3).id;

    // m_objects.push_back(std::make_shared<ImportedSurface>());
    // m_objects.at(iop.at(3).id)->initialize_imported_surface(iop.at(3));

    // // Link 4
    // iop.at(4).id = m_objects.size();
    // iop.at(4).pos = glm::vec3(-0.332649f, 0.3f, 0.533f);
    // iop.at(4).euler = glm::vec3(M_PI_2, 0.0f, 0.0f);
    // iop.at(4).shader_filename = m_simple_shader_path;
    // iop.at(4).mesh_filename = (m_current_path / "objects/lwa4p/lwa4p_link4.obj").string();
    // iop.at(4).texture_filename = (m_current_path / "objects/lwa4p/lwa4p_plastic.png").string();
    // iop.at(4).shininess = 10.0f;
    // iop.at(4).draw_type = GL_DYNAMIC_DRAW;

    // m_lwa4p_ids.at(4) = iop.at(4).id;

    // m_objects.push_back(std::make_shared<ImportedSurface>());
    // m_objects.at(iop.at(4).id)->initialize_imported_surface(iop.at(4));

    // // Link 5
    // iop.at(5).id = m_objects.size();
    // iop.at(5).pos = glm::vec3(-0.332649f, 0.3f, 0.838f);
    // iop.at(5).euler = glm::vec3(M_PI_2, 0.0f, 0.0f);
    // iop.at(5).shader_filename = m_simple_shader_path;
    // iop.at(5).mesh_filename = (m_current_path / "objects/lwa4p/lwa4p_link5.obj").string();
    // iop.at(5).texture_filename = (m_current_path / "objects/lwa4p/lwa4p_metal.png").string();
    // iop.at(5).shininess = 10.0f;
    // iop.at(5).draw_type = GL_DYNAMIC_DRAW;

    // m_lwa4p_ids.at(5) = iop.at(5).id;

    // m_objects.push_back(std::make_shared<ImportedSurface>());
    // m_objects.at(iop.at(5).id)->initialize_imported_surface(iop.at(5));

    // // Link 6
    // iop.at(6).id = m_objects.size();
    // iop.at(6).pos = glm::vec3(-0.332649f, 0.3f, 0.91275f);
    // iop.at(6).euler = glm::vec3(M_PI_2, 0.0f, 0.0f);
    // iop.at(6).shader_filename = m_simple_shader_path;
    // iop.at(6).mesh_filename = (m_current_path / "objects/lwa4p/lwa4p_link6.obj").string();
    // iop.at(6).texture_filename = (m_current_path / "objects/lwa4p/lwa4p_metal.png").string();
    // iop.at(6).shininess = 10.0f;
    // iop.at(6).draw_type = GL_DYNAMIC_DRAW;

    // m_lwa4p_ids.at(6) = iop.at(6).id;

    // m_objects.push_back(std::make_shared<ImportedSurface>());
    // m_objects.at(iop.at(6).id)->initialize_imported_surface(iop.at(6));
}