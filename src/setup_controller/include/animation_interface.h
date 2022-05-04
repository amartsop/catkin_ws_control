#pragma once

#include <iostream>
#include "ros/package.h"
#include <map>
#include <memory>
#include <armadillo>
#include <filesystem>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <scene.hpp>
#include <object_type.hpp>
#include <object.h>
#include <imported_surface.h>
#include <primitives/point.h>
#include <primitives/line.h>
#include <primitives/box.h>

#include "utils.h"


class AnimationInterface
{

public:
    AnimationInterface(){};

    // Initialize animation
    void initialize_animation(Scene<AnimationInterface>* scene);

    // Update objects
    void update(double real_time);

public:
    // Set lwa4p links positions
    void set_lwa4p_links_positions(const std::vector<glm::vec3>& links_positions)
    {
        m_lwa4p_links_positions = links_positions;
    }

    // Set lwa4p links orientations
    void set_lwa4p_links_orientations(const std::vector<glm::vec3>& links_orientations)
    {
       m_lwa4p_links_orientations = links_orientations; 
    }

private:

    // Generate background
    void generate_background(void);

    // Generate table
    void generate_table(void);

    // Generate lwa4p
    void generate_lwa4p(void);

private:
    /************************** Objects *******************/
    // Objects container
    std::vector<std::shared_ptr<Object>> m_objects;

    // Simple shader path
    std::string m_simple_shader_path;

    // Current path
    std::filesystem::path m_current_path;
    

private:
    // Background id
    int m_background_id;
    
private:

    // Lwa4p ids
    std::vector<int> m_lwa4p_ids;
    
    // Lwa4p relative mesh names
    std::vector<std::string> m_lwa4p_rel_mesh_names = 
    {
        "objects/lwa4p/lwa4p_link0.obj",
        "objects/lwa4p/lwa4p_link1.obj",
        "objects/lwa4p/lwa4p_link2.obj",
        "objects/lwa4p/lwa4p_link3.obj",
        "objects/lwa4p/lwa4p_link4.obj",
        "objects/lwa4p/lwa4p_link5.obj",
        "objects/lwa4p/lwa4p_link6.obj"
    };

    // Lwa4p relative texture names
    std::vector<std::filesystem::path> m_lwa4p_rel_texture_names = 
    {
        "objects/lwa4p/lwa4p_metal.png", 
        "objects/lwa4p/lwa4p_metal.png",
        "objects/lwa4p/lwa4p_plastic.png",
        "objects/lwa4p/lwa4p_metal.png",
        "objects/lwa4p/lwa4p_plastic.png",
        "objects/lwa4p/lwa4p_metal.png",
        "objects/lwa4p/lwa4p_metal.png"
    };

    // Lwa4p links positions
    std::vector<glm::vec3> m_lwa4p_links_positions;

    // Lwa4p links orientations
    std::vector<glm::vec3> m_lwa4p_links_orientations;

private:

    // Background relative mesh name
    std::filesystem::path m_bkg_rel_mesh_name =
        "objects/floor/floor.obj";

    // Background relative texture names
    std::filesystem::path m_bkg_rel_texture_name =
        "objects/floor/floor.png";

private:

    // Table relative mesh name
    std::filesystem::path m_table_rel_mesh_name =
        "objects/table/table.obj";

    // Table relative texture names
    std::filesystem::path m_table_rel_texture_name =
        "objects/table/table.png";
};