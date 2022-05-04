#pragma once 

#include <iostream>
#include <vector>
#include <armadillo>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>

#include "utils.h"
#include "affine4d.h"
#include <dynamics_math_arma/dynamics_math_arma.h>

class Lwa4pForwardKinematics
{
public:
    Lwa4pForwardKinematics(/* args */);

    // Calcualte the forward kinematics for the lwa4p arm
    void calculate(const std::vector<double>& joint_angles);

    // Get links positions
    std::vector<glm::vec3> get_links_positions(void);

    // Get link orientations
    std::vector<glm::vec3> get_links_orientations(void);

private:

    // Number of links
    int m_links_num = 7;

    // Link absolute positions
    std::vector<arma::dvec> m_links_positions;

    // Link absolute orientations
    std::vector<arma::dvec> m_links_orientations;

private:

    // Position of base frame with respect to inertial frame
    arma::dvec m_r_fr_F_F = {-0.332649f, 0.3f, -0.022f};

    // Orientation of base frame with respect to the inertial frame
    arma::dmat m_rot_fr_F = dma::EulerRotations::rotation({0.0, 0.0, 0.0});

    /**
     * @brief  Relative transformation matrices
     * {t_f0_fr, t_f1_f0, t_f2_f1, t_f3_f2, t_f4_f3, t_f5_f4, t_f6_f5}
     */
    std::vector<arma::dmat> m_rel_tm;

    /**
     * @brief  Absolute transformation matrices 
     * {t_f0_F, t_f1_F, t_f2_F, t_f3_F, t_f4_F, t_f5_F, t_f6_F}
     */
    std::vector<arma::dmat> m_abs_tm;

private:
    
    // Eye rotation
    arma::dmat eye_rot = dma::EulerRotations::rotation({0.0, 0.0, 0.0});

    // Relative transformation of link 0
    Affine4d m_t_f0_fr = Affine4d({0.0, 0.0, 0.0}, eye_rot);

    // Relative transformation of link 1
    Affine4d m_t_f1_f0 = Affine4d({0.0, 0.0, 0.205}, eye_rot);

    // Relative transformation of link 2
    Affine4d m_t_f2_f1 = Affine4d({0.0, 0.0, 0.0}, eye_rot);

    // Relative transformation of link 3
    Affine4d m_t_f3_f2 = Affine4d({0.0, 0.0, 0.350}, eye_rot);

    // Relative transformation of link 4
    Affine4d m_t_f4_f3 = Affine4d({0.0, 0.0, 0.0}, eye_rot);
    
    // Relative transformation of link 5
    Affine4d m_t_f5_f4 = Affine4d({0.0, 0.0, 0.305}, eye_rot);

    // Relative transformation of link 5
    Affine4d m_t_f6_f5 = Affine4d({0.0, 0.0, 0.07475}, eye_rot);
};