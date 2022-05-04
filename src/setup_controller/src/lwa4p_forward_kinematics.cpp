#include "../include/lwa4p_forward_kinematics.h"

Lwa4pForwardKinematics::Lwa4pForwardKinematics(/* args */)
{
    // Initialize links positions and orientations
    m_links_positions.resize(m_links_num);
    m_links_orientations.resize(m_links_num);

    for (size_t i = 0; i < m_links_positions.size(); i++)
    {
        m_links_positions.at(i) = {0.0, 0.0, 0.0};
        m_links_orientations.at(i) = {0.0, 0.0, 0.0};
    }

    // Initialize relative transformation matrices
    m_rel_tm.resize(m_links_num);

    // Initialize absolute transformation matrices
    m_abs_tm.resize(m_links_num);

    // Relative transformation of link 0
    m_rel_tm.at(0) = m_t_f0_fr.get_transformation_matrix();
}

// Calcualte the forward kinematics for the lwa4p arm
void Lwa4pForwardKinematics::calculate(const std::vector<double>& joint_angles)
{
    // Relative transformation of link 1
    arma::dmat rot_f1_f0 = dma::EulerRotations::rotation({0.0, 0.0, joint_angles[0]});
    m_t_f1_f0.set_rotation_matrix(rot_f1_f0);
    m_rel_tm.at(1) = m_t_f1_f0.get_transformation_matrix();

    // Relative transformation of link 2 (manually insert -1 because of weird schunk joint config)
    arma::dmat rot_f2_f1 = dma::EulerRotations::rotation({0.0, -joint_angles[1], 0.0});
    m_t_f2_f1.set_rotation_matrix(rot_f2_f1);
    m_rel_tm.at(2) = m_t_f2_f1.get_transformation_matrix();

    // Relative transformation of link 3
    arma::dmat rot_f3_f2 = dma::EulerRotations::rotation({0.0, joint_angles[2], 0.0});
    m_t_f3_f2.set_rotation_matrix(rot_f3_f2);
    m_rel_tm.at(3) = m_t_f3_f2.get_transformation_matrix();

    // Relative transformation of link 4
    arma::dmat rot_f4_f3 = dma::EulerRotations::rotation({0.0, 0.0, joint_angles[3]});
    m_t_f4_f3.set_rotation_matrix(rot_f4_f3);
    m_rel_tm.at(4) = m_t_f4_f3.get_transformation_matrix();

    // Relative transformation of link 5
    arma::dmat rot_f5_f4 = dma::EulerRotations::rotation({0.0, joint_angles[4], 0.0});
    m_t_f5_f4.set_rotation_matrix(rot_f5_f4);
    m_rel_tm.at(5) = m_t_f5_f4.get_transformation_matrix();

    // Relative transformation of link 6
    arma::dmat rot_f6_f5 = dma::EulerRotations::rotation({0.0, 0.0, joint_angles[5]});
    m_t_f6_f5.set_rotation_matrix(rot_f6_f5);
    m_rel_tm.at(6) = m_t_f6_f5.get_transformation_matrix();

    // Initialize global transformation matrix
    Affine4d t_mat_affine = Affine4d(m_r_fr_F_F, m_rot_fr_F);
    arma::dmat t_mat = t_mat_affine.get_transformation_matrix();

    // Generate global transfromation matrices
    for (size_t i = 0; i < m_rel_tm.size(); i++)
    {
        // Update global transformation matrix
        t_mat = t_mat * m_rel_tm.at(i);
        t_mat_affine.set_transformation_matrix(t_mat);
    
        // Get rotation and position from transformation matrix
        m_links_positions.at(i) = t_mat_affine.get_position_vector();
        m_links_orientations.at(i) =
            dma::EulerRotations::rotation_to_euler(t_mat_affine.get_rotation_matrix());
    }
}

// Get links positions
std::vector<glm::vec3> Lwa4pForwardKinematics::get_links_positions(void)
{
    // Initialize links positions
    std::vector<glm::vec3> links_positions(m_links_num);

    for (size_t i = 0; i < m_links_num; i++)    
    {
        links_positions.at(i) = Utils::arma_to_glm_vec3(m_links_positions.at(i));
    }

    return links_positions;
}

// Get links orienations
std::vector<glm::vec3> Lwa4pForwardKinematics::get_links_orientations(void)
{
    // Initialize links orientations
    std::vector<glm::vec3> links_orientations(m_links_num);

    for (size_t i = 0; i < m_links_num; i++)    
    {
        links_orientations.at(i) =
            Utils::arma_to_glm_vec3(m_links_orientations.at(i));
    }

    return links_orientations;
}