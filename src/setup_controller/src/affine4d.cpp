#include "../include/affine4d.h"

// Constructor
Affine4d::Affine4d(const arma::dmat44& t_mat)
{
    // Set transformation matrix
    m_t_mat = t_mat;

    // Set rotation matrix
    m_rot_mat = t_mat(arma::span(0, 2), arma::span(0, 2));

    // Set position vector
    m_pos_vec = t_mat(arma::span(0, 2), 3);
}

// Constructor overload
Affine4d::Affine4d(const arma::dvec3& pos_vec, const arma::dmat33& rot_mat)
{
    // Set rotation matrix
    m_rot_mat = rot_mat;

    // Set position vector
    m_pos_vec = pos_vec;

    // Set transformation matrix
    m_t_mat = arma::eye(4, 4);
    m_t_mat(arma::span(0, 2), arma::span(0, 2)) = rot_mat;
    m_t_mat(arma::span(0, 2), 3) = pos_vec;
}

// Set transformation matrix
void Affine4d::set_transformation_matrix(const arma::dmat44& t_mat)
{
    // Set transformation matrix
    m_t_mat = t_mat; 

    // Set rotation matrix
    m_rot_mat = m_t_mat(arma::span(0, 2), arma::span(0, 2));

    // Set position vector
    m_pos_vec = m_t_mat(arma::span(0, 2), 3);
}


// Set rotation matrix
void Affine4d::set_rotation_matrix(const arma::dmat33& rot_mat)
{
    // Set rotation matrix
    m_rot_mat = rot_mat;

    // Set transformation matrix
    m_t_mat(arma::span(0, 2), arma::span(0, 2)) = rot_mat;
}

// Set position vector
void Affine4d::set_position_vector(const arma::dvec3& pos_vec)
{
    // Set position vector
    m_pos_vec = pos_vec;

    // Set transformation matrix
    m_t_mat(arma::span(0, 2), 3) = pos_vec;
}