#pragma once

#include <iostream>
#include <armadillo>

class Affine4d
{
public:

    // Constructor
    Affine4d(const arma::dmat44& t_mat);

    // Constructor overload
    Affine4d(const arma::dvec3& pos_vec, const arma::dmat33& rot_mat);

    // Get transformation matrix
    arma::dmat44 get_transformation_matrix(void) { return m_t_mat; }

    // Get rotation matrix
    arma::dmat33 get_rotation_matrix(void) { return m_rot_mat; }
    
    // Get position vector
    arma::dvec3 get_position_vector(void) {return m_pos_vec; }

public:
    // Set transformation matrix
    void set_transformation_matrix(const arma::dmat44& t_mat);

    // Set rotation matrix
    void set_rotation_matrix(const arma::dmat33& rot_mat);

    // Set position vector
    void set_position_vector(const arma::dvec3& pos_vec);

private:

    // Rotation matrix
    arma::dmat33 m_rot_mat = arma::eye(3, 3);

    // Position vector
    arma::dvec3 m_pos_vec = {0.0, 0.0, 0.0};

    // Transformation matrix
    arma::dmat44 m_t_mat = arma::eye(4, 4);
};

