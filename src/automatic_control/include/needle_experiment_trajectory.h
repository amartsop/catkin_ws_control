#pragma once
#include <iostream>
#include "ros/ros.h"

#include <dynamics_math_eigen/dynamics_math_eigen.h>

class NeedleExperimentTrajectory
{

public:
    NeedleExperimentTrajectory();
    
    // Get desired ee pose
    dme::Cartesian get_desired_ee_pose(double time, double delta_time,
        const dme::Cartesian& cur_ee_pose);

    // Set initial end-effector pose
    void set_initial_ee_pose(const dme::Cartesian& ee_pose);

    // Get experiment duration
    double get_experiment_duration(void) { return m_tf; }

    // Reset experiment
    void reset(void) { m_tp = 0.0; }

public:
    // Set position amplitude
    void set_position_amplitude(const Eigen::Vector3d& a) { m_pos_a = a; }

    // Set position frequency
    void set_position_frequency(const Eigen::Vector3d& a) { m_pos_f = a; }

    // Set position phase
    void set_position_phase(const Eigen::Vector3d& a) { m_pos_p = a; }

    // Set rotation amplitude
    void set_rotation_amplitude(const Eigen::Vector3d& a) { m_rot_a = a; }

    // Set rotation frequency
    void set_rotation_frequency(const Eigen::Vector3d& a) { m_rot_f = a; }

    // Set rotation phase
    void set_rotation_phase(const Eigen::Vector3d& a) { m_rot_p = a; }

public:

    // Set centre point distance from ee
    void set_centre_point_distance_from_ee(const Eigen::Vector3d& rec_fe_fe)
    { m_rec_fe_fe = rec_fe_fe; }

private:

    // Initial end-effector pose 
    dme::Cartesian m_initial_ee_pose;

    // Desired ee pose
    dme::Cartesian m_desired_ee_pose;

    // Initial centre of mass pose
    dme::Cartesian m_initial_com_pose;

    // Initial theta and its derivatices
    Eigen::Vector3d m_theta_i, m_theta_dot_i, m_theta_ddot_i;

private:

    // Trajectory initial and final time 
    double m_t0 = 0.0; // Initial time (s) 
    double m_tf = 10.0; // Final time (s)

    // Trajectory sampling frequency
    double m_fs = 100.0;  // Sampling frequency (Hz)
    double m_ts = 1.0 / m_fs; // Sampling period (s)

    // Time from start 
    double m_time_from_start = 0.1;

    // Previous time
    double m_tp = 0.0;

private:
    // Calculate desired com pose
    dme::Cartesian calculate_desired_com_pose(double time);

    // Centre point distance from ee
    Eigen::Vector3d m_rec_fe_fe = {0.0604, 0.0, -0.04515};

private:
    // Position amplitude (m), frequency (Hz), phase (rad)
    Eigen::Vector3d m_pos_a = {0.0, 0.0, 0.0};
    Eigen::Vector3d m_pos_f = {0.5, 0.5, 0.5};
    Eigen::Vector3d m_pos_p = {0.0, 0.0, 0.0};

    // Rotation amplitude (m), frequency (Hz), phase (rad)
    Eigen::Vector3d m_rot_a = {0.0, 0.0, 0.0};
    Eigen::Vector3d m_rot_f = {0.5, 0.5, 0.5};
    Eigen::Vector3d m_rot_p = {0.0, 0.0, 0.0};

};