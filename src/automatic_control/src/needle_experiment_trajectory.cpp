#include "../include/needle_experiment_trajectory.h"

NeedleExperimentTrajectory::NeedleExperimentTrajectory()
{
}

// Set initial end-effector pose
void NeedleExperimentTrajectory::set_initial_ee_pose(const dme::Cartesian& ee_pose)
{
    /*********************** Get initial pose data ************************/
    // Set initial ee pose
    m_initial_ee_pose = ee_pose;    

    // Transfer euler angles
    m_initial_com_pose.euler = m_initial_ee_pose.euler;

    // Transfer rotational velocity
    m_initial_com_pose.w_f_F_F = m_initial_ee_pose.w_f_F_F;

    // Transfer rotational acceleration
    m_initial_com_pose.w_f_F_F_dot = m_initial_ee_pose.w_f_F_F_dot;

    // Calculate rotation matrix
    Eigen::Matrix3d rot_fe_F = dme::EulerRotations::rotation(m_initial_ee_pose.euler);
    
    // Calculate G matrix
    Eigen::Matrix3d g_mat = dme::EulerRotations::G(m_initial_ee_pose.euler);

    // Calculate w_fe_F_fe
    Eigen::Vector3d w_fe_F_fe = rot_fe_F.transpose() * m_initial_ee_pose.w_f_F_F;
    
    // Calculate w_fe_F_fe_dot
    Eigen::Vector3d w_fe_F_fe_dot = rot_fe_F.transpose() * m_initial_ee_pose.w_f_F_F_dot;

    // Initial euler angles
    m_theta_i = m_initial_ee_pose.euler;

    // Calculate initial theta_dot 
    m_theta_dot_i = g_mat.householderQr().solve(w_fe_F_fe);

    // Calculate G_dot matrix
    Eigen::Matrix3d g_dot_mat = dme::EulerRotations::G_dot(m_theta_i, m_theta_dot_i);

    // Calculate initial theta_ee_ddot
    m_theta_ddot_i =
        g_mat.householderQr().solve(w_fe_F_fe_dot - g_dot_mat * m_theta_dot_i);
    
    // Calculate initial com position
    m_initial_com_pose.rop_F_F = m_initial_ee_pose.rop_F_F +
        rot_fe_F * m_rec_fe_fe;
    
    // Calculate initial com velocity
    m_initial_com_pose.rop_F_F_dot = m_initial_ee_pose.rop_F_F_dot +
        dme::S(m_initial_ee_pose.w_f_F_F) * rot_fe_F * m_rec_fe_fe; 

    // Calculate initial com acceleration
    m_initial_com_pose.rop_F_F_ddot = m_initial_ee_pose.rop_F_F_ddot +
        ( dme::S(m_initial_ee_pose.w_f_F_F_dot) +
        dme::S(m_initial_ee_pose.w_f_F_F) * dme::S(m_initial_ee_pose.w_f_F_F) ) *
        rot_fe_F * m_rec_fe_fe;
}

// Update trajectory
dme::Cartesian NeedleExperimentTrajectory::get_desired_ee_pose(double time,
    double delta_time, const dme::Cartesian& cur_ee_pose)
{
    // Calculate a new point every sampling period
    if (time - m_tp >= m_ts)
    {
        // Calculate desired com pose
        dme::Cartesian des_com_pose = calculate_desired_com_pose(time);

        // Calculate rotation matrix
        Eigen::Matrix3d rot_fe_F = dme::EulerRotations::rotation(des_com_pose.euler);

        // Set position
        m_desired_ee_pose.rop_F_F = des_com_pose.rop_F_F - rot_fe_F * m_rec_fe_fe;
    
        // Set velocity
        m_desired_ee_pose.rop_F_F_dot = des_com_pose.rop_F_F_dot - 
            dme::S(m_desired_ee_pose.w_f_F_F) * rot_fe_F * m_rec_fe_fe;

        // Set acceleration
        m_desired_ee_pose.rop_F_F_ddot = des_com_pose.rop_F_F_ddot - 
            (dme::S(m_desired_ee_pose.w_f_F_F_dot) +
            dme::S(m_desired_ee_pose.w_f_F_F) * dme::S(m_desired_ee_pose.w_f_F_F)) * 
            rot_fe_F * m_rec_fe_fe;

        // Set euler 
        m_desired_ee_pose.euler = des_com_pose.euler;

        // Set rotational velocity
        m_desired_ee_pose.w_f_F_F = des_com_pose.w_f_F_F;

        // Set rotational acceleration
        m_desired_ee_pose.w_f_F_F_dot = des_com_pose.w_f_F_F_dot;

        // Set time from start
        m_desired_ee_pose.time = m_time_from_start;
        
        // Update previous time
        m_tp = time;
    }

    // Return desired ee pose (state)
    return m_desired_ee_pose;
}

// Calculate desired com state
dme::Cartesian NeedleExperimentTrajectory::calculate_desired_com_pose(double time)
{
    // Initialize desired com pose
    dme::Cartesian des_com_pose;
    
    // Theta and it's derivatives
    Eigen::Vector3d des_theta_vec = {0.0, 0.0, 0.0};
    Eigen::Vector3d des_theta_dot_vec = {0.0, 0.0, 0.0};
    Eigen::Vector3d des_theta_ddot_vec = {0.0, 0.0, 0.0};

    for (int i = 0; i < 3; i++)
    {
        // Rigid body translational trajectory
        double a_dot = 2.0 * M_PI * m_pos_f(i);
        double a = a_dot * time + m_pos_p(i);

        // Position
        des_com_pose.rop_F_F(i) = m_pos_a(i) * sin(a) +
            m_initial_com_pose.rop_F_F(i); 
        
        // Velocity
        des_com_pose.rop_F_F_dot(i) = m_pos_a(i) * a_dot * cos(a) +
            m_initial_com_pose.rop_F_F_dot(i); 

        // Acceleration
        des_com_pose.rop_F_F_ddot(i) = - m_pos_a(i) * powf(a_dot, 2.0) * sin(a) +
            m_initial_com_pose.rop_F_F_ddot(i);

        // Rigid body rotational trajectory
        double b_dot = 2.0 * M_PI * m_rot_f(i);
        double b = b_dot * time + m_rot_p(i);
            
        // Theta vector
        des_theta_vec(i) = m_rot_a(i) * sin(b) + m_theta_i(i);
        
        // Theta dot vector
        des_theta_dot_vec(i) = m_rot_a(i) * b_dot * cos(b) + m_theta_dot_i(i);
                
        // Theta ddot vector
        des_theta_ddot_vec(i) = - m_rot_a(i) * powf(b_dot, 2.0) * sin(b) +
            m_theta_ddot_i(i);
    }

    /******************** Generate desired com pose ***********************/

    // Set euler angles
    des_com_pose.euler = des_theta_vec;
    
    // Calculate rotation matrix
    Eigen::Matrix3d rot_fe_F = dme::EulerRotations::rotation(des_theta_vec);

    // Calculate G matrix
    Eigen::Matrix3d g_mat = dme::EulerRotations::G(des_theta_vec);

    // Calculate G_dot matrix
    Eigen::Matrix3d g_dot_mat = dme::EulerRotations::G_dot(des_theta_vec, des_theta_dot_vec);

    // Set rotational velocity
    des_com_pose.w_f_F_F = rot_fe_F * g_mat * des_theta_dot_vec;

    // Set rotational acceleration
    des_com_pose.w_f_F_F_dot = rot_fe_F * (g_dot_mat * des_theta_dot_vec + 
        g_mat * des_theta_ddot_vec);

    return des_com_pose;
}