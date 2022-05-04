#include "../include/nve_ft_processing.h"

NveFTProcessing::NveFTProcessing(ros::NodeHandle *nh)
{
    /******************* Axia Ft Data Subscriber *****************/
    // Subscribe to ee state topic    
    m_ft_data_sub = nh->subscribe(m_ft_sensor_topic_name, 1000,
        &NveFTProcessing::ft_sensor_callback, this);

    // // Wait to subscribe to ee state topic
    // while(!m_ft_data_sub_flag) {
    //     ROS_INFO("Waiting to subscribe to axia ft data topic");
    // };

    ROS_INFO("Succesfully subscribed to axia ft data topic");

    /******************* End-Effector State Topic Subscriber *****************/
    // Subscribe to ee state topic    
    m_ee_state_sub = nh->subscribe(m_ee_state_topic_name, 1000,
        &NveFTProcessing::ee_state_callback, this);

    // // Wait to subscribe to ee state topic
    // while(!m_ee_state_sub_flag) {
    //     ROS_INFO("Waiting to subscribe to end-effector state topic");
    // };

    ROS_INFO("Succesfully subscribed to end-effector state topic");

    /*********** Handle ft data topic publisher *************/
    // Handle ft data timer
    m_handle_ft_data_timer = nh->createTimer(
        ros::Duration(m_handle_ft_data_pub_interval),
        &NveFTProcessing::publish_handle_ft_data, this);

    // Publisher for ft data
    m_handle_ft_data_pub = nh->advertise<geometry_msgs::WrenchStamped>(
        m_handle_ft_data_topic_name, 1000);

    // The rotation matrix of the ft frame with respect to the fe frame 
    m_rot_ft_fe << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0;

    // The rotation matrix of the fm frame with respect to the ft frame 
    m_rot_fm_ft << 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0;

    // Gravity vector of support body
    m_wm_fr = {0.0, 0.0, - m_grav * m_mm};

    // Support inertia tensor
    m_im(0, 0) = 0.000057931;
    m_im(1, 1) = 0.00003074;
    m_im(2, 2) = 0.00003074;
}


// Publish axia ft data
void NveFTProcessing::publish_handle_ft_data(const ros::TimerEvent& event)
{
    // Get the state of the fm frame with respect to the robot's base
    dme::Cartesian fm_frame_state = dme::state_transformation(
        m_ee_state, m_rot_fm_fe, m_rem_fe_fe);
    
    // Get the rotation matrix of the fe with respect to the robot's base
    Eigen::Matrix3d rot_fe_fr = dme::EulerRotations::rotation(m_ee_state.euler);

    // Get the rotation matrix of the ft with respect to the robot's base
    Eigen::Matrix3d rot_ft_fr = rot_fe_fr * m_rot_ft_fe;

    // Get the rotation matrix of the fm with respect to the robot's base
    Eigen::Matrix3d rot_fm_fr = rot_fe_fr * m_rot_fm_fe;
    Eigen::Matrix3d rot_fr_fm = rot_fm_fr.transpose();
    
    // Get the sensor's forces expressed on the inertial frame    
    Eigen::Vector3d Ft_fr = rot_ft_fr * m_ft_data_ft.forces;

    // Get the sensor's moments expressed on the ft frame
    Eigen::Vector3d Mt_ft = m_ft_data_ft.moments;

    // Get the reaction forces and moments at point At
    Eigen::Vector3d Fat_fr = Ft_fr;
    Eigen::Vector3d Mat_ft = Mt_ft -
        dme::S(m_rtat_ft_ft) * rot_ft_fr.transpose() * Ft_fr;

    // Transfer moments to fm frame
    Eigen::Vector3d Mat_fm = m_rot_fm_ft * Mat_ft;

    // Get the inertial forces of the support body
    Eigen::Vector3d Fm_fr = m_mm * fm_frame_state.rop_F_F_ddot;

    // Get the rotational velocity of the frame fm with respect to the fr
    // expressed in fm
    Eigen::Vector3d w_fm_fr_fm = rot_fr_fm * fm_frame_state.w_f_F_F;

    // Get the rotational acceleration of the frame fm with respect to the fr
    // expressed in fm
    Eigen::Vector3d w_fm_fr_fm_dot = rot_fr_fm * fm_frame_state.w_f_F_F_dot;

    // Get the inertial moments of the support body
    Eigen::Vector3d Mm_fm = m_im * w_fm_fr_fm_dot +
        dme::S(w_fm_fr_fm) * m_im * w_fm_fr_fm;

    // Calculate the reaction forces at Am
    Eigen::Vector3d Fam_fr = Fat_fr + Fm_fr - m_wm_fr;
    

    // Calculate the reaction moments at Am
    Eigen::Vector3d Mam_fm = Mm_fm + Mat_fm - dme::S(m_rmam_fm_fm) * 
        rot_fr_fm * Fam_fr - dme::S(m_rmat_fm_fm) * (rot_fr_fm * Fat_fr);

    // Transfer forces to the fe frame
    Eigen::Vector3d Fam_fe = rot_fe_fr.transpose() * Fam_fr;
    Eigen::Vector3d Mam_fe = m_rot_fm_fe * Mam_fm;

    /************************* Publish handle data ***************************/
    geometry_msgs::WrenchStamped ft_data_msg;

    // Set message    
    ft_data_msg.wrench.force.x = Fam_fe(0);
    ft_data_msg.wrench.force.y = Fam_fe(1);
    ft_data_msg.wrench.force.z = Fam_fe(2);

    ft_data_msg.wrench.torque.x = Mam_fe(0);
    ft_data_msg.wrench.torque.y = Mam_fe(1);
    ft_data_msg.wrench.torque.z = Mam_fe(2);

    // // Set message    
    // Eigen::Vector3d forces_fe = m_rot_ft_fe * m_ft_data_ft.forces;
    // ft_data_msg.wrench.force.x = forces_fe(0);
    // ft_data_msg.wrench.force.y = forces_fe(1);
    // ft_data_msg.wrench.force.z = forces_fe(2);

    // Eigen::Vector3d moments_fe = m_rot_ft_fe * m_ft_data_ft.moments;
    // ft_data_msg.wrench.torque.x = moments_fe(0);
    // ft_data_msg.wrench.torque.y = moments_fe(1);
    // ft_data_msg.wrench.torque.z = moments_fe(2);
    
    // Send msg to publisher
    m_handle_ft_data_pub.publish(ft_data_msg);



}

// FT sensor callback function
void NveFTProcessing::ft_sensor_callback(const
    geometry_msgs::WrenchStampedConstPtr& msg)
{
    // Update flag
    m_ft_data_sub_flag = true;

    if (m_ft_pending_start)
    {
        // Get the state of the fm frame with respect to the robot's base
        dme::Cartesian fm_frame_state = dme::state_transformation(
            m_ee_state, m_rot_fm_fe, m_rem_fe_fe);

        // Get the inertial forces of the support body
        Eigen::Vector3d Fm_fr = m_mm * fm_frame_state.rop_F_F_ddot;
        
        // Calculate Fat_fr offset
        Eigen::Vector3d Fat_fr_offset = m_wm_fr - Fm_fr;

        // Get the rotation matrix of the fe with respect to the robot's base
        Eigen::Matrix3d rot_fe_fr = dme::EulerRotations::rotation(m_ee_state.euler);

        // Get the rotation matrix of the fm with respect to the robot's base
        Eigen::Matrix3d rot_fm_fr = rot_fe_fr * m_rot_fm_fe;
        Eigen::Matrix3d rot_fr_fm = rot_fm_fr.transpose();
        
        // Calculate Fat_fm offset
        Eigen::Vector3d Fat_fm_offset = rot_fr_fm * Fat_fr_offset; 

        // Get the rotational velocity of the frame fm with respect to the fr
        // expressed in fm
        Eigen::Vector3d w_fm_fr_fm = rot_fr_fm * fm_frame_state.w_f_F_F;

        // Get the rotational acceleration of the frame fm with respect to the fr
        // expressed in fm
        Eigen::Vector3d w_fm_fr_fm_dot = rot_fr_fm * fm_frame_state.w_f_F_F_dot;

        // Get the inertial moments of the support body
        Eigen::Vector3d Mm_fm = m_im * w_fm_fr_fm_dot +
            dme::S(w_fm_fr_fm) * m_im * w_fm_fr_fm;

        // Get reaction force at connection point At 
        Eigen::Vector3d Mat_fm_offset = dme::S(m_rmat_fm_fm) * Fat_fm_offset -
            Mm_fm;
    
        m_ft_support_offset.forces = m_rot_fm_ft * Fat_fm_offset;
        m_ft_support_offset.moments = m_rot_fm_ft * Mat_fm_offset;
    
        m_ft_pending_start = false;
    }

    // Set ft forces with respect to the ft frame
    m_ft_data_ft.forces = {msg->wrench.force.x, msg->wrench.force.y,
        msg->wrench.force.z};

    // Set ft moments with respect to the ft frame
    m_ft_data_ft.moments = {msg->wrench.torque.x, msg->wrench.torque.y,
        msg->wrench.torque.z};

    // Apply offsets
    m_ft_data_ft.forces = m_ft_data_ft.forces + m_ft_support_offset.forces;
    m_ft_data_ft.moments = m_ft_data_ft.moments + m_ft_support_offset.moments;
}

// End-effector state callback function
void NveFTProcessing::ee_state_callback(const
    cartesian_msgs::CartesianPoseConstPtr& msg)
{
    // Change the flag
    m_ee_state_sub_flag = true;

    // Set time stamp
    m_ee_state.time = msg->header.stamp.sec;

    // Set end-effector position        
    m_ee_state.rop_F_F = {msg->rop_F_F.x, msg->rop_F_F.y, msg->rop_F_F.z};
    
    // Set end-effector velocity        
    m_ee_state.rop_F_F_dot = {msg->rop_F_F_dot.x, msg->rop_F_F_dot.y,
        msg->rop_F_F_dot.z};

    // Set end-effector acceleration        
    m_ee_state.rop_F_F_ddot = {msg->rop_F_F_ddot.x, msg->rop_F_F_ddot.y,
        msg->rop_F_F_ddot.z};

    // Set end-effector euler        
    m_ee_state.euler = {msg->euler.x, msg->euler.y, msg->euler.z};

    // Set end-effector rotational velocity        
    m_ee_state.w_f_F_F = { msg->omega_f_F_F.x, msg->omega_f_F_F.y,
        msg->omega_f_F_F.z};

    // Set end-effector rotational acceleration        
    m_ee_state.w_f_F_F_dot = {msg->omega_f_F_F_dot.x, msg->omega_f_F_F_dot.y,
        msg->omega_f_F_F_dot.z};
}