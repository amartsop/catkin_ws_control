#include "../include/axia_ft_processing.h"

AxiaFTProcessing::AxiaFTProcessing(ros::NodeHandle *nh)
{
    /********************* FT Sensor Subscriber *******************/
    // Subscribe to ft data topic
    m_ft_data_sub = nh->subscribe(m_ft_sensor_topic_name, 1000,
        &AxiaFTProcessing::ft_sensor_callback, this);

    // Wait to subscribe to ee state topic
    while(!m_ft_data_sub_flag) {
        ROS_INFO("Waiting to subscribe to FT data topic");
    };

    ROS_INFO("Succesfully subscribed to FT data topic");

    /******************* End-Effector State Topic Subscriber *****************/
    // Subscribe to ee state topic    
    m_ee_state_sub = nh->subscribe(m_ee_state_topic_name, 1000,
        &AxiaFTProcessing::ee_state_callback, this);

    // Wait to subscribe to ee state topic
    while(!m_ee_state_sub_flag) {
        ROS_INFO("Waiting to subscribe to end-effector state topic");
    };

    ROS_INFO("Succesfully subscribed to end-effector state topic");

    /*********** Axia ft data topic publisher *************/
    // Axia ft data timer
    m_axia_ft_data_timer = nh->createTimer(
        ros::Duration(m_axia_ft_data_pub_interval),
        &AxiaFTProcessing::publish_axia_ft_data, this);

    // Publisher handle for the axia ft data
    m_axia_ft_data_pub = nh->advertise<geometry_msgs::WrenchStamped>(
        m_axia_ft_data_topic_name, 1000);

    /********************** Filters ***********************/
    // Forces filter initialization
    m_forces_filter = std::make_shared<LowPassFilterEigen>(
        m_forces_filter_order, m_forces_filter_cutoff_freq,
        m_axia_ft_data_pub_frequency, 3, 1, false);

    // Momenmts filter initialization
    m_moments_filter = std::make_shared<LowPassFilterEigen>(
        m_moments_filter_order, m_moments_filter_cutoff_freq,
        m_axia_ft_data_pub_frequency, 3, 1, false);

    // The rotation matrix of the the ft frame with respect to the fe frame 
    m_rot_ft_fe << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0;

    double ixx = 0.000553; double ixy = 0.001; double ixz = .0005;
    double iyy = 0.001; double iyz = 0.001; double izz = 0.000037;

    m_sensor_inertia << ixx, ixy, ixz,
                        ixy, iyy, iyz, 
                        ixz, iyz, izz;
}

// FT sensor callback function
void AxiaFTProcessing::ft_sensor_callback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
    // Update flag
    m_ft_data_sub_flag = true;

    // Set raw ft forces 
    m_axia_raw_ft_data.forces = {msg->wrench.force.x, msg->wrench.force.y,
        msg->wrench.force.z};

    // Set raw ft moments
    m_axia_raw_ft_data.moments = {msg->wrench.torque.x, msg->wrench.torque.y,
        msg->wrench.torque.z};
}

// Publish axia ft data
void AxiaFTProcessing::publish_axia_ft_data(const ros::TimerEvent& event)
{
    /******************** Taring routine ***************************/
    if (m_taring_routine_counter < m_taring_routine_thresh)
    {
        // Bias forces
        m_ft_bias.forces += m_axia_raw_ft_data.forces;

        // Bias moments
        m_ft_bias.moments += m_axia_raw_ft_data.moments;
          
        // Update counter 
        m_taring_routine_counter++;
    }
    else
    {
        // Calculate bias forces and moments once 
        if (m_pending_start_flag)
        {

            /*************** Caclulate sensor's weight vector **************/
            // Get the state of the ft frame with respect to the robot's base
            dme::Cartesian ft_frame_state = dme::state_transformation(
                m_ee_state, m_rot_ft_fe, m_ret_fe_fe);

            // Get rotation matrix of the ft frame with respect to the inertial frame
            Eigen::Matrix3d rot_ft_fr = dme::EulerRotations::rotation(ft_frame_state.euler) ;

            // Weight vector 
            m_sensor_weight_fr = {0.0, 0.0, - m_sensor_mass * m_grav};

            // Calculate bias forces
            m_ft_bias.forces = m_ft_bias.forces / m_taring_routine_thresh;
            
            // Remove weight from bias
            m_ft_bias.forces = m_ft_bias.forces - rot_ft_fr.transpose() *
                m_sensor_weight_fr;

            // Calculate bias moments
            m_ft_bias.moments = m_ft_bias.moments / m_taring_routine_thresh;

            // Remove weight torque from bias
            m_ft_bias.moments = m_ft_bias.moments -
                dme::S(m_rtct_ft_ft) * rot_ft_fr.transpose() * m_sensor_weight_fr;

            // Set pending start flag
            m_pending_start_flag = false;
        }

        // Apply biases to raw measurements
        m_filtered_ft_data.forces = m_axia_raw_ft_data.forces - m_ft_bias.forces;
        m_filtered_ft_data.moments = m_axia_raw_ft_data.moments - m_ft_bias.moments;
    
        // Filter the data
        m_filtered_ft_data.forces = m_forces_filter->filt(m_filtered_ft_data.forces);
        m_filtered_ft_data.moments = m_moments_filter->filt(m_filtered_ft_data.moments);
    
        // Get the state of the ft frame with respect to the robot's base
        dme::Cartesian ft_frame_state = dme::state_transformation(
            m_ee_state, m_rot_ft_fe, m_ret_fe_fe);
    
        // Remove sensor's inertia
        m_filtered_ft_data = remove_sensor_inertia(ft_frame_state,
            m_filtered_ft_data);
    }

    /************************* Publish filtered data ***************************/
    geometry_msgs::WrenchStamped ft_data_msg;

    // Set timestamp
    ft_data_msg.header.stamp.sec = ros::Time::now().toSec();
    ft_data_msg.header.stamp.nsec = ros::Time::now().toNSec();

    // Set forces
    ft_data_msg.wrench.force.x = m_filtered_ft_data.forces(0);
    ft_data_msg.wrench.force.y = m_filtered_ft_data.forces(1);
    ft_data_msg.wrench.force.z = m_filtered_ft_data.forces(2);

    // Set torques
    // ft_data_msg.wrench.torque.x = m_filtered_ft_data.moments(0);
    // ft_data_msg.wrench.torque.y = m_filtered_ft_data.moments(1);
    // ft_data_msg.wrench.torque.z = m_filtered_ft_data.moments(2);

    ft_data_msg.wrench.torque.x = m_a(0);
    ft_data_msg.wrench.torque.y = m_a(1);
    ft_data_msg.wrench.torque.z = m_a(2);


    // Send msg to publisher
    m_axia_ft_data_pub.publish(ft_data_msg);
}

// End-effector state callback function
void AxiaFTProcessing::ee_state_callback(const
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

// Remove sensor inertia
dme::FTData AxiaFTProcessing::remove_sensor_inertia(const dme::Cartesian&
    ft_frame_state, const dme::FTData& net_ft_data)
{
    // Initialize FT Data
    dme::FTData ft_data;

    // Inertial/Coriolis FT data
    dme::FTData ft_inertial_coriolis;

    // Get the rotation matrix of the ft frame with respect to the fr frame
    Eigen::MatrixXd rot_ft_fr = dme::EulerRotations::rotation(ft_frame_state.euler);

    // Get the rotation matrix of the fr frame with respect to the ft frame
    Eigen::MatrixXd rot_fr_ft = rot_ft_fr.transpose();

    // Get the velocity of the ft frame wrt to fr expressed in the ft frame
    Eigen::Vector3d rrt_fr_ft_dot = rot_fr_ft * ft_frame_state.rop_F_F_dot;

    // Get the acceleration of the ft frame wrt to fr expressed in the ft frame
    Eigen::Vector3d rrt_fr_ft_ddot = rot_fr_ft * ft_frame_state.rop_F_F_ddot;

    // Get the rotational velocity of the ft frame wrt to fr expressed in the ft
    Eigen::Vector3d w_ft_fr_ft = rot_fr_ft * ft_frame_state.w_f_F_F;

    // Get the rotational acceleration of the ft frame wrt to fr expressed in the ft
    Eigen::Vector3d w_ft_fr_ft_dot = rot_fr_ft * ft_frame_state.w_f_F_F_dot;

    // Iniertial/Coriolis forces
    ft_inertial_coriolis.forces = m_sensor_mass * rrt_fr_ft_ddot + 
        dme::S(w_ft_fr_ft) * m_sensor_mass * rrt_fr_ft_dot;

    // m_a = rrt_fr_ft_dot;

    // Iniertial/Coriolis moments
    ft_inertial_coriolis.moments = m_sensor_inertia * w_ft_fr_ft_dot + 
        dme::S(w_ft_fr_ft) * m_sensor_inertia * w_ft_fr_ft;

    // Remove inertial forces 
    ft_data.forces = ft_inertial_coriolis.forces + net_ft_data.forces;

    // Remove sensor weight
    ft_data.forces = ft_data.forces - rot_fr_ft * m_sensor_weight_fr;

    // Remove inertial moments
    ft_data.moments = ft_inertial_coriolis.moments + net_ft_data.moments;

    // Remove sensor weight
    ft_data.moments = ft_data.moments -
        dme::S(m_rtct_ft_ft) * rot_ft_fr.transpose() * m_sensor_weight_fr;
    
    return ft_data; 
}