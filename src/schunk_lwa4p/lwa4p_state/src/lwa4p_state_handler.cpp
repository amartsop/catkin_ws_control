#include "../include/lwa4p_state_handler.h"

Lwa4pStateHandler::Lwa4pStateHandler(ros::NodeHandle *nh,
    robot_model::RobotModelPtr kinematic_model)
{
    // Set previous time 
    time_prev = ros::Time::now();

    // Subscribe to joint angles topic    
    m_joint_state_sub = nh->subscribe(m_arm_joint_state_topic, 1000,
        &Lwa4pStateHandler::joint_state_callback, this);

    // Wait to subscribe to topic
    while(!m_joint_state_sub_flag) {
        ROS_INFO("Waiting to subscribe to joint state topic");
    };

    ROS_INFO("Succesfully subscribed to joint state topic");

    // Kinematic state handle
    m_kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);
    m_kinematic_state->setToDefaultValues();

    // Create joint model group
    m_joint_model_group = kinematic_model->getJointModelGroup("lwa4p_arm");
    
    // Set joint and link names 
    m_joint_names = m_joint_model_group->getJointModelNames();
    m_link_names = m_joint_model_group->getLinkModelNames();

    // Get ee name
    m_ee_name = m_link_names.back();

    // Robot arm ee state publisher 
    m_ee_state_pub = nh->advertise<cartesian_msgs::CartesianPose>(
        m_ee_state_topic_name, 1000);

    /**************************** Filters ***************************/

    // Acceleration filter initialization
    m_ee_acceleration_filter = std::make_shared<LowPassFilterEigen>(
        m_ee_acceleration_filter_order, m_ee_acceleration_filter_cutoff_freq, 
        m_ee_pub_frequency, 6, 1, false);
}

void Lwa4pStateHandler::joint_state_callback(const
    control_msgs::JointTrajectoryControllerStateConstPtr& msg)
{
    // Change the flag
    m_joint_state_sub_flag = true;

    // Get joint positions
    if (msg->actual.positions.size() > 0){
        m_joint_positions = msg->actual.positions;
    }

    // Get joint velocities
    if (msg->actual.velocities.size() > 0){
        m_joint_velocities = msg->actual.velocities;
    }
    
    //Get joit accelerations
    if (msg->actual.accelerations.size() > 0){
        m_joint_accelerations = msg->actual.accelerations;
    }
}

// Publish state
void Lwa4pStateHandler::publish_ee_state(const ros::TimerEvent& event)
{
    // Convert joint positions eigen vector
    Eigen::VectorXd q = vec_to_eigen(m_joint_positions);

    // Convert joint velocities eigen vector
    Eigen::VectorXd q_dot = vec_to_eigen(m_joint_velocities);

    // Convert joint accelerations eigen vector
    Eigen::VectorXd q_ddot = vec_to_eigen(m_joint_accelerations);

    /*********************** Position ***********************/
    // Calculate forward kinematics
    m_kinematic_state->setJointGroupPositions(m_joint_model_group, q);

    // Get ee pose
    const Eigen::Affine3d& ee_pose =
        m_kinematic_state->getGlobalLinkTransform(m_ee_name);
    
    // Get ee position (m)
    Eigen::Vector3d pos = ee_pose.translation();
        
    // Get ee orientation 
    Eigen::Matrix3d rot = ee_pose.rotation();
    Eigen::Quaterniond quatern(rot);

    // Quaternions to euler angles 
    dme::EulerRotations::Euler eul_str =
        dme::EulerRotations::quaternions_to_euler(quatern.w(), quatern.x(),
        quatern.y(), quatern.z());

    // Convert euler angles to eigen
    Eigen::Vector3d eul(eul_str.phi, eul_str.theta, eul_str.psi);   

    /*********************** Velocity ***********************/
    // Find jacobian
    Eigen::MatrixXd jac = m_kinematic_state->getJacobian(m_joint_model_group);
    
    // Find the velocity vector
    Eigen::VectorXd ksi = jac * q_dot;
    
    // Find linear velocity 
    Eigen::Vector3d rop_F_F_dot = {ksi(0), ksi(1), ksi(2)};

    // Find angular velocity 
    Eigen::Vector3d omega_f_F_F = {ksi(3), ksi(4), ksi(5)};

    /*********************** Acceleration ***********************/
    // Update current time
    time_current = ros::Time::now();
    
    // Get time step
    double h = (event.current_real - event.last_real).toSec();
    
    // Get derivative of ksi  
    Eigen::VectorXd ksi_dot = (ksi - m_ksi_prev) / h;
    
    // Filter acceleration
    ksi_dot = m_ee_acceleration_filter->filt(ksi_dot);

    // Get derivative of linear velocity
    Eigen::Vector3d rop_F_F_ddot = {ksi_dot(0), ksi_dot(1), ksi_dot(2)};

    // Get derivative of rotational velocity
    Eigen::Vector3d omega_f_F_F_dot = {ksi_dot(3), ksi_dot(4), ksi_dot(5)};
    
    // Update ksi previous
    m_ksi_prev = ksi;
    
    /*********************** Publish ***********************/
    // Set time
    m_ee_state.header.stamp.sec = ros::Time::now().sec;
    m_ee_state.header.stamp.nsec = ros::Time::now().nsec;

    // Set positions
    m_ee_state.rop_F_F.x = pos(0);
    m_ee_state.rop_F_F.y = pos(1);
    m_ee_state.rop_F_F.z = pos(2);

    // Set euler angles
    m_ee_state.euler.x = eul(0);
    m_ee_state.euler.y = eul(1);
    m_ee_state.euler.z = eul(2);

    // Set linear velocities
    m_ee_state.rop_F_F_dot.x = rop_F_F_dot(0);
    m_ee_state.rop_F_F_dot.y = rop_F_F_dot(1);
    m_ee_state.rop_F_F_dot.z = rop_F_F_dot(2);

    // Set angular velocities
    m_ee_state.omega_f_F_F.x = omega_f_F_F(0);
    m_ee_state.omega_f_F_F.y = omega_f_F_F(1);
    m_ee_state.omega_f_F_F.z = omega_f_F_F(2);

    // Set linear accelerations
    m_ee_state.rop_F_F_ddot.x = rop_F_F_ddot(0);
    m_ee_state.rop_F_F_ddot.y = rop_F_F_ddot(1);
    m_ee_state.rop_F_F_ddot.z = rop_F_F_ddot(2);

    // Set angular accelerations
    m_ee_state.omega_f_F_F_dot.x = omega_f_F_F_dot(0);
    m_ee_state.omega_f_F_F_dot.y = omega_f_F_F_dot(1);
    m_ee_state.omega_f_F_F_dot.z = omega_f_F_F_dot(2);
    
    // Publish state
    m_ee_state_pub.publish(m_ee_state);
}

// Convert an std::vector<double> to Eigen::VectorXd
Eigen::VectorXd Lwa4pStateHandler::vec_to_eigen(const std::vector<double>& a)
{
    return Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(a.data(), a.size());
}