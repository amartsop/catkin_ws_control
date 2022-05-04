#include "../include/lwa4p_interface.h"

Lwa4pInterface::Lwa4pInterface(ros::NodeHandle *nh,
    std::shared_ptr<ControlPanelWindow> cp_window,
    JoystickInterface* joystick_interface)
{
    // Store control panel window ptr to member variable
    m_cp_window_ptr = cp_window;

    // Store joystick interface ptr to member variable
    m_joystick_interface = joystick_interface;

    /*********************** Joint State Topic Subscriber *****************/
    // Subscribe to joint angles topic    
    m_joint_state_sub = nh->subscribe(m_joint_state_topic_name, 1000,
        &Lwa4pInterface::joint_state_callback, this);

    // Wait to subscribe to joint state topic
    while(!m_joint_state_sub_flag) {
        ROS_INFO("Waiting to subscribe to joint state topic");
    };

    ROS_INFO("Succesfully subscribed to joint state topic");

    /******************* End-Effector State Topic Subscriber *****************/
    // Subscribe to ee state topic    
    m_ee_state_sub = nh->subscribe(m_ee_state_topic_name, 1000,
        &Lwa4pInterface::ee_state_callback, this);

    // Wait to subscribe to ee state topic
    while(!m_ee_state_sub_flag) {
        ROS_INFO("Waiting to subscribe to end-effector state topic");
    };

    ROS_INFO("Succesfully subscribed to end-effector state topic");

    /************* Desired End-Effector State Topic Subscriber *****************/

    // Subscribe to desired ee state subscriber
    m_des_ee_state_sub = nh->subscribe(m_des_ee_state_topic_name, 1000, 
        &Lwa4pInterface::des_ee_state_callback, this);

    /******************** Robot Arm Model *******************/
    // Load robot model
    m_robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");

    // Kinematic model
    robot_model::RobotModelPtr kinematic_model = m_robot_model_loader->getModel();

    // Kinematic state
    robot_state::RobotState kinematic_state(kinematic_model);

    // Create joint model group
    robot_state::JointModelGroup* joint_model_group =
        kinematic_model->getJointModelGroup("lwa4p_arm");
    
    // // Get kinematics base pointer
    m_kinematics_base_ptr = joint_model_group->getSolverInstance();
    
    // Set joint names
    m_joint_names = {"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint",
        "arm_5_joint", "arm_6_joint"};
    
    // Set link names
    m_link_names = joint_model_group->getLinkModelNames();

    // Get ee name
    m_ee_name = m_link_names.back();

    /****************** Desired Joint State Topic Publisher *****************/
    // Desired joint configuration timer
    m_des_joint_timer = nh->createTimer(
        ros::Duration(m_des_joint_traj_pub_interval), 
        &Lwa4pInterface::publish_desired_joint_trajectory, this);

    // Publisher handle for the desired joint state
    m_des_joint_pub = nh->advertise<control_msgs::FollowJointTrajectoryGoal>(
        m_des_joint_traj_topic_name, 1000);

}

// Update lwa4p interface
void Lwa4pInterface::update(int state)
{
    // State 0: Schunk Lwa4p manual joint control mode from GUI.
    if (state == 0)
    {
        // Pending start routine 
        if (m_pending_start.at(state))
        {
            std::fill(m_pending_start.begin(), m_pending_start.end(), true);
            m_pending_start.at(state) = false;
            m_cp_window_ptr->set_lwa4p_initial_desired_joint_angles(m_joint_angles);
        }

        // Get desired lwa4p joint angles
        std::vector<double> des_joint_angles =
            m_cp_window_ptr->get_lwa4p_desired_joint_angles();
        
        // Set desired joint position
        m_des_joint_state.q = Utils::std_dvec_to_eigen_dvec(des_joint_angles);
    
        // Set desired joint velocity
        m_des_joint_state.q_dot = Eigen::VectorXd::Zero(m_joints_num);
    
        // Set desired joint acceleration
        m_des_joint_state.q_ddot = Eigen::VectorXd::Zero(m_joints_num);

        // Set desired time (from start)
        m_des_joint_state.time = 0.3;
    }

    // State 1: Schunk Lwa4p manual end_effector control mode from GUI.
    else if (state == 1)
    {
        // Pending start routine 
        if (m_pending_start.at(state))
        {
            std::fill(m_pending_start.begin(), m_pending_start.end(), true);
            m_pending_start.at(state) = false;
            m_cp_window_ptr->set_lwa4p_initial_desired_ee_pose(m_ee_pose);
        }

        // Get desired lwa4p pose            
        std::vector<double> des_lwa4p_pose =
            m_cp_window_ptr->get_lwa4p_desired_ee_pose();

        // Initialize desired ee state
        dme::Cartesian des_ee_state;

        // Set position
        des_ee_state.rop_F_F = {des_lwa4p_pose.at(0), des_lwa4p_pose.at(1), 
            des_lwa4p_pose.at(2)};

        // Set orientation
        des_ee_state.euler = {des_lwa4p_pose.at(3), des_lwa4p_pose.at(4), 
            des_lwa4p_pose.at(5)};

        // Set time from start
        des_ee_state.time = 0.8;

        // Calulcate desired joint state with inverse kinematics
        m_des_joint_state = inverse_kinematics(des_ee_state);
    }

    // State 2: Schunk Lwa4p manual joint control mode from Joystick.
    else if (state == 2)
    {
        // Set pending start
        std::fill(m_pending_start.begin(), m_pending_start.end(), true);

        // Get joystick inputs
        auto joy_input = m_joystick_interface->get_joystick_values();

        // Get arrow vertical value
        int arrow_vertical = (int) joy_input["arrows_vertical"];

        // Update joint idx dummy
        m_idx_count_manual_joint_joy_dummy += arrow_vertical;

        // Update idx based on the vertical button
        if (std::abs(m_idx_count_manual_joint_joy_dummy) >= 
            m_idx_count_manual_joint_joy_dummy_thresh)
        {
            // Manual joint control joystick index count
            m_idx_count_manual_joint_joy += (double)
                Utils::sgn<double>(arrow_vertical);

            // Check if we've hit the limits
            if (m_idx_count_manual_joint_joy >= m_joint_state.q.rows() - 1)
            {
                m_idx_count_manual_joint_joy = m_joint_state.q.rows() - 1;
            }
            else if (m_idx_count_manual_joint_joy <= 0)
            {
                m_idx_count_manual_joint_joy = 0;
            }

            // Update dummy variable
            m_idx_count_manual_joint_joy_dummy = 0;
        }

        // Set desired joint angles
        m_des_joint_state.q(m_idx_count_manual_joint_joy) += 
            m_manual_joint_joy_step *
            (joy_input["button_R1"] - joy_input["button_R2"]);

        // Set desired joint velocity
        m_des_joint_state.q_dot = Eigen::VectorXd::Zero(m_joints_num);
    
        // Set desired joint acceleration
        m_des_joint_state.q_ddot = Eigen::VectorXd::Zero(m_joints_num);

        // Set desired time (from start)
        m_des_joint_state.time = 0.3;
    }

    // State 3: Schunk Lwa4p manual end_effector control mode from joystick.
    else if (state == 3)
    {
        // Set pending start
        std::fill(m_pending_start.begin(), m_pending_start.end(), true);

        // Get joystick inputs
        auto joy_input = m_joystick_interface->get_joystick_values();

        // Initialize desired lwa4p pose            
        std::vector<double> des_lwa4p_pose = m_ee_pose;

        // Set desired x position
        des_lwa4p_pose.at(0) += m_manual_ee_pos_joy_step *
            (joy_input["button_R1"] - joy_input["button_R2"]);

        // Set desired y position
        des_lwa4p_pose.at(1) += m_manual_ee_pos_joy_step * joy_input["arrows_horizontal"];

        // Set desired z position
        des_lwa4p_pose.at(2) += m_manual_ee_pos_joy_step * joy_input["arrows_vertical"];

        // Set desired roll position
        des_lwa4p_pose.at(3) += m_manual_ee_eul_joy_step *
            (joy_input["button_L1"] - joy_input["button_L2"]);

        // Set desired pitch position
        des_lwa4p_pose.at(4) += m_manual_ee_eul_joy_step * joy_input["right_joy_vertical"];
            
        // Set desired yaw position
        des_lwa4p_pose.at(5) += m_manual_ee_eul_joy_step * joy_input["right_joy_horizontal"];

        // Initialize desired ee state
        dme::Cartesian des_ee_state;

        // Set position
        des_ee_state.rop_F_F = {des_lwa4p_pose.at(0), des_lwa4p_pose.at(1), 
            des_lwa4p_pose.at(2)};

        // Set orientation
        des_ee_state.euler = {des_lwa4p_pose.at(3), des_lwa4p_pose.at(4), 
            des_lwa4p_pose.at(5)};

        // Set time from start
        des_ee_state.time = 0.8;

        // Calulcate desired joint state with inverse kinematics
        m_des_joint_state = inverse_kinematics(des_ee_state);
    }

    // State 4: Schunk Lwa4p manual tool control mode from joystick.
    else if (state == 4)
    {
        // Set pending start
        std::fill(m_pending_start.begin(), m_pending_start.end(), true);

        // Get joystick inputs
        auto joy_input = m_joystick_interface->get_joystick_values();

        // Initialize desired ee state
        dme::Cartesian des_ee_state;
    
        // Vector of current position
        Eigen::Vector3d pos = {m_ee_pose.at(0), m_ee_pose.at(1), 
            m_ee_pose.at(2)};
    
        // Vector of current euler values
        Eigen::Vector3d eul = {m_ee_pose.at(3), m_ee_pose.at(4), m_ee_pose.at(5)};
    
        // Current rotation matrix
        Eigen::Matrix3d rot = dme::EulerRotations::rotation(m_ee_pose.at(3), 
            m_ee_pose.at(4), m_ee_pose.at(5));

        // Position perturbation
        double dx = m_manual_ee_pos_joy_step *
            (joy_input["button_R1"] - joy_input["button_R2"]);
        
        double dy = m_manual_ee_pos_joy_step * joy_input["arrows_horizontal"];
        
        double dz = m_manual_ee_pos_joy_step * joy_input["arrows_vertical"];

        Eigen::Vector3d d_pos = {dx, dy, dz};

        // New position
        des_ee_state.rop_F_F = pos + rot * d_pos;

        // Orientation perturbation
        double d_phi = m_manual_ee_eul_joy_step *
            (joy_input["button_L1"] - joy_input["button_L2"]);

        double d_theta = m_manual_ee_eul_joy_step * joy_input["right_joy_vertical"];
        
        double d_psi = m_manual_ee_eul_joy_step * joy_input["right_joy_horizontal"];

        Eigen::Vector3d d_eul = {d_phi, d_theta, d_psi};

        // New orientation        
        des_ee_state.euler = eul + d_eul;

        // Set time from start
        des_ee_state.time = 0.8;        

        // Calculate desired joint angles
        m_des_joint_state = inverse_kinematics(des_ee_state);
    }

    // State 5: Schunk Lwa4p homing routine (home position)
    else if (state == 5)
    {
        // Set pending start
        std::fill(m_pending_start.begin(), m_pending_start.end(), true);

        // Set desired joint angles to home position
        m_des_joint_state.q = Utils::std_dvec_to_eigen_dvec(m_lwa4p_home_position);

        // Set desired joint velocity
        m_des_joint_state.q_dot = Eigen::VectorXd::Zero(m_joints_num);
    
        // Set desired joint acceleration
        m_des_joint_state.q_ddot = Eigen::VectorXd::Zero(m_joints_num);

        // Set desired time (from start)
        m_des_joint_state.time = 3.0;
    }

    // State 6: Schunk Lwa4p homing routine (zero position)
    else if (state == 6)
    {
        // Set pending start
        std::fill(m_pending_start.begin(), m_pending_start.end(), true);

        // Set desired joint angles to home position
        m_des_joint_state.q = Utils::std_dvec_to_eigen_dvec(m_lwa4p_zero_position);
    
        // Set desired joint velocity
        m_des_joint_state.q_dot = Eigen::VectorXd::Zero(m_joints_num);
    
        // Set desired joint acceleration
        m_des_joint_state.q_ddot = Eigen::VectorXd::Zero(m_joints_num);

        // Set desired time (from start)
        m_des_joint_state.time = 3.0;
    }

    // State 7: Needle experiment initialization.
    else if (state == 7)
    {
        // Set pending start
        std::fill(m_pending_start.begin(), m_pending_start.end(), true);

        // Set desired joint angles to home position
        m_des_joint_state.q = Utils::std_dvec_to_eigen_dvec(m_lwa4p_home_position);
    
        // Set desired joint velocity
        m_des_joint_state.q_dot = Eigen::VectorXd::Zero(m_joints_num);
    
        // Set desired joint acceleration
        m_des_joint_state.q_ddot = Eigen::VectorXd::Zero(m_joints_num);

        // Set desired time (from start)
        m_des_joint_state.time = 3.0;
    }

    // State 8: Needle experiment execution.
    else if (state == 8)
    {
        // Set pending start
        std::fill(m_pending_start.begin(), m_pending_start.end(), true);

        if (m_des_ee_state_sub_flag)
        {
            // Calculate desired joint angles
            m_des_joint_state = inverse_kinematics(m_des_ee_state);
        }
        else
        {
            // Set desired joint angles to home position
            m_des_joint_state.q = Utils::std_dvec_to_eigen_dvec(m_lwa4p_home_position);
    
            // Set desired joint velocity
            m_des_joint_state.q_dot = Eigen::VectorXd::Zero(m_joints_num);
    
            // Set desired joint acceleration
            m_des_joint_state.q_ddot = Eigen::VectorXd::Zero(m_joints_num);

            // Set desired time (from start)
            m_des_joint_state.time = 3.0;
        }
    



    }
}

// Joint state callback function
void Lwa4pInterface::joint_state_callback(const
    control_msgs::JointTrajectoryControllerStateConstPtr& msg)
{
    // Change the flag
    m_joint_state_sub_flag = true;

    // Get joint positions
    if (msg->actual.positions.size() > 0){
        m_joint_angles = msg->actual.positions;
        m_joint_state.q = Utils::std_dvec_to_eigen_dvec(m_joint_angles);
    }

    // Get joint velocities
    if (msg->actual.velocities.size() > 0){
        m_joint_state.q_dot =
            Utils::std_dvec_to_eigen_dvec(msg->actual.velocities);
    }

    // Get joint acceleration
    if (msg->actual.accelerations.size() > 0){
        m_joint_state.q_ddot =
            Utils::std_dvec_to_eigen_dvec(msg->actual.accelerations);
    }
}

// End-effector state callback function
void Lwa4pInterface::ee_state_callback(const
    cartesian_msgs::CartesianPoseConstPtr& msg)
{
    // Change the flag
    m_ee_state_sub_flag = true;

    /***********************  Set end-effector pose ***********************/
    m_ee_pose.at(0) = msg->rop_F_F.x;
    m_ee_pose.at(1) = msg->rop_F_F.y;
    m_ee_pose.at(2) = msg->rop_F_F.z;

    m_ee_pose.at(3) = msg->euler.x;
    m_ee_pose.at(4) = msg->euler.y;
    m_ee_pose.at(5) = msg->euler.z;

    /***********************  Set end-effector state ***********************/
    // Inertial position
    m_ee_state.rop_F_F = {msg->rop_F_F.x, msg->rop_F_F.y, msg->rop_F_F.z};
    
    // Inertial velocity
    m_ee_state.rop_F_F_dot = {msg->rop_F_F_dot.x, msg->rop_F_F_dot.y,
        msg->rop_F_F_dot.z};

    // Inertial acceleration
    m_ee_state.rop_F_F_ddot = {msg->rop_F_F_ddot.x, msg->rop_F_F_ddot.y,
        msg->rop_F_F_ddot.z};

    // Euler angles 
    m_ee_state.euler = {msg->euler.x, msg->euler.y, msg->euler.z};

    // Rotational velocity
    m_ee_state.w_f_F_F = {msg->omega_f_F_F.x, msg->omega_f_F_F.y,
        msg->omega_f_F_F.z};

    // Rotational acceleration
    m_ee_state.w_f_F_F_dot = {msg->omega_f_F_F_dot.x, msg->omega_f_F_F_dot.y,
        msg->omega_f_F_F_dot.z};
}

// Joint state callback function
void Lwa4pInterface::publish_desired_joint_trajectory(const ros::TimerEvent& event)
{
    // Initialize message
    control_msgs::FollowJointTrajectoryGoal goal;

    // Set header
    goal.trajectory.header.stamp.sec = ros::Time::now().sec;
    goal.trajectory.header.stamp.nsec = ros::Time::now().nsec;    

    // Set joint names 
    goal.trajectory.joint_names = m_joint_names;

    // Resize message
    goal.trajectory.points.resize(1);

    // Set position
    goal.trajectory.points[0].positions = 
        Utils::eigen_dvec_to_std_dvec(m_des_joint_state.q);
    
    // Set velocities
    goal.trajectory.points[0].velocities =
        Utils::eigen_dvec_to_std_dvec(m_des_joint_state.q_dot);

    // Set acceleration
    goal.trajectory.points[0].accelerations =
        Utils::eigen_dvec_to_std_dvec(m_des_joint_state.q_ddot);

    // Set time from start
    goal.trajectory.points[0].time_from_start =
        ros::Duration(m_des_joint_state.time);

    m_des_joint_pub.publish(goal);
}

// Lwa4p inverse kinematics routine
dme::Joint Lwa4pInterface::inverse_kinematics(const dme::Cartesian&
    des_ee_state)
{
    // Initialize desired joint state
    dme::Joint des_joint_state;

    // Set time duration
    des_joint_state.time = des_ee_state.time;

    /********************** Desired Joint Position ******************/
    // Geometry message 
    geometry_msgs::Pose cart_pose;
        
    // Set position
    cart_pose.position.x = des_ee_state.rop_F_F(0);
    cart_pose.position.y = des_ee_state.rop_F_F(1);
    cart_pose.position.z = des_ee_state.rop_F_F(2);

    // Set orientation
    dme::EulerRotations::Quaternions quatern =
    dme::EulerRotations::euler_to_quaternions(des_ee_state.euler(0), 
        des_ee_state.euler(1), des_ee_state.euler(2));

    cart_pose.orientation.w = quatern.w;
    cart_pose.orientation.x = quatern.x;
    cart_pose.orientation.y = quatern.y;
    cart_pose.orientation.z = quatern.z;
        
    // Initialize moveit error code
    moveit_msgs::MoveItErrorCodes error_code;

    // Initial guess 
    std::vector<double> joint_angles_guess =
        Utils::eigen_dvec_to_std_dvec(m_joint_state.q);
    
    // Initialize desired joint angles
    std::vector<double> des_joint_angles;
    
    // Solve inverse kinematics (desired_pose, initial_guess, calculated joint angles)
    bool found_ik = m_kinematics_base_ptr->getPositionIK(cart_pose,
        joint_angles_guess, des_joint_angles, error_code);
    
    // Now, we can print out the IK solution (if found):
    if (!found_ik)
    {
        // Error finding ik
        std::cout << "Can't find IK for Lwa4p!" << std::endl;
    }

    // Set desired joint angles
    des_joint_state.q = Utils::std_dvec_to_eigen_dvec(des_joint_angles);

    /********************** Desired Joint Velocity ******************/
    // Set to zero for now
    des_joint_state.q_dot = Eigen::VectorXd::Zero(des_joint_state.q.rows());

    /********************** Desired Joint Acceleration ******************/
    // Set to zero for now
    des_joint_state.q_ddot = Eigen::VectorXd::Zero(des_joint_state.q.rows());

    // Retun desired joint state
    return des_joint_state;
}

// Desired end-effector state callback function
void Lwa4pInterface::des_ee_state_callback(const
    cartesian_msgs::CartesianGoalConstPtr& msg)
{
    // Change subscription flag
    m_des_ee_state_sub_flag = true;

    // Get time from start
    m_des_ee_state.time = msg->time_from_start;

    // Get position
    m_des_ee_state.rop_F_F = {msg->rop_F_F.x, msg->rop_F_F.y, msg->rop_F_F.z};

    // Get velocity
    m_des_ee_state.rop_F_F_dot = {msg->rop_F_F_dot.x, msg->rop_F_F_dot.y,
        msg->rop_F_F_dot.z};

    // Get acceleration
    m_des_ee_state.rop_F_F_ddot = {msg->rop_F_F_ddot.x, msg->rop_F_F_ddot.y,
        msg->rop_F_F_ddot.z};

    // Get euler
    m_des_ee_state.euler = {msg->euler.x, msg->euler.y, msg->euler.z};

    // Get rotational velocity
    m_des_ee_state.w_f_F_F = {msg->omega_f_F_F.x, msg->omega_f_F_F.y, 
        msg->omega_f_F_F.z};

    // Get rotational acceleration
    m_des_ee_state.w_f_F_F_dot = {msg->omega_f_F_F_dot.x, msg->omega_f_F_F_dot.y, 
        msg->omega_f_F_F_dot.z};
}
