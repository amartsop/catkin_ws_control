#include "../include/automatic_control.h"

AutomaticControl::AutomaticControl(ros::NodeHandle *nh)
{
    /******************** System State Subscriber *******************/
    m_system_state_sub = nh->subscribe(m_system_state_topic_name, 1000, 
        &AutomaticControl::system_state_callback, this);
    
    // Wait to subscribe to topic
    while(!m_system_state_sub_pending_start) {
        ROS_INFO("Waiting to subscribe to the system's state topic");
    };

    ROS_INFO("Succesfully subscribed to the system's state topic");

    /******************* End-Effector State Topic Subscriber *****************/
    // Subscribe to ee state topic    
    m_ee_state_sub = nh->subscribe(m_ee_state_topic_name, 1000,
        &AutomaticControl::ee_state_callback, this);

    // Wait to subscribe to ee state topic
    while(!m_ee_state_sub_flag) {
        ROS_INFO("Waiting to subscribe to end-effector state topic");
    };

    ROS_INFO("Succesfully subscribed to end-effector state topic");

    /******* Needle Vibration Experiment Parameters Topic Subscriber ********/
    // Subscribe to nve topic
    m_nve_sub = nh->subscribe(m_nve_params_topic_name, 1000,
        &AutomaticControl::nve_prameters_callback, this);

    // Wait to subscribe to nve topic
    while(!m_nve_sub_flag) {
        ROS_INFO("Waiting to subscribe to nve topic");
    };

    ROS_INFO("Succesfully subscribed to nve topic");

    /*********** Desired End-Effector Pose Topic Publisher *************/
    // Desired end effector pose timer
    m_des_ee_pose_timer = nh->createTimer(
        ros::Duration(m_des_ee_pose_pub_interval),
        &AutomaticControl::publish_autonomous_desired_ee_pose, this);

    // Publisher handle for the desired end-effector pose
    m_lwa4p_des_ee_pose_pub = nh->advertise<cartesian_msgs::CartesianGoal>(
        m_lwa4p_des_ee_pose_topic_name, 1000);

    // The rotation matrix of the the ft frame with respect to the fe frame 
    m_rot_ft_fe << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0;
}

// Update automatic control
void AutomaticControl::update(double delta_time)
{
    /************** Choose automatic control mode ***********************/
    // Needle experiment initialization    
    if (m_state_id == 7)
    {
       // Reset experiment time
       m_needle_experiment_time = 0.0; 

        // Reset pending start flag
        m_needle_experiment_pending_start = true;

        // Reset experiment
        m_needle_experiment_trajectory.reset();
    }
    // Needle experiment execution    
    else if (m_state_id == 8)
    {
        // Check if we are pending for a start    
        if (m_needle_experiment_pending_start)
        {
            // Store the initial time frame
            m_needle_experiment_initial_time = ros::Time::now();

            // Set initial end effector pose
            m_needle_experiment_trajectory.set_initial_ee_pose(m_ee_pose);

            // Set needle experiment duration
            m_needle_experiment_duration =
                m_needle_experiment_trajectory.get_experiment_duration();

            // Deactivate pending start
            m_needle_experiment_pending_start = false;
        
            // Set experiment trajectory properties
            m_needle_experiment_trajectory.set_position_amplitude(m_pos_a);
            m_needle_experiment_trajectory.set_position_frequency(m_pos_f);
            m_needle_experiment_trajectory.set_position_phase(m_pos_p);
            m_needle_experiment_trajectory.set_rotation_amplitude(m_rot_a);
            m_needle_experiment_trajectory.set_rotation_frequency(m_rot_f);
            m_needle_experiment_trajectory.set_rotation_phase(m_rot_p);
        }

        // Update experiment time
        m_needle_experiment_time += delta_time;
       
        if (m_needle_experiment_time <= m_needle_experiment_duration) 
        {
            // Calculate the position of the sensor's center of mass Ct wrt
            // to the fe frame expressed in the fe frame
            // m_rect_fe_fe =  m_ret_fe_fe + m_rot_ft_fe * m_rtct_ft_ft; 
            m_rect_fe_fe =  m_ret_fe_fe;

            m_needle_experiment_trajectory.set_centre_point_distance_from_ee(m_rect_fe_fe);

            // Get desired ee pose
            m_des_ee_pose = m_needle_experiment_trajectory.get_desired_ee_pose(
                m_needle_experiment_time, delta_time, m_ee_pose);
        }
    }
    
}

// System state callback function
void AutomaticControl::system_state_callback(const smch_msgs::StateConstPtr& msg)
{
    // Change subscribe flag
    m_system_state_sub_pending_start = true;

    m_state_id = msg->state.data;    
}


// Desired end-effector pose callback function
void AutomaticControl::publish_autonomous_desired_ee_pose(const
    ros::TimerEvent& event)
{
    // Calculate the time between two consecutive callbacks
    double delta_time = (event.current_real - event.last_real).toSec();

    // Update desired ee pose
    update(delta_time);

    /******************** Send message *******************/ 

    // Initialize cartesian goal message
    cartesian_msgs::CartesianGoal goal;

    // Set header
    goal.header.stamp.sec = ros::Time::now().toSec();
    goal.header.stamp.nsec = ros::Time::now().toNSec();

    // Set time from start    
    goal.time_from_start = m_des_ee_pose.time;
    
    // Set positions
    goal.rop_F_F.x = m_des_ee_pose.rop_F_F(0);
    goal.rop_F_F.y = m_des_ee_pose.rop_F_F(1);
    goal.rop_F_F.z = m_des_ee_pose.rop_F_F(2);
        
    // Set velocities
    goal.rop_F_F_dot.x = m_des_ee_pose.rop_F_F_dot(0);
    goal.rop_F_F_dot.y = m_des_ee_pose.rop_F_F_dot(1);
    goal.rop_F_F_dot.z = m_des_ee_pose.rop_F_F_dot(2);

    // Set accelerations
    goal.rop_F_F_ddot.x = m_des_ee_pose.rop_F_F_ddot(0);
    goal.rop_F_F_ddot.y = m_des_ee_pose.rop_F_F_ddot(1);
    goal.rop_F_F_ddot.z = m_des_ee_pose.rop_F_F_ddot(2);

    // Set euler
    goal.euler.x = m_des_ee_pose.euler(0);
    goal.euler.y = m_des_ee_pose.euler(1);
    goal.euler.z = m_des_ee_pose.euler(2);
    
    // Set rotational velocity
    goal.omega_f_F_F.x = m_des_ee_pose.w_f_F_F(0);
    goal.omega_f_F_F.y = m_des_ee_pose.w_f_F_F(1);
    goal.omega_f_F_F.z = m_des_ee_pose.w_f_F_F(2);

    // Set rotational acceleration
    goal.omega_f_F_F_dot.x = m_des_ee_pose.w_f_F_F_dot(0);
    goal.omega_f_F_F_dot.y = m_des_ee_pose.w_f_F_F_dot(1);
    goal.omega_f_F_F_dot.z = m_des_ee_pose.w_f_F_F_dot(2);

    // Send msg to publisher
    m_lwa4p_des_ee_pose_pub.publish(goal);
}

// End-effector state callback function
void AutomaticControl::ee_state_callback(const
    cartesian_msgs::CartesianPoseConstPtr& msg)
{
    // Change the flag
    m_ee_state_sub_flag = true;

    // Set time stamp
    m_ee_pose.time = msg->header.stamp.sec;

    // Set end-effector position        
    m_ee_pose.rop_F_F(0) = msg->rop_F_F.x;
    m_ee_pose.rop_F_F(1) = msg->rop_F_F.y;
    m_ee_pose.rop_F_F(2) = msg->rop_F_F.z;
    
    // Set end-effector velocity        
    m_ee_pose.rop_F_F_dot(0) = msg->rop_F_F_dot.x;
    m_ee_pose.rop_F_F_dot(1) = msg->rop_F_F_dot.y;
    m_ee_pose.rop_F_F_dot(2) = msg->rop_F_F_dot.z;

    // Set end-effector acceleration        
    m_ee_pose.rop_F_F_ddot(0) = msg->rop_F_F_ddot.x;
    m_ee_pose.rop_F_F_ddot(1) = msg->rop_F_F_ddot.y;
    m_ee_pose.rop_F_F_ddot(2) = msg->rop_F_F_ddot.z;

    // Set end-effector euler        
    m_ee_pose.euler(0) = msg->euler.x;
    m_ee_pose.euler(1) = msg->euler.y;
    m_ee_pose.euler(2) = msg->euler.z;

    // Set end-effector rotational velocity        
    m_ee_pose.w_f_F_F(0) = msg->omega_f_F_F.x;
    m_ee_pose.w_f_F_F(1) = msg->omega_f_F_F.y;
    m_ee_pose.w_f_F_F(2) = msg->omega_f_F_F.z;

    // Set end-effector rotational acceleration        
    m_ee_pose.w_f_F_F_dot(0) = msg->omega_f_F_F_dot.x;
    m_ee_pose.w_f_F_F_dot(1) = msg->omega_f_F_F_dot.y;
    m_ee_pose.w_f_F_F_dot(2) = msg->omega_f_F_F_dot.z;
}


// Nve callback function
void AutomaticControl::nve_prameters_callback(const
    parameters_msgs::NeedleVibrationExperimentConstPtr& msg)
{
   // Set subscrition flag
    m_nve_sub_flag = true;

   // Set needle parameters 
    m_pos_a = {msg->translation_amplitude.x, msg->translation_amplitude.y,
        msg->translation_amplitude.z};

    m_pos_f = {msg->translation_frequency.x, msg->translation_frequency.y,
        msg->translation_frequency.z};

    m_pos_p = {msg->translation_phase.x, msg->translation_phase.y,
        msg->translation_phase.z};

    m_rot_a = {msg->rotation_amplitude.x, msg->rotation_amplitude.y,
        msg->rotation_amplitude.z};

    m_rot_f = {msg->rotation_frequency.x, msg->rotation_frequency.y,
        msg->rotation_frequency.z};

    m_rot_p = {msg->rotation_phase.x, msg->rotation_phase.y,
        msg->rotation_phase.z};

    // Set sensors com with respect to the reference frame
    m_rtct_ft_ft = {msg->rtct_ft_ft.x, msg->rtct_ft_ft.y, msg->rtct_ft_ft.z};
}