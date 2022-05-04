# Experimental setup with Schunk LWA4P (Schunk Powerball)

# Robot setup

### Gazebo Simulation:

1. Bring up gazebo
```
roslaunch lwa4p_gazebo lwa4p_sim.launch
```

2. Bring up moveit controllers
```
roslaunch lwa4p_moveit_config lwa4p_moveit_planning_execution.launch
```

3. Run robot manipulation node with
```
rosrun robot_manipulation robot_manipulation_node
```
or 
```
source run_script.sh
```

| WARNING: Use std::string traj_client_name = "arm_controller/follow_joint_trajectory"; |
| --- |

### Real Robot

<ins>**Method 1**</ins>

1. Establish CAN communication
```
sudo ip link set dev can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set dev can0 up
sudo ifconfig can0 txqueuelen 20
```

2. Bring up robot
```
roslaunch lwa4p_bringup lwa4p_bringup.launch
```

3. Initialize arm driver
```
rosservice call /arm/driver/init
```

On success:

4. Bring up moveit controllers
```
roslaunch lwa4p_moveit_config lwa4p_moveit_planning_execution.launch
```

5. Run robot manipulation node with
```
rosrun robot_manipulation robot_manipulation_node
```
or 
```
source run_script.sh
```

<ins>**Method 2**</ins>
1. Establish CAN communication
```
source can_connect.sh
```

2. Launch experiment
```
roslaunch experimental_setup experiment_launch.launch
```

| WARNING: Use std::string traj_client_name = "arm/arm_controller/follow_joint_trajectory"; |
| --- |

3. Run cameras
```
rosrun camera_control camera_control_main.py
```


### Common Errors

1. Cannot find device "can0"
```
    cd /opt/peak-linux-driver-8.13.0
    sudo make clean 
    sudo make NET=NETDEV_SUPPORT
    sudo make install
    reboot
```