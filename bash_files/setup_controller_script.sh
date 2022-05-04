# Cd to the parent(catkin_ws) directory
cd $HOME/catkin_ws_control

# Run catkin_make from catkin_ws
catkin_make

# Source workspace setup file
source $HOME/catkin_ws_control/devel/setup.bash

# rosrun axia_ft_processing axia_ft_processing_node
rosrun image_processing image_processing.py
# rosrun setup_controller setup_controller_node
