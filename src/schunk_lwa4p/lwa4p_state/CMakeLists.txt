cmake_minimum_required(VERSION 3.0.2)
project(lwa4p_state)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs geometry_msgs actionlib moveit_ros_planning_interface
  moveit_msgs control_msgs cartesian_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs actionlib 
  moveit_ros_planning_interface moveit_msgs control_msgs cartesian_msgs)

# Dynamics math eigen linking
find_package(dynamics_math_eigen CONFIG REQUIRED)

#******************** Build ***********************#
# Source files variable
set(PROJ_SOURCE_FILES
  ./src/lwa4p_state_handler.cpp
  ./src/low_pass_filter_eigen.cpp
)

# Directories variable
set(PROJ_INCL_DIRS include
  /usr/local/include/eigen3
  ${catkin_INCLUDE_DIRS})

set(PROJ_LIBS ${catkin_LIBRARIES} dme::dynamics_math_eigen)

# Include directories to the project
include_directories(${PROJ_INCL_DIRS} ${Boost_INCLUDE_DIRS})

# Add executables to target sync_test_node
add_executable(${PROJECT_NAME}_node src/lwa4p_state_node.cpp
  ${PROJ_SOURCE_FILES})

# Link libraries to target robot_manipulation_node
target_link_libraries(${PROJECT_NAME}_node ${PROJ_LIBS})

# Give priority to messages
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})