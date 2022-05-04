cmake_minimum_required(VERSION 3.0.2)
project(setup_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs
  geometry_msgs actionlib moveit_ros_planning_interface moveit_msgs
  cartesian_msgs control_msgs smch_msgs trajectory_msgs sensor_msgs
  parameters_msgs image_msgs cv_bridge)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs cartesian_msgs 
  geometry_msgs actionlib moveit_ros_planning_interface moveit_msgs
  control_msgs smch_msgs trajectory_msgs sensor_msgs parameters_msgs 
  image_msgs cv_bridge)

# # Boost linking
# find_package(Boost COMPONENTS filesystem system iostreams serialization REQUIRED)

# Armadillo linking
find_package(Armadillo REQUIRED)

# OpenCV
find_package( OpenCV REQUIRED )

# Dynamics math eigen linking
find_package(dynamics_math_eigen CONFIG REQUIRED)

# Dynamics math arma linking
find_package(dynamics_math_arma CONFIG REQUIRED)

#Link OPENGL library
cmake_policy(SET "CMP0072" OLD)
find_package(OpenGL REQUIRED)
set(OPENGL_LIBRARY_FULL
	${OPENGL_LIBRARY}
	-lGL -lGLU -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi
	${CMAKE_DL_LIBS})

# Add third party directory
add_subdirectory(libs)

#******************** Build ***********************#
# Source files variable
set(PROJ_SOURCE_FILES
  ./src/application_interface.cpp
  ./src/animation_interface.cpp
  ./src/ros_interface.cpp
  ./src/user_interface.cpp
  ./src/utils.cpp
  ./src/affine4d.cpp
  ./src/camera_window.cpp
  ./src/experiment_window.cpp
  ./src/control_panel_window.cpp
  ./src/lwa4p_forward_kinematics.cpp
  ./src/lwa4p_interface.cpp
  ./src/joystick_interface.cpp
  ./src/state_handler.cpp
  ./src/parameters_interface.cpp
  ./src/vision_interface.cpp
)

# Directories variable
set(PROJ_INCL_DIRS include
  /usr/local/include/eigen3
  /usr/include/opencv4/opencv2
  ${catkin_INCLUDE_DIRS})

set(PROJ_LIBS ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    ${OPENGL_LIBRARY_FULL}
    ${OpenCV_LIBS}
    third_party
    dme::dynamics_math_eigen
    dma::dynamics_math_arma)

# Include directories to the project
include_directories(${PROJ_INCL_DIRS} 
${ARMADILLO_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Add executables to target sync_test_node
add_executable(${PROJECT_NAME}_node src/setup_controller_node.cpp
  ${PROJ_SOURCE_FILES})

# Link libraries to target robot_manipulation_node
target_link_libraries(${PROJECT_NAME}_node ${PROJ_LIBS})

# Give priority to messages
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})