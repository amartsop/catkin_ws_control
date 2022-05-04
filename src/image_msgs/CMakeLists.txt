cmake_minimum_required(VERSION 3.0.2)
project(image_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs
  sensor_msgs)

# Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES ImagesSynchronized.msg
CompressedImageCustom.msg ImagesSynchronizedCompressed.msg )

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy message_runtime std_msgs sensor_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

