cmake_minimum_required(VERSION 2.8.3)
project(inaho_vehicle_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES inaho_vehicle_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

