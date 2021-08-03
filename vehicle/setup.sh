#!/bin/bash -e

source /opt/ros/noetic/setup.bash
echo '. /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo '. ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# create workspace & package
PACKAGE_NAME="vehicle"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_create_pkg $PACKAGE_NAME std_msgs rospy
cd $PACKAGE_NAME
mv /app/{launch,scripts} .
mv /msgs ..

# catkin make
cd ~/catkin_ws
catkin_make
