#!/bin/bash -e

SCRIPT_DIR=$(cd $(dirname "$0"); pwd)

source /opt/ros/noetic/setup.bash
echo '. /opt/ros/noetic/setup.bash' >> $HOME/.bashrc
echo '. $HOME/catkin_ws/devel/setup.bash' >> $HOME/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# joystick
PACKAGE_NAME="joystick"
catkin_create_pkg $PACKAGE_NAME std_msgs sensor_msgs rospy
cd $PACKAGE_NAME
cp -r /app/{scripts,launch} .

# catkin make
cd ~/catkin_ws
catkin_make
