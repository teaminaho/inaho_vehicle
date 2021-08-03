#!/bin/bash -e

. /opt/ros/melodic/setup.bash
echo '. /opt/ros/melodic/setup.bash' >> ~/.bashrc
echo '. ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# create workspace & package
PACKAGE_NAME="mbed"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_create_pkg $PACKAGE_NAME
cd $PACKAGE_NAME
mv /app/launch .
mv /msgs ..

# catkin make
cd ~/catkin_ws
catkin_make
