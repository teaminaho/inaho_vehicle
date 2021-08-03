#!/bin/bash -e
. ~/catkin_ws/devel/setup.bash
roslaunch --no-summary --screen --wait mbed serial_mbed.launch
