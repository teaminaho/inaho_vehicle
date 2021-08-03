#!/bin/bash

. ~/catkin_ws/devel/setup.bash
roslaunch --no-summary --screen --wait joystick joystick_generic.launch
