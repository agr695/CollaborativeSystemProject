#!/usr/bin/env bash

source /opt/ros/melodic/setup.bash
# change to your location of catkinWS
source ~/catkin_ws/devel/setup.bash

roslaunch mavros px4.launch fcu_url:="udp://:14540@10.0.0.1:14557"
