#!/usr/bin/env bash

source /opt/ros/melodic/setup.bash
# change to your location of catkinWS
source ~/catkin_ws/devel/setup.bash

roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.43.115:14557"
