#!/bin/bash

##########################################

# Set the home position for the simulation. If not set the home position
# is located in Zurich. This home position is located as the takeoff area
# Odense Model airfield
# export PX4_HOME_LAT=55.471979
# export PX4_HOME_LON=10.414697

# Point near Seden Strand, near powerlines
export PX4_HOME_LAT=55.43620
export PX4_HOME_LON=10.46091

#if [[ $1 = "" ]]; then
# change this to a location of your firmware! must use V1.8.2 of PX4 Firmware
FIRMDIR=/home/*/src/Firmware
#else
#FIRMDIR=$1
#fi

echo $FIRMDIR
# source ros - remeber to change to your version, e.g. melodic/kinetic
source /opt/ros/melodic/setup.bash

# argument used to browse to your PX4 SITL firmware folder
cd $FIRMDIR

# Needed environment for running SITL
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# # launch basic PX4 SITL
#roslaunch px4 msc_posix.launch #&
roslaunch px4 msc_posix_lidar.launch #&
