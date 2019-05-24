#!/bin/bash

#launch PX4_gazebo_simulation

bash px4_launch_gazebo.sh &
sleep 5

gnome-terminal --tab -- bash launch_mavROS.sh &
sleep 5

#gnome-terminal --tab -- rosrun keep_offboard offboard_mode.py &
#sleep 0.2

#gnome-terminal --tab -- rostopic pub -r 100000 /mavros/setpoint_position/local geometry_msgs/PoseStamped "header:
#  seq: 0
#  stamp:
#    secs: 0
#    nsecs: 0
#  frame_id: ''
#pose:
#  position:
#    x: 0.0
#    y: 0.0
#    z: 10.0
#  orientation:
#    x: 0.0
#    y: 0.0
#    z: 0.0
#    w: 0.0" 
#sleep 0.5

#rosservice call /mavros/cmd/arming "value: true"
#rosrun leddar_node simLeddar.py
#roslaunch estimator3d system.launch
#rostopic echo /onboard/setpoint/inspect 



