# Collaborative System Project
Power line detection Collaborative System

**The packages inside the PowerLineThesis are taken from:**

	Oscar Bowen Schofield (osbow17@student.sdu.dk)
	Kasper Høj Lorenzen (kalor14@student.sdu.dk)
	
	https://github.com/OBSchofieldUK/PowerLineThesis

In order to run this project it is necessary to install the following dependecies:

    ROS: http://wiki.ros.org/kinetic/Installation/Ubuntu
    MavROS: https://dev.px4.io/en/ros/mavros_installation.html
    SMBus: https://pypi.org/project/smbus/
    Video-Stream-openCV: http://wiki.ros.org/video_stream_opencv
    RapidJson v1.1.0: http://rapidjson.org/

If the simulation and the navigation node are running in different computers, then it is necessary 
to run the following commands to connect both system in every terminal it is going to be used:

	export ROS_IP=192.168.43.115  # ip of your machine
	export ROS_MASTER_URI=http://192.168.43.15:11311  # http://ip of the master:11311

The file launch_mavROS.launch located in the package estimator3D need to be modified, the last line
has to be change to:

	 roslaunch mavros px4.launch fcu_url:="udp://:14540@IPSimulationComputer:14557"

If running the system using real sensor some lines needs to be uncommented:

**src/PowerLinesThesis/3D_estimator/launch/launch_system.launch**

	<!--<include file="$(find image_aquisation)/launch/Webcam.launch"/>-->
	<!--node name="ultrasonic" pkg="ultrasonic" type="ultrasonic.py" /-->
	<!--remap from="/iris/sonar_front/scan" to="/sonar_front/scan"/-->
	<!--remap from="/iris/sonar_left/scan" to="/sonar_left/scan"/-->

**src/PowerLinesThesis/3D_estimator/launch/launch_system.launch**

uncomment

	<!--include file="$(find lidar_matcher)/launch/Lidar.launch" /--> 
comment

	<include file="$(find lidar_matcher)/launch/prog.launch" />
	
**src/PowerLinesThesis/3D_estimator/launch/run_simulation.launch**

uncomment

	#roslaunch px4 msc_posix.launch #&
comment

	roslaunch px4 msc_posix_lidar.launch #&

**src/PowerLinesThesis/Image_processing/launch/prog.launch**

	<!--<remap from="/iris/camera/image_raw" to="/webcam/image_raw"/> -->
	
**src/own_programs/navigation/scripts/navigation.py**

uncomment (lines 29-30)
	
	# self.sonar_sub_front = rospy.Subscriber('/iris/sonar_front/scan', LaserScan, self.front_sonar, queue_size=1)
	# self.sonar_sub_left = rospy.Subscriber('/iris/sonar_left/scan', LaserScan, self.left_sonar, queue_size=1)

comment (lines 32-33)


	self.sonar_sub_front = rospy.Subscriber('/iris/sonar_front/scan', Int32, self.front_sonar, queue_size=1)
	self.sonar_sub_left = rospy.Subscriber('/iris/sonar_left/scan', Int32, self.left_sonar, queue_size=1)

uncomment (lines 65/76)

	# distance = msg.data/100.0
	
comment (lines 66/77)

	distance = min(msg.ranges)


To run the simulation and the complete system in the same computer it is necessary to follow this
steps:

	catkin build
	source devel/setup.bash
	cd src/PowerLineThesis/SimulationAssets/
	./installAssets.sh /home/user/src/Firmware
	roscore
	rosrun estimator3d run_simulation.sh 
	roslaunch estimator3d launch_system.launch
	rostopic pub -r 10000 /mavros/setpoint_position/local geometry_msgs/PoseStamped "header:
	  seq: 0
	  stamp:
	    secs: 0
	    nsecs: 0
	  frame_id: ''
	pose:
	  position:
	    x: 0.0
	    y: 0.0
	    z: 11.0
	  orientation:
	    x: 0.0
	    y: 0.0
	    z: 0.0
	    w: 0.0" 
	rosservice call /mavros/cmd/arming "value: true"
	
**Then stop the rostopic pub and the navigation should start**
	
