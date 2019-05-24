#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import rospy
from inspec_msg.msg import line_control_info
from mavros_msgs.msg import PositionTarget 

class navigation():
    def __init__(self):
        rospy.init_node('navigation')
        self.relative_x = 0
        self.relative_y = 0
        self.relative_z = 0
        self.relative_yaw = 0

        self.local_pose = PoseStamped()     

        self.target_sub = rospy.Subscriber('/onboard/setpoint/inspect', line_control_info, self.get_target_data)
        self.local_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_local_data)
        self.command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    def get_local_data(self, msg):
        self.local_pose = msg

    def get_target_data(self, msg):
        self.relative_x = msg.x
        self.relative_y = msg.y
        self.relative_z = msg.z
        self.relative_yaw = msg.Yaw

        self.follow_line()


    def follow_line(self):
        target_pose = PositionTarget()
        target_pose.position.y = self.local_pose.pose.position.y - self.relative_y
        target_pose.position.z = self.local_pose.pose.position.z - (3 - self.relative_z)

        target_pose.velocity.x = 0.5
        
        target_pose.yaw = self.relative_yaw

        self.command_pub.publish(target_pose)

        
        


if __name__ == "__main__":
    nav = navigation()

    rospy.spin()