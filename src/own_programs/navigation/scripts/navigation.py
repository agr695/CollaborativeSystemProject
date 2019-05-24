#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, TwistStamped
import rospy
from inspec_msg.msg import line_control_info
from mavros_msgs.msg import PositionTarget
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class navigation():
    def __init__(self):
        rospy.init_node('navigation')
        self.relative_x = 0
        self.relative_y = 0
        self.relative_z = 0
        self.relative_yaw = 0

        self.dt = 0.3

        self.local_pose = PoseStamped()     

        self.target_sub = rospy.Subscriber('/onboard/setpoint/inspect', line_control_info, self.get_target_data, queue_size=1)
        self.local_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_local_data, queue_size=1)
        self.command_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        # self.command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    def get_local_data(self, msg):
        self.local_pose = msg
        self.follow_line()

    def get_target_data(self, msg):
        self.relative_x = msg.x
        self.relative_y = msg.y
        self.relative_z = msg.z
        self.relative_yaw = msg.Yaw


    # def follow_line(self):
    #     target_pose = PositionTarget()
    #     target_pose.type_mask = 3057  # control pos_y, pos_z, vel_x, yaw
    #     target_pose.header.frame_id = "FRAME_BODY_NED"  # realtive to body frame
    #     target_pose.position.y = -self.relative_y
    #     target_pose.position.z = self.relative_z - 3
    #     target_pose.velocity.x = 0.5
    #     target_pose.yaw = self.relative_yaw
    #
    #     self.command_pub.publish(target_pose)

    def follow_line(self):
        target_pose = TwistStamped()
        target_pose.twist.linear.x = 0.3
        target_pose.twist.linear.y = -self.relative_y * self.dt
        target_pose.twist.linear.z = -(3 - self.relative_z) * self.dt
        target_pose.header.frame_id = "FRAME_BODY_OFFSET_NED"  # realtive to body frame

        # target_pose.velocity.y = 0.5
        #
        # target_pose.acceleration_or_force.z = 0.02
        # target_pose.acceleration_or_force.y = 9
        orientation_q = self.local_pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        target_pose.twist.angular.z = -self.relative_yaw * 0.3

        self.command_pub.publish(target_pose)

        
        


if __name__ == "__main__":
    nav = navigation()

    rospy.spin()