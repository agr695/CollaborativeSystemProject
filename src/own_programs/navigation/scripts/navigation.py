#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, TwistStamped
import rospy
from inspec_msg.msg import line_control_info
from mavros_msgs.msg import PositionTarget
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from mavros_msgs.srv import SetMode

class navigation():
    def __init__(self):
        rospy.init_node('navigation')
        self.mode = "follow"

        self.relative_x = 0
        self.relative_y = 0
        self.relative_z = 0
        self.relative_yaw = 0

        self.dt = 0.3

        self.local_pose = PoseStamped()     

        self.sonar_sub_front = rospy.Subscriber('/iris/sonar_front/scan', LaserScan, self.front_sonar)
        self.sonar_sub_left = rospy.Subscriber('/iris/sonar_left/scan', LaserScan, self.left_sonar)

        self.setMode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.target_sub = rospy.Subscriber('/onboard/setpoint/inspect', line_control_info, self.get_target_data, queue_size=1)
        self.local_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_local_data, queue_size=1)
        self.command_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        # self.command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    def get_local_data(self, msg):
        self.local_pose = msg
        self.follow_line()
        self.setMode(0,'OFFBOARD')

    def get_target_data(self, msg):
        self.relative_x = msg.x
        self.relative_y = msg.y
        self.relative_z = msg.z
        self.relative_yaw = msg.Yaw

    def front_sonar(self, msg):
        distance = msg.ranges[0]

        if distance < 1:                    #we are close to tower so move to side
            self.mode = "avoid"
            pub_avoid_msg("side")
        elif self.mode == "avoid":          #we don't detect anything in front so we are in corner, move straight
            self.mode = "corner"
            pub_avoid_msg("straight")
    
    def left_sonar(self, msg):
        distance = msg.ranges[0]

        if distance < 5:                    #we are next to tower so go straight
            self.mode = "next_to_tower"     
        elif self.mode == "next_to_tower":  #we are past the tower so try to follow the line again
            self.mode = "follow"

        if self.mode == "corner":           
            self.pub_avoid_msg("straight")
        elif self.mode == "next_to_tower":
            self.pub_avoid_msg("straight")


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
    
    def pub_avoid_msg(self, direction):
        target_pose = TwistStamped()
        target_pose.header.frame_id = "FRAME_BODY_OFFSET_NED"
        if direction == "side":
            target_pose.twist.linear.y = -0.3
        elif direction == "straight":
            target_pose.twist.linear.x = 0.3

        self.command_pub.publish(target_pose)
        rospy.sleep(1)


    def follow_line(self):
        if self.mode == "follow":
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