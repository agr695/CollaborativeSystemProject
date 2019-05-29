#!/usr/bin/env python

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped

import rospy


class fetch_state():
    def __init__(self):
        rospy.init_node('keep_offboard')
        #self.state_msg = State()
        #rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, self.state)
        self.setMode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.state()


    def state(self):
        while True:
            # self.state_msg = data
            self.setMode(0,'OFFBOARD')

if __name__ == "__main__":
    set_state = fetch_state()
   
    # while True:
    #     set_state.state()
    #     #setMode(0,'OFFBOARD')

    #     rospy.sleep(1)

    rospy.spin()
        


