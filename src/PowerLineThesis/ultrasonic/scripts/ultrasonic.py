#!/usr/bin/python

import rospy
from std_msgs.msg import Int32
import smbus
import time

class UltrasonicNode:
    def __init__(self, bus=1, address=0x70, reg_mode=0x00, reg_high_byte=0x02, reg_low_byte=0x03, topic_name="/sonar_front/scan"):
        rospy.init_node('UltrasonicNode')

        self.distance_pub = rospy.Publisher(topic_name, Int32, queue_size=1)

        self.bus = bus = smbus.SMBus(bus)
        self.device_address = address
        self.device_reg_mode = reg_mode
        self.device_reg_high_byte = reg_high_byte
        self.device_reg_low_byte = reg_low_byte

    def SetMode(self, mode):
        self.bus.write_byte_data(self.device_address, self.device_reg_mode, mode)

    def ReadData(self):
        high = self.bus.read_word_data(self.device_address, self.device_reg_high_byte)
        low = self.bus.read_word_data(self.device_address, self.device_reg_low_byte)

        ll = int(bin(low & 0b1111111), 2)
        lh = int(bin(low >> 15), 2)
        hl = int(bin(high&0b11),2)

        distance = hl * 255 + lh * 128 + ll
        d = Int32()
        d.data = distance
        self.distance_pub.publish(d)
        return distance

    def GetMinRange(self):
        high = self.bus.read_word_data(self.device_address, 0x04)
        low = self.bus.read_word_data(self.device_address, 0x05)

        ll = int(bin(low & 0b1111111), 2)
        lh = int(bin(low >> 15), 2)
        hl = int(bin(high & 0b11), 2)

        d = hl * 255 + lh * 128 + ll
        return d


if __name__ == '__main__':
    ultrasonic_front = UltrasonicNode(address=0x70, topic_name="/sonar_front/scan")
    ultrasonic_left = UltrasonicNode(address=0x72, topic_name="/sonar_left/scan")
    rate_read = rospy.Rate(10)
    rate_execution = rospy.Rate(100)

    try:
        while not rospy.is_shutdown():
            ultrasonic_front.SetMode(0x51)
            ultrasonic_left.SetMode(0x51)
            print ultrasonic_front.ReadData()
            rate_read.sleep()
            print ultrasonic_left.ReadData()
            rate_execution.sleep()
    except KeyboardInterrupt:
        print 'shutting down'