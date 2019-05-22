#!/usr/bin/python

# import rospy
# import std_msgs.msg
import smbus
import time

class UltrasonicNode:
    def __init__(self, bus=1, address=0x70, reg_mode=0x00, reg_high_byte=0x02, reg_low_byte=0x03):
        self.bus = bus = smbus.SMBus(bus)
        self.device_address = address
        self.device_reg_mode = reg_mode
        self.device_reg_high_byte = reg_high_byte
        self.device_reg_low_byte = reg_low_byte

    def SetMode(self, mode):
        self.bus.write_byte_data(self.device_address, self.device_reg_mode, mode)
        time.sleep(0.07)

    def ReadData(self):
        high = self.bus.read_word_data(self.device_address, self.device_reg_high_byte)
        low = self.bus.read_word_data(self.device_address, self.device_reg_low_byte)

        ll = int(bin(low & 0b1111111), 2)
        lh = int(bin(low >> 15), 2)
        hl = int(bin(high&0b11),2)

        d = hl * 255 + lh * 128 + ll
        return d

    def GetMinRange(self):
        high = self.bus.read_word_data(self.device_address, 0x04)
        low = self.bus.read_word_data(self.device_address, 0x05)

        ll = int(bin(low & 0b1111111), 2)
        lh = int(bin(low >> 15), 2)
        hl = int(bin(high & 0b11), 2)

        d = hl * 255 + lh * 128 + ll
        return d

if __name__ == '__main__':
    ultrasonic = UltrasonicNode()
    while True:
        ultrasonic.SetMode(0x51)
        print ultrasonic.ReadData()
        print ultrasonic.GetMinRange()
        time.sleep(1)