#!/usr/bin/env python


import rospy
import serial
import struct
import binascii
from geometry_msgs.msg import Twist
from xbee import ZigBee

xbee = None
XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x41\xB8\x93\x1E'
XBEE_ADDR_SHORT = '\x0C\x67'
DEVICE = '/dev/ttyUSB0'


def print_data(data):
    rospy.loginfo("XBee Response: %s" % data)

def prepare_data(msg):
    linear = 0
    angular = 0

    if msg.linear.x > 0:
        linear = 1
    elif msg.linear.x < 0:
        linear = 2

    if msg.angular.z > 0:
        angular = 2
    elif msg.angular.z < 0:
        angular = 1

    data = struct.pack('BBB', 1, linear, angular)
    return data

def cmd_vel_command(msg):

    data = prepare_data(msg)

    rospy.loginfo("Sending: %s" % binascii.hexlify(data))

    xbee.tx(
        dest_addr_long = XBEE_ADDR_LONG,
        dest_addr = XBEE_ADDR_SHORT,
        data=data,
    )

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    
    cmd_vel_command(msg)
    
    
def listener():
    global xbee

    ser = serial.Serial(DEVICE, 9600)
    xbee = ZigBee(ser, callback=print_data)

    rospy.init_node('cmd_vel_listener', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, callback)

    rospy.spin()

    xbee.halt()
    ser.close()
        
if __name__ == '__main__':
    listener()
