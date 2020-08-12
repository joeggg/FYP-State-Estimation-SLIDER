#!/usr/bin/env python
import roslib, rospy
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Quaternion
import serial, time

port = serial.Serial('/dev/ttyACM2', 4000000)

def data_out():
    imu_data = Quaternion()
    while not rospy.is_shutdown():
        try:
            data1 = float(port.readline())
            data2 = float(port.readline())
            enc1_pub.publish(data1)
            enc2_pub.publish(data2)
        except ValueError:
            data1 = float(0)
            data2 = float(0)

if __name__ == '__main__':
    try:
        enc1_pub = rospy.Publisher('enc1_data', Float64, queue_size=1000)
        enc2_pub = rospy.Publisher('enc2_data', Float64, queue_size=1000)
        rospy.init_node('teensy_reader')
        data_out()
    except rospy.ROSInterruptException:
        pass
