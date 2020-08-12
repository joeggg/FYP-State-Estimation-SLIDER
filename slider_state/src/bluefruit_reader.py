#!/usr/bin/env python
import roslib, rospy
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Imu
import serial, time

port = serial.Serial('/dev/ttyUSB2', 115200)

def data_out():
    imu_data = Imu()
    while not rospy.is_shutdown():
        try:
            read = port.readline()
            s = read.split()
            imu_data.orientation.x = float(s[0])
            imu_data.orientation.y = float(s[1])
            imu_data.orientation.z = float(s[2])
            imu_data.orientation.w = float(s[3])
            read = port.readline()
            s = read.split()
            imu_data.linear_acceleration.x = float(s[0])
            imu_data.linear_acceleration.y = float(s[1])
            imu_data.linear_acceleration.z = float(s[2])
            imu_pub.publish(imu_data)
        except ValueError:
            pass

if __name__ == '__main__':
    try:
        imu_pub = rospy.Publisher('imu_out', Imu, queue_size=1000)
        rospy.init_node('bluefruit_reader')
        data_out()
    except rospy.ROSInterruptException:
        pass
