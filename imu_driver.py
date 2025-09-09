#!/usr/bin/env python

import rospy
import serial
import utm
import math
import numpy as np
from imu_driver.msg import *

def get_quaternion_from_euler(roll, pitch, yaw):
    roll = (roll*np.pi)/180
    yaw = (yaw*np.pi)/180
    pitch = (pitch*np.pi)/180


    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
    return [qx, qy, qz, qw]

def main():
    rospy.init_node('imu_driver', anonymous=True)
    pub = rospy.Publisher('imu', imu_msg, queue_size=10)
    msg = imu_msg()
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    ser = serial.Serial(port, 115200, timeout=1)
    ser.write(b"$VNYMR,07,40*XX")
    
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode()
            line = line.lstrip('\r')
            data = line.split(',')

            if data[0] == '$VNYMR':
                msg.Header.stamp = rospy.Time.now()
                msg.Header.frame_id = 'IMU1_Frame'
                msg.Header.seq += 1
                #use euler
                msg.IMU.header.stamp = rospy.Time.now()
                msg.IMU.header.frame_id = 'IMU1_Frame'
                msg.IMU.header.seq +=1
                msg.IMU.orientation.x = get_quaternion_from_euler(float(data[3]), float(data[2]), float(data[1]))[0]
                msg.IMU.orientation.y = get_quaternion_from_euler(float(data[3]),float(data[2]),float(data[1]))[1]
                msg.IMU.orientation.z = get_quaternion_from_euler(float(data[3]),float(data[2]),float(data[1]))[2]
                msg.IMU.orientation.w = get_quaternion_from_euler(float(data[3]),float(data[2]),float(data[1]))[3]
                msg.IMU.angular_velocity.x = float(data[10])
                msg.IMU.angular_velocity.y = float(data[11])
                data[12] = data[12].split('*')
                msg.IMU.angular_velocity.z = float(data[12][0])
                msg.IMU.linear_acceleration.x = float(data[7])
                msg.IMU.linear_acceleration.y = float(data[8])
                msg.IMU.linear_acceleration.z = float(data[9])
                msg.MagField.header.stamp = rospy.Time.now()
                msg.MagField.header.frame_id = 'IMU1_Frame'
                msg.MagField.header.seq +=1
                msg.MagField.magnetic_field.x = float(data[4])
                msg.MagField.magnetic_field.y = float(data[5])
                msg.MagField.magnetic_field.z = float(data[6])
                rospy.loginfo(msg)
                pub.publish(msg)
              

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass