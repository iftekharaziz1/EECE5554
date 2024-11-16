#!/usr/bin/env python3

import utm
import rospy
import serial
from vn_driver.msg import Vectornav
import numpy as np
from math import pi

    
    #Convert Euler angles (in degrees) to a quaternion.
def convert_to_quaternion(deg_yaw, deg_pitch, deg_roll):

    yaw_rad = deg_yaw * pi / 180
    pitch_rad = deg_pitch * pi / 180
    roll_rad = deg_roll * pi / 180

    cy = np.cos(yaw_rad / 2)
    cp = np.cos(pitch_rad / 2)
    cr = np.cos(roll_rad / 2)
    sy = np.sin(yaw_rad / 2)
    sp = np.sin(pitch_rad / 2)
    sr = np.sin(roll_rad / 2)

    qw = (cr * cp * cy) + (sr * sp * sy)
    qx = (sr * cp * cy) - (cr * sp * sy)
    qy = (cr * sp * cy) + (sr * cp * sy)
    qz = (cr * cp * sy) - (sr * sp * cy)

    return qw, qx, qy, qz

#Read IMU data from a serial port, convert orientation data to quaternion, and publish it to the '/imu' topic.
def imudata_sender():

    pub = rospy.Publisher('/imu', Vectornav, queue_size=10)

    rospy.init_node('imudata_sender')
    rate = rospy.Rate(40)

    serial_port = rospy.get_param('/port', '/dev/ttyUSB0')

    ser = serial.Serial(serial_port, 115200)

    hertz = "$VNWRG,07,40*XX"
    data_output = "$VNWRG,06,14*XX"
    ser.write(hertz.encode('utf-8'))
    ser.write(data_output.encode('utf-8'))

    while not rospy.is_shutdown():
        raw_data = ser.readline().decode('utf-8')

        if "VNYMR" in raw_data:
            seg_data = raw_data.split(",")

            deg_yaw = float(seg_data[1])
            deg_pitch = float(seg_data[2])
            deg_roll = float(seg_data[3])
            mag_x = float(seg_data[4])
            mag_y = float(seg_data[5])
            mag_z = float(seg_data[6])
            acc_x = float(seg_data[7])
            acc_y = float(seg_data[8])
            acc_z = float(seg_data[9])
            ang_x = float(seg_data[10])
            ang_y = float(seg_data[11])
            ang_z = float((seg_data[12].split('*'))[0])

            qw, qx, qy, qz = convert_to_quaternion(deg_yaw, deg_pitch, deg_roll)

            msg = Vectornav()
            msg.header.stamp = rospy.get_rostime()
            msg.header.frame_id = "imu1_frame"

            msg.imu.orientation.w = qw
            msg.imu.orientation.x = qx
            msg.imu.orientation.y = qy
            msg.imu.orientation.z = qz

            msg.imu.angular_velocity.x = ang_x
            msg.imu.angular_velocity.y = ang_y
            msg.imu.angular_velocity.z = ang_z
            msg.imu.linear_acceleration.x = acc_x
            msg.imu.linear_acceleration.y = acc_y
            msg.imu.linear_acceleration.z = acc_z

            msg.mag_field.magnetic_field.x = mag_x
            msg.mag_field.magnetic_field.y = mag_y
            msg.mag_field.magnetic_field.z = mag_z

            msg.raw_data = raw_data

            rospy.loginfo(msg)
            pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        imudata_sender()
    except rospy.ROSInterruptException:
        pass
