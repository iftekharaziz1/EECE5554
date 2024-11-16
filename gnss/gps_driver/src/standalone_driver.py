#!/usr/bin/env python3

import utm        
import serial     
import rospy
from std_msgs.msg import Header, String, Float64, UInt8
from gps_driver.msg import Customgps

def publish_gps_data():
    
    # Initialize the ROS node and publisher
    rospy.init_node('gps_data_publisher')
    gps_publisher = rospy.Publisher('/gps', Customgps, queue_size=10)
    rate = rospy.Rate(10)

    port_name = rospy.get_param('/port', '/dev/pts/9')      
    serial_conn = serial.Serial(port_name, 4800)
    gps_msg = Customgps()

    while not rospy.is_shutdown():
        
        nmea_line = serial_conn.readline().decode('utf-8')
        print(nmea_line)
 
        if "GPGGA" in nmea_line: 
            data_parts = nmea_line.split(",")
            print(data_parts)

            # Extract UTC time
            utc_time = float(data_parts[1]) 

            utc_hours = int(utc_time / 10000)
            utc_minutes = int((utc_time % 10000) / 100)
            utc_seconds = int(utc_time % 100)
 
            timestamp_seconds = utc_hours * 3600 + utc_minutes * 60 + utc_seconds
             
            lat_value = float(data_parts[2]) / 100
            if data_parts[3] == "S":
                lat_value *= -1     # South is negative
            
             
            lon_value = float(data_parts[4]) / 100
            if data_parts[5] == "W":
                lon_value *= -1    # West is negative
             
            hdop_measure = float(data_parts[8])
            altitude_measure = float(data_parts[9])
 
            utm_coords = utm.from_latlon(lat_value, lon_value)

            # Extract UTM components
            utm_east = utm_coords[0]
            utm_north = utm_coords[1]
            utm_zone = utm_coords[2]
            utm_letter = utm_coords[3]

           # Store required variables to message file
            gps_msg.Header.stamp.secs = timestamp_seconds
            gps_msg.Header.frame_id = "GPS1_Frame"
            gps_msg.Latitude = lat_value
            gps_msg.Longitude = lon_value
            gps_msg.Altitude = altitude_measure
            gps_msg.HDOP = hdop_measure
            gps_msg.UTM_easting = utm_east
            gps_msg.UTM_northing = utm_north
            gps_msg.UTC = utc_time
            gps_msg.Zone = utm_zone
            gps_msg.Letter = utm_letter

            # Publish the GPS message
            gps_publisher.publish(gps_msg)

if __name__ == '__main__':
    try:
        publish_gps_data()
    except rospy.ROSInterruptException:
        pass
