#!/usr/bin/env python

import rospy
import serial
import time
import utm
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header


def isGPGGAinString(inputString):
    return '$GPGGA' in inputString

def splitGPGGA(gpggaString):
    return gpggaString.split(",")
 
def degMinstoDegDec(LatOrLong):
    LatOrLong = float(LatOrLong)
    degree = int(LatOrLong / 100)   
    minutes = LatOrLong % 100       
    return degree + (minutes / 60)

 
def LatLongSignConvention(value, direction):
    return -value if direction in ['S', 'W'] else value

 
def UTCtoEpoch(UTC):
    hh = int(UTC[:2])
    mm = int(UTC[2:4])
    ss = float(UTC[4:])
     
    current_time = time.gmtime(time.time())
    time_sec = time.mktime((current_time.tm_year, current_time.tm_mon, current_time.tm_mday, hh, mm, int(ss), 0, 0, 0))
    
    return int(time_sec), int((time_sec - int(time_sec)) * 1e9)
 
def ReadFromSerial(serialPort):
    try:
        data = serialPort.readline().decode('ascii').strip()
        rospy.loginfo(f"Raw data read from serial: {data}")  # Log each line read
        return data
    except serial.SerialException as e:
        rospy.logerr(f"Serial read error: {e}")
        return None
    except UnicodeDecodeError as e:
        rospy.logwarn(f"Decode error: {e}")
        return None

# Main function
if __name__ == '__main__':
    rospy.init_node('gps_driver')

  
    serial_port = rospy.get_param('~port', '/dev/pts/7')
    serial_baud = rospy.get_param('~baudrate', 4800)
    gps_fix_topic = '/gps/fix'
 
    try:
        port = serial.Serial(serial_port, serial_baud, timeout=1.0)  
    except serial.SerialException as e:
        rospy.logerr(f"Error opening serial port {serial_port}: {e}")
        rospy.signal_shutdown("Unable to open serial port")
        exit(1)

    # ROS publisher for NavSatFix messages
    gps_pub = rospy.Publisher(gps_fix_topic, NavSatFix, queue_size=5)

    rospy.loginfo("GPS driver started, reading from serial port: " + serial_port)

    try:
        while not rospy.is_shutdown():
            gpggaRead = ReadFromSerial(port)

            if gpggaRead and isGPGGAinString(gpggaRead):
                gpggaSplit = splitGPGGA(gpggaRead)

                if len(gpggaSplit) > 5 and gpggaSplit[2] and gpggaSplit[4]:
                    # Extract fields
                    UTC = gpggaSplit[1]
                    Latitude = gpggaSplit[2]
                    LatitudeDir = gpggaSplit[3]
                    Longitude = gpggaSplit[4]
                    LongitudeDir = gpggaSplit[5]
                    Altitude = float(gpggaSplit[9]) if gpggaSplit[9] else 0.0

                    # Convert Latitude and Longitude to Decimal Degrees
                    LatitudeDec = degMinstoDegDec(Latitude)
                    LongitudeDec = degMinstoDegDec(Longitude)
 
                    LatitudeSigned = LatLongSignConvention(LatitudeDec, LatitudeDir)
                    LongitudeSigned = LatLongSignConvention(LongitudeDec, LongitudeDir)

                    # Convert UTC to epoch time
                    epoch_sec, epoch_nsec = UTCtoEpoch(UTC)

                    # Create NavSatFix message
                    navsat_msg = NavSatFix()
                    navsat_msg.header = Header()
                    navsat_msg.header.frame_id = 'gps'
                    navsat_msg.header.stamp = rospy.Time(epoch_sec, epoch_nsec)
                    navsat_msg.latitude = LatitudeSigned
                    navsat_msg.longitude = LongitudeSigned
                    navsat_msg.altitude = Altitude
                    navsat_msg.status.status = NavSatStatus.STATUS_FIX
                    navsat_msg.status.service = NavSatStatus.SERVICE_GPS

                    # Publish the NavSatFix message
                    gps_pub.publish(navsat_msg)
                else:
                    rospy.logwarn("Invalid GPGGA string or missing Latitude/Longitude fields: " + gpggaRead)

            
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down GPS driver...")
    except serial.SerialException as e:
        rospy.logerr("Serial error: " + str(e))
    except Exception as e:
        rospy.logerr("Error: " + str(e))
