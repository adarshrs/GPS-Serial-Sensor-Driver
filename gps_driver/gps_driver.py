#!/usr/bin/env python 

import rospy 
import serial 
from math import sin, pi 
from std_msgs.msg import Header
from Lab1.msg import gps
import utm
 

if __name__ == '__main__': 

    rospy.init_node('gps_node') 
    serial_port = rospy.get_param('~port','/dev/ttyUSB0') 
    serial_baud = rospy.get_param('~baudrate',4800) 

    port = serial.Serial(serial_port, serial_baud, timeout=3.) 
    
    line = port.readline() 
    gps_pub = rospy.Publisher('/gps_data', gps, queue_size=5)

    gps_msg = gps()

    try: 
        while not rospy.is_shutdown(): 
            line = port.readline() 
            if line == '': 
                rospy.logwarn("No data") 
            else: 
                if line.startswith('$GPGGA'): 
                    gps_msg.raw_data = line
                    data = line.split(',')
                    print(data)
                    
                    # Header
                    header_msg = Header()
                    hh = float(data[1][:2])
                    mm = float(data[1][2:4])
                    ss = float(data[1][4:])
                    header_msg.stamp.secs = hh * 3600 + mm * 60 + ss
                    gps_msg.header = header_msg

                    # Latitude
                    if data[3] == 'N':
                        gps_msg.lat = float(data[2][:2])/60 + float(data[2][2:])/60
                        print(gps_msg.lat)
                    elif data[3] == 'S':
                        gps_msg.lat = -1 * (float(data[2][:2])/60 + float(data[2][2:])/60)

                    # Longitude
                    if data[5] == 'E':
                        gps_msg.lon = float(data[4][:2])/60 + float(data[4][2:])/60
                    elif data[5] == 'W':
                        gps_msg.lon = -1 * (float(data[4][:2])/60 + float(data[4][2:])/60)

                    # Altitude
                    gps_msg.alt = float(data[9])

                    (gps_msg.utm_easting, gps_msg.utm_northing, gps_msg.zone, gps_msg.letter) = utm.from_latlon(gps_msg.lat, gps_msg.lon)

                    gps_pub.publish(gps_msg)
                    
    except rospy.ROSInterruptException: 
        port.close() 

     
    except serial.serialutil.SerialException: 
        rospy.loginfo("Shutting down gps node...") 