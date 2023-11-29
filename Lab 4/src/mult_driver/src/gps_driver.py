#!/usr/bin/python3

import rospy
from mult_driver.msg import gps_msg
import serial
import utm
import numpy


class GPS_Node:

    def __init__(self):
        

        # Intializing the node
        rospy.init_node("GPS_Driver")

        # Reading the parameters
        port = rospy.get_param('/gps_port', '/dev/ttyUSB0')
        print("Listening to :" + port)
        
        # Setting up publisher and its rate
        pub = rospy.Publisher('gps', gps_msg, queue_size=100)
        rate = rospy.Rate(10)

        self.msg = gps_msg()
        self.msg.Header.frame_id = "GPS1_frame"
        # print(type(self.msg.Header.stamp.nsecs))

        self.flag = 0

        while(not rospy.is_shutdown()):
            
            try:
                # Reading data from serial buffer
                with serial.Serial(port, 4800) as ser:
                    line = ser.readline()
                    pass
            except:
                print("No signal")
                continue
            
            try:
                # To convert the byte to string
                line_data = line.decode()
                self.data = line_data.split(',')
                
                if (self.data[0] != '$GPGGA'):
                    continue
            
            except:
                continue

            try:
                # Extracting HDOP and Altitude
                self.msg.HDOP = float(self.data[8])
                self.msg.Altitude = float(self.data[9])
            except:
                print("No GPS Signal")
                continue
        

            # To parse UTC
            self.parse_UTC()

            # To get GPPGA time concurred
            self.get_GPGGA_secs()
            
            # To convert lat & lon in decimal degrees
            self.parse_latlon()

            # Convert lat & lon to UTM co-ordinates
            (self.msg.UTM_easting, self.msg.UTM_northing, self.msg.Zone, self.msg.Letter) = utm.from_latlon(self.msg.Latitude, self.msg.Longitude)
            
            pub.publish(self.msg)

            self.flag = 1

            # self.print_test()
            rate.sleep()

            pass
        
        pass


    def parse_UTC(self):
        
        hr = self.data[1][:2]
        min = self.data[1][2:4]
        sec = self.data[1][4:6]

        self.msg.UTC = hr + ":" + min + ":" + sec

        pass
    

    def parse_latlon(self):

        # Extracting the degrees
        self.msg.Latitude = float(self.data[2][:2])
        self.msg.Longitude = float(self.data[4][:3])

        # Extracting the decimal mins
        lat_mins = float(self.data[2][2:9]) / 60
        long_mins = float(self.data[4][3:10]) / 60

        # Getting the decimal degrees
        self.msg.Latitude = self.msg.Latitude + lat_mins
        self.msg.Longitude = self.msg.Longitude + long_mins

        lat_coeff = 1 if self.data[3] == 'N' else -1
        long_coeff = 1 if self.data[5] == 'E' else -1

        # Sign correction
        self.msg.Latitude = self.msg.Latitude * lat_coeff
        self.msg.Longitude = self.msg.Longitude * long_coeff

        pass

    def get_GPGGA_secs(self):

        # Extracting current time
        hr = int(self.data[1][:2])
        min = int(self.data[1][2:4])
        sec = int(self.data[1][4:6])
        ms = int(self.data[1][7:10])

        millis = (hr * 3600000) + (min * 60000) + (sec * 1000) + ms
        secs = numpy.uint32(millis/1000)
        nsecs = numpy.uint32((millis%1000) * (10**6))

        self.msg.Header.stamp.secs = numpy.uint32(secs)
        self.msg.Header.stamp.nsecs = numpy.uint32(nsecs)

        pass


    def print_test(self):
        
        # This is a funciton used to print all the parsed data from the GPS puck 
        # for testing purposes

        print("----------------------------------")
        print("Lat: ",self.msg.Latitude)
        print("Long: ", self.msg.Longitude)
        print("Altitude: ", self.msg.Altitude)
        print("HDOP: ", self.msg.HDOP)
        print("UTM_easting: ", self.msg.UTM_easting)
        print("UTM_northing: ", self.msg.UTM_northing)
        print("Zone number: ", self.msg.Zone)
        print("Zone Letter: ", self.msg.Letter)
        
        pass


if __name__ == '__main__':

    GPS_Node()
    pass