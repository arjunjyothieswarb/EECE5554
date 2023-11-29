#!/usr/bin/python3

import rospy
import serial
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
from mult_driver.msg import imu_msg
from mult_driver.srv import *
import math

class IMU_Node():
    
    def __init__(self):
        
        rospy.init_node("IMU_Node")

        # Getting the port param
        port = rospy.get_param('/imu_port', '/dev/ttyUSB0')

        pub = rospy.Publisher("/imu", imu_msg, queue_size=100)
        
        # Initializing frame_id  for TF
        self.imu_data = imu_msg()
        self.imu_data.Header.frame_id = "IMU1_Frame"
        self.imu_data.IMU.header.frame_id = "IMU1_Frame"
        self.imu_data.MagField.header.frame_id = "IMU1_Frame"

        self.euler_angles = Vector3()

        # Initializing a serial object
        try:
            ser = serial.Serial(port, 115200, timeout=3)
        except:
            print("Error! Unable to open",port)
            exit()

        while(not rospy.is_shutdown()):

            raw_data = ser.readline().decode()
            self.imu_data.raw = raw_data

            raw_data = raw_data.split(',')

            self.imu_data.Header.stamp = rospy.Time.now()
            self.imu_data.IMU.header.stamp = self.imu_data.Header.stamp
            self.imu_data.MagField.header.stamp = self.imu_data.Header.stamp

            try:
                # Converting deg to rad
                self.euler_angles.x = float(raw_data[3]) * math.pi / 180
                self.euler_angles.y = float(raw_data[2]) * math.pi / 180
                self.euler_angles.z = float(raw_data[1]) * math.pi / 180
                
                self.imu_data.IMU.orientation = self.get_quaternions()

                # Converting string to float
                self.imu_data.IMU.linear_acceleration.x = float(raw_data[7])
                self.imu_data.IMU.linear_acceleration.y = float(raw_data[8])
                self.imu_data.IMU.linear_acceleration.z = float(raw_data[9])

                # Converting MagField from Gauss to Tesla
                self.imu_data.MagField.magnetic_field.x = float(raw_data[4]) / 10**4
                self.imu_data.MagField.magnetic_field.y = float(raw_data[5]) / 10**4
                self.imu_data.MagField.magnetic_field.z = float(raw_data[6]) / 10**4

                self.imu_data.IMU.angular_velocity.x = float(raw_data[10])
                self.imu_data.IMU.angular_velocity.y = float(raw_data[11])
                self.imu_data.IMU.angular_velocity.z = float(raw_data[12][:-5])

            except Exception as e:
                print("Try block failed!:")
                print(e)
                continue

            pub.publish(self.imu_data)
            pass
        
        ser.close()
        pass
    
    def get_quaternions(self):
        
        # Waiting for the service server
        rospy.wait_for_service("convert_to_quaternion")

        # Calling the service
        try:
            get_quat = rospy.ServiceProxy("convert_to_quaternion", convert_to_quaternion)
            quat = get_quat(self.euler_angles)
            return quat.quaternion
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        pass

    pass


if __name__ == '__main__':
    IMU_Node()
    pass