#!/usr/bin/python3

import rospy
import serial
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
from imu_driver.msg import imu_msg
from imu_driver.srv import *
import math

class IMU_Node():
    
    def __init__(self):
        
        rospy.init_node("IMU_Node")

        port = rospy.get_param('/port', '/dev/ttyUSB0')

        pub = rospy.Publisher("/imu_data", imu_msg, queue_size=100)
        
        self.imu_data = imu_msg()
        self.imu_data.Header.frame_id = "IMU1_Frame"
        self.imu_data.IMU.header.frame_id = "IMU1_Frame"
        self.imu_data.MagField.header.frame_id = "IMU1_Frame"

        self.euler_angles = Vector3()
        self.prev_euler_angles = Vector3()
        
        self.prev_euler_angles.x = 0
        self.prev_euler_angles.y = 0
        self.prev_euler_angles.z = 0

        self.prev_stamp = rospy.Time.now()
        
        rate = rospy.Rate(100)

        try:
            ser = serial.Serial(port, 115200, timeout=3)
        except:
            print("Error! Unable to open",port)
            exit()
        
        # ser.write(b"VNWRG,75,1,40,05,0128,0100*XX")
        # ser.write(b"$VNRRG,8*4B")

        while(not rospy.is_shutdown()):

            # ser.write(b'$VNWRG,75,1,40,05,0128,0100*XX')
            # ser.write(b"$VNRRG,8*4B")

            raw_data = ser.readline().decode()
            self.imu_data.raw = raw_data
            # print(raw_data)

            raw_data = raw_data.split(',')
            # print(raw_data)

            self.imu_data.Header.stamp = rospy.Time.now()
            self.imu_data.IMU.header.stamp = rospy.Time.now()
            self.imu_data.MagField.header.stamp = rospy.Time.now()

            try:
                self.euler_angles.x = float(raw_data[3]) * math.pi / 180
                self.euler_angles.y = float(raw_data[2]) * math.pi / 180
                self.euler_angles.z = float(raw_data[1]) * math.pi / 180
                
                self.imu_data.IMU.orientation = self.get_quaternions()

                self.imu_data.IMU.linear_acceleration.x = float(raw_data[7])
                self.imu_data.IMU.linear_acceleration.y = float(raw_data[8])
                self.imu_data.IMU.linear_acceleration.z = float(raw_data[9])

                self.imu_data.MagField.magnetic_field.x = float(raw_data[4])
                self.imu_data.MagField.magnetic_field.y = float(raw_data[5])
                self.imu_data.MagField.magnetic_field.z = float(raw_data[6])

                self.imu_data.IMU.angular_velocity.x = float(raw_data[10])
                self.imu_data.IMU.angular_velocity.y = float(raw_data[11])
                self.imu_data.IMU.angular_velocity.z = float(raw_data[12][:-5])
                # print(raw_data[12])

            except Exception as e:
                print("Try block failed!:")
                print(e)
                continue

            # self.get_angular_velocity()
            pub.publish(self.imu_data)
            rate.sleep()
            pass
        
        ser.close()
        pass
    
    def get_quaternions(self):
        rospy.wait_for_service("convert_to_quaternion")

        try:
            get_quat = rospy.ServiceProxy("convert_to_quaternion", convert_to_quaternion)
            quat = get_quat(self.euler_angles)
            return quat.quaternion
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        pass

    # def get_angular_velocity(self):
        
    #     duration = (self.imu_data.Header.stamp.to_nsec() - self.prev_stamp.to_nsec()) * (10 ** -9)

    #     self.imu_data.IMU.angular_velocity.x = (self.euler_angles.x - self.prev_euler_angles.x)/duration
    #     self.imu_data.IMU.angular_velocity.y = (self.euler_angles.y - self.prev_euler_angles.y)/duration
    #     self.imu_data.IMU.angular_velocity.z = (self.euler_angles.z - self.prev_euler_angles.z)/duration

    #     self.prev_euler_angles.x = self.euler_angles.x
    #     self.prev_euler_angles.y = self.euler_angles.y
    #     self.prev_euler_angles.z = self.euler_angles.z
    #     self.prev_stamp = self.imu_data.Header.stamp
    #     pass
    pass


if __name__ == '__main__':
    IMU_Node()
    pass