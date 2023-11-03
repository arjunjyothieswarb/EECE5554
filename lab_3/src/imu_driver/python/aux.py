#!/usr/bin/python3

import rosbag
import rospy
from sensor_msgs.msg import Imu

class Pub_imu():
    def __init__(self):

        bag = rosbag.Bag("/home/bhanu/lab3/src/Data/group/2023-10-25-13-00-38.bag")
        
        rospy.init_node("IMU_parser")
        pub = rospy.Publisher('imu', Imu, queue_size=10)
        rate = rospy.Rate(40)
        IMU = Imu()

        for topic, msg, t in bag.read_messages(topics=['/imu_data']):
            IMU = msg.IMU
            IMU.header.stamp = rospy.Time.now()
            pub.publish(IMU)
            rate.sleep()

        bag.close()

        pass

if __name__ == '__main__':
    Pub_imu()