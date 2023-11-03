#!/usr/bin/python3

import rosbag
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PointStamped
import tf
from tf import transformations


class Pub_imu():
    def __init__(self):

        bag = rosbag.Bag("/home/bhanu/lab3/src/Data/group/2023-10-25-13-00-38.bag")
        
        rospy.init_node("IMU_parser")
        
        # Initializing the publisher and transform broadcaster
        pub = rospy.Publisher('imu', Imu, queue_size=10)
        br = tf.TransformBroadcaster()

        # Setting the rate at which the IMU data is published
        rate = rospy.Rate(40)
        
        IMU = Imu()
        quat = Quaternion()

        flag = 0
        
        # Initializing the position and velocity
        curr_x = 0
        curr_y = 0
        curr_z = 0

        prev_velx = 0
        prev_vely = 0
        prev_velz = 0

        # Initializing gravity vector
        g = PointStamped()

        bias_vector = Vector3()

        for topic, msg, t in bag.read_messages(topics=['/imu_data']):  # Reading messages from bag file
            if rospy.is_shutdown():  # Checking for shutdown
                exit()
            if not flag:
                # Getting the first time stamp
                self.prev_time = msg.Header.stamp.secs + (msg.Header.stamp.nsecs * (10**-9))
                flag = 1
            
            self.curr_time = msg.Header.stamp.secs + (msg.Header.stamp.nsecs * (10**-9))
            
            IMU = msg.IMU

            t = self.get_time() # Getting the sample time

            g = self.get_bias(msg.IMU.orientation) # Getting the gravity vector represented w.r.t to "IMU1_Frame"

            # Getting the current position and update the previous velocity
            curr_x, prev_velx = self.get_pos(msg.IMU.linear_acceleration.x - g.point.x, prev_velx, t)
            curr_y, prev_vely = self.get_pos(msg.IMU.linear_acceleration.y - g.point.y, prev_vely, t)
            curr_z, prev_velz = self.get_pos(msg.IMU.linear_acceleration.z + g.point.z, prev_velz, t)
            
            IMU.header.stamp = rospy.Time.now() # Getting the current time stamp

            x = IMU.orientation.x
            y = IMU.orientation.y
            z = IMU.orientation.z
            w = IMU.orientation.w

            # Broadcasting the tf between "IMU1_Frame" frame and "world" frame
            br.sendTransform((curr_x, curr_y, curr_z), (x,y,z,w), rospy.Time.now(), "IMU1_Frame", "world")

            # Publishing the IMU message at predetermined rate
            pub.publish(IMU)
            rate.sleep()

        bag.close()

        pass


    def get_bias(self, quat):
        
        bias = PointStamped()
        g = 9.82
        
        # Series of operations to obtain the transformation matrix mapping vector from "world" frame to "IMU1_Frame" frame
        euler = transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        rot_mat = transformations.euler_matrix(euler[0], euler[1], euler[2])
        inv_mat = transformations.inverse_matrix(rot_mat)
        
        # Gravity vector represented w.r.t "IMU1_Frame"
        bias.point.x = inv_mat[0][2] * g
        bias.point.y = inv_mat[1][2] * g
        bias.point.z = inv_mat[2][2] * g
        
        return bias
    

    def get_time(self):
        # The function returns the time difference between the current sample and the previous sample
        # and updates the prev_time variable
        
        t = self.curr_time - self.prev_time
        self.prev_time = self.curr_time  
        return t


    def get_pos(self, acc, prev_v, t):
        # The arg acc takes in the acceleration, the arg prev_v takes in the previous velocity and
        # the arg t takes the time and returns the final position of the frame

        # S = ut + 0.5(at^2)
        dist = (prev_v * t) + (0.5* acc * (t**2))
        # v = u + at
        curr_v = prev_v + (acc * t)
        return dist, curr_v



if __name__ == '__main__':
    Pub_imu()