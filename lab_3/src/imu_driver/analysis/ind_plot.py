#!/usr/bin/python3

import rosbag
from imu_driver.msg import imu_msg
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt
import math

bag = rosbag.Bag("/home/bhanu/lab3/src/Data/individual/EXP_building.bag")

ax = []
ay = []
az = []

gx = []
gy = []
gz = []

magx = []
magy = []
magz = []

sec = []

frame = 0

def euler_from_quaternion(quat):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """

        w = quat.w
        x = quat.x
        y = quat.y
        z = quat.z

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        roll_x = roll_x * 180 / math.pi
        pitch_y = pitch_y * 180 / math.pi
        yaw_z = yaw_z * 180 / math.pi
     
        return roll_x, pitch_y, yaw_z # in degrees

def get_med(arr):
    arr.sort()
    return arr[int(len(arr)/2)]



for topic, msg, t in bag.read_messages(topics=['/imu_data']):
    
    if not frame:
        bias = msg.Header.stamp.secs + (msg.Header.stamp.nsecs * 10**-9)
        sec.append(0)
        pass
    else:
        sec.append(msg.Header.stamp.secs + (msg.Header.stamp.nsecs * 10**-9) - bias)
        # print('1')
    ax.append(msg.IMU.linear_acceleration.x)
    ay.append(msg.IMU.linear_acceleration.y)
    az.append(msg.IMU.linear_acceleration.z)

    magx.append(msg.MagField.magnetic_field.x)
    magy.append(msg.MagField.magnetic_field.y)
    magz.append(msg.MagField.magnetic_field.z)

    roll, pitch, yaw = euler_from_quaternion(msg.IMU.orientation)
    
    gx.append(roll)
    gy.append(pitch)
    gz.append(yaw)

    frame = frame + 1

print("Average accel_x: ", sum(ax)/frame)
print("Average accel_y: ", sum(ay)/frame)
print("Average accel_z: ", sum(az)/frame)

print("Average mag_x: ", sum(magx)/frame)
print("Average mag_y: ", sum(magy)/frame)
print("Average mag_z: ", sum(magz)/frame)

print("Average roll: ", sum(gx)/frame)
print("Average pitch: ", sum(gy)/frame)
print("Average yaw: ", sum(gz)/frame)

plt.figure("accel_x")
plt.plot(sec, ax)
plt.figure("accel_y")
plt.plot(sec, ay)
plt.figure("accel_z")
plt.plot(sec, az)

plt.figure("mag_x")
plt.plot(sec, magx)
plt.figure("mag_y")
plt.plot(sec, magy)
plt.figure("mag_z")
plt.plot(sec, magz)

plt.figure("roll")
plt.plot(sec, gx)
plt.figure("pitch")
plt.plot(sec, gy)
plt.figure("yaw")
plt.plot(sec, gz)

print("Median accel_x: ", get_med(ax))
print("Median accel_y: ", get_med(ay))
print("Median accel_z: ", get_med(az))

print("Median mag_x: ", get_med(magx))
print("Median mag_y: ", get_med(magy))
print("Median mag_z: ", get_med(magz))

print("Median roll: ", get_med(gx))
print("Median pitch: ", get_med(gy))
print("Median yaw: ", get_med(gz))

plt.show()

