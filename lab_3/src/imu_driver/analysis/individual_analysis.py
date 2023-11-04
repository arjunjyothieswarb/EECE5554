#!/usr/bin/python3

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import math

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
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

    roll = roll_x*180/math.pi
    pitch = pitch_y*180/math.pi
    yaw = yaw_z*180/math.pi

    return roll, pitch, yaw # in radians

def normalize(values, bias):
    norm = []
    for i in values:
        norm.append(i - bias)
    
    return norm

bag = rosbag.Bag("/home/bhanu/lab3/src/Data/individual/EXP_building_imu.bag")
# bag = rosbag.Bag("/home/bhanu/lab3/src/Data/group/group.bag")

time_list = []

accel_x = []
accel_y = []
accel_z = []

gyro_x = []
gyro_y = []
gyro_z = []

flag = 0

for topic, msg, t in bag.read_messages(topics=['/imu']):
    
    if not flag:
                # Getting the first time stamp
                time_zero = msg.Header.stamp.secs + (msg.Header.stamp.nsecs * (10**-9))
                flag = 1
    
    
    time_list.append(msg.Header.stamp.secs + (msg.Header.stamp.nsecs * (10**-9)) - time_zero)

    accel_x.append(msg.IMU.linear_acceleration.x)
    accel_y.append(msg.IMU.linear_acceleration.y)
    accel_z.append(msg.IMU.linear_acceleration.z)

    roll, pitch, yaw = euler_from_quaternion(msg.IMU.orientation.x, msg.IMU.orientation.y, msg.IMU.orientation.z, msg.IMU.orientation.w)
    
    gyro_x.append(roll)
    gyro_y.append(pitch)
    gyro_z.append(yaw)

print("Mean:")

print("Accel X: ", np.mean(accel_x))
print("Accel Y: ", np.mean(accel_y))
print("Accel Z: ", np.mean(accel_z))

print("Roll: ", np.mean(roll))
print("Pitch: ", np.mean(pitch))
print("Yaw: ", np.mean(yaw))


med_x = np.median(accel_x)
med_y = np.median(accel_y)
med_z = np.median(accel_z)

med_roll = np.median(roll)
med_pitch = np.median(pitch)
med_yaw = np.median(yaw)

print("Median:")

print("Accel X: ", med_x)
print("Accel Y: ", med_y)
print("Accel Z: ", med_z)

print("Roll: ", med_roll)
print("Pitch: ", med_pitch)
print("Yaw: ", med_yaw)

bin = 100

plt.figure("Plot", figsize=(18,12))

plt.subplot(231)
plt.title("Accel along X")
plt.xlabel("Time")
plt.ylabel("Accel along X(m/s^2)")
plt.plot(time_list, accel_x)

plt.subplot(232)
plt.title("Accel along Y")
plt.xlabel("Time")
plt.ylabel("Accel along Y(m/s^2)")
plt.plot(time_list, accel_y)

plt.subplot(233)
plt.title("Accel along Z")
plt.xlabel("Time")
plt.ylabel("Accel along Z(m/s^2)")
plt.plot(time_list, accel_z)

plt.subplot(234)
plt.title("Roll")
plt.xlabel("Time")
plt.ylabel("Roll in degrees")
plt.plot(time_list, gyro_x)

plt.subplot(235)
plt.title("Pitch")
plt.xlabel("Time")
plt.ylabel("Pitch in degrees")
plt.plot(time_list, gyro_y)

plt.subplot(236)
plt.title("Yaw")
plt.xlabel("Time")
plt.ylabel("Yaw in degrees")
plt.plot(time_list, gyro_z)

plt.figure("Histogram", (18,12))

plt.subplot(231)
plt.title("Accel X")
plt.xlabel("Error")
plt.ylabel("Frequency")
plt.hist(normalize(accel_x, med_x), bins=bin)

plt.subplot(232)
plt.title("Accel Y")
plt.xlabel("Error")
plt.ylabel("Frequency")
plt.hist(normalize(accel_y, med_y), bins=bin)

plt.subplot(233)
plt.title("Accel Z")
plt.xlabel("Error")
plt.ylabel("Frequency")
plt.hist(normalize(accel_z, med_z), bins=bin)

plt.subplot(234)
plt.title("Roll")
plt.xlabel("Error")
plt.ylabel("Frequency")
plt.hist(normalize(gyro_x, med_roll), bins=bin)

plt.subplot(235)
plt.title("Pitch")
plt.xlabel("Error")
plt.ylabel("Frequency")
plt.hist(normalize(gyro_y, med_pitch), bins=bin)

plt.subplot(236)
plt.title("Yaw")
plt.xlabel("Error")
plt.ylabel("Frequency")
plt.hist(normalize(gyro_z, med_yaw), bins=bin)

plt.show()