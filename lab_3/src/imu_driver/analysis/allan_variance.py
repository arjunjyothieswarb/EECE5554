#!/usr/bin/python3

import rosbag
import numpy as np
import math
from std_msgs.msg import Header
from imu_driver.msg import trail
import matplotlib.pyplot as plt

bag = rosbag.Bag("/home/bhanu/lab3/src/Data/individual/LocationD.bag")
# bag = rosbag.Bag("/home/bhanu/lab3/src/Data/individual/trial.bag")

freq = 40
batch_size = 50000

frame = 0

class imu_data():

    def __init__(self):
        self.header = []
        
        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []

        self.accel_x = []
        self.accel_y = []
        self.accel_z = []

        self.accum_ax = []
        self.accum_ay = []
        self.accum_az = []

        self.cumm_gx = []
        self.cumm_gy = []
        self.cumm_gz = []
        pass
    
    def pnt(self):
        print(self.header)


def get_var(omega_cum, freq, M):

    L = len(omega_cum)
    max_m = 2*np.floor(np.log2(L/2))
    m = np.logspace(np.log10(1), np.log10(max_m), M)
    m = np.ceil(m)
    m = np.unique(m)

    t0 = 1/freq
    avar = []
    tau = []
    
    for i in m: 
        i1 = int(i)
        tau.append(i1*t0)
        sum = 0
        for k in range(L - 2*i1):
            # print(k + 2*i1)
            sum = sum + ((omega_cum[k + 2*i1] + 2*omega_cum[k + i1] + omega_cum[k]) * t0) ** 2
        con = 2 * ((i1*t0)**2) * (L - 2*i1)
        avar.append(math.sqrt(sum / con))

    return avar, tau


imu = imu_data()
cumm_ax = [0]
cumm_ay = [0]
cumm_az = [0]

cumm_gx = [0]
cumm_gy = [0]
cumm_gz = [0]

for topic, msg, t in bag.read_messages(topics=['/vectornav']):
    
    try:
        raw_data = msg.data.split(',')
        imu.header.append(msg.header)

        imu.gyro_x.append(float(raw_data[10]))
        imu.gyro_y.append(float(raw_data[11]))
        imu.gyro_z.append(float(raw_data[12][:-5]))

        imu.accel_x.append(float(raw_data[7]))
        imu.accel_y.append(float(raw_data[8]))
        imu.accel_z.append(float(raw_data[9]))
    
        cumm_ax.append(cumm_ax[frame - 1] + imu.accel_x[frame])
        cumm_ay.append(cumm_ay[frame - 1] + imu.accel_y[frame])
        cumm_az.append(cumm_az[frame - 1] + imu.accel_z[frame])

        cumm_gx.append(cumm_gx[frame - 1] + imu.gyro_x[frame])
        cumm_gy.append(cumm_gy[frame - 1] + imu.gyro_y[frame])
        cumm_gz.append(cumm_gz[frame - 1] + imu.gyro_z[frame])

        frame = frame + 1
    except Exception as e:
        print(e)

bag.close()
print(frame)

avar_gx, tau_gx = get_var(cumm_gx, freq, batch_size)
plt.figure("X")
plt.loglog(tau_gx, avar_gx)

avar_gy, tau_gy = get_var(cumm_gy, freq, batch_size)
plt.figure("Y")
plt.loglog(tau_gy, avar_gy)

avar_gz, tau_gz = get_var(cumm_gz, freq, batch_size)
plt.figure("Z")
plt.loglog(tau_gz, avar_gz)

plt.show()



