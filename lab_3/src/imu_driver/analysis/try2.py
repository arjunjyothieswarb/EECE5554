import rosbag
import numpy as np
import allantools
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

        # self.accum_ax = []
        # self.accum_ay = []
        # self.accum_az = []

        # self.cumm_gx = []
        # self.cumm_gy = []
        # self.cumm_gz = []
        pass
    
    def pnt(self):
        print(self.header)
    
imu = imu_data()
cumm_ax = [0]
cumm_ay = [0]
cumm_az = [0]

cumm_gx = [0]
cumm_gy = [0]
cumm_gz = [0]

for topic, msg, t in bag.read_messages(topics=['/vectornav']):
    
    raw_data = msg.data.split(',')
    imu.header.append(msg.header)
    try:
        gx = float(raw_data[10])
        gy = float(raw_data[11])
        gz = float(raw_data[12][:-5])

        ax = float(raw_data[7])
        ay = float(raw_data[8])
        az = float(raw_data[9])
    except Exception as e:
        print(e)
        continue
    imu.gyro_x.append(gx)
    imu.gyro_y.append(gy)
    imu.gyro_z.append(gz)
    imu.accel_x.append(ax)
    imu.accel_y.append(ay)
    imu.accel_z.append(az)

    cumm_ax.append(cumm_ax[frame - 1] + imu.accel_x[frame])
    cumm_ay.append(cumm_ay[frame - 1] + imu.accel_y[frame])
    cumm_az.append(cumm_az[frame - 1] + imu.accel_z[frame])
    cumm_gx.append(cumm_gx[frame - 1] + imu.gyro_x[frame])
    cumm_gy.append(cumm_gy[frame - 1] + imu.gyro_y[frame])
    cumm_gz.append(cumm_gz[frame - 1] + imu.gyro_z[frame])

    frame = frame + 1

taus, ad, ade, ns = allantools.adev(data=np.array(imu.gyro_x),rate=freq,data_type="freq", taus="all")
plt.figure("X")
plt.loglog(taus, ade)

taus, ad, ade, ns = allantools.adev(data=np.array(imu.gyro_y),rate=freq,data_type="freq", taus="all")
plt.figure("Y")
plt.loglog(taus, ade)

taus, ad, ade, ns = allantools.adev(data=np.array(imu.gyro_z),rate=freq,data_type="freq", taus="all")
plt.figure("Z")
plt.loglog(taus, ade)


plt.show()

bag.close()
print(frame)