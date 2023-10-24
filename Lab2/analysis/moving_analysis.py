import rosbag
from std_msgs.msg import Header
import matplotlib.pyplot as plt
import time

# Open Vertices: 185, 380, 545, 730
# Occluded Vertices: 260, 390, 610


class msg:
    def __init__(self, header, easting, northing, sec):
        
        self.header = header
        self.easting = easting
        self.northing = northing
        self.sec = sec
        pass

class msg_plotter():
    def __init__(self):

        self.messages_open = []
        self.messages_occluded = []

        path = "/home/bhanu/catkin_ws/src/lab2/data/open_motion.bag"
        self.messages_open = self.read_val(path)
        pts_open = [185, 380, 545, 730]

        path2 = "/home/bhanu/catkin_ws/src/lab2/data/occluded_motion.bag"
        self.messages_occluded = self.read_val(path2)
        pts_occluded = [260, 390, 610, 735]

        self.plot_graph(self.messages_open, pts_open, "Open")
        self.plot_graph(self.messages_occluded, pts_occluded, "Occluded")
        plt.show()
        pass

    def read_val(self, path):
        
        bagread = rosbag.Bag(path)
        msg_list = []

        for topic, data, t in bagread.read_messages("/gps"):
            
            header = data.header
            easting = data.utm_easting
            northing = data.utm_northing
            sec = (float(data.utc[:2]) * 3600) + (float(data.utc[2:4]) * 60) + float(data.utc[4:6])

            msg_list.append(msg(header, easting, northing, sec))
            
        bagread.close()
        return msg_list

    def plot_graph(self, messages, pts, fig_name):

        Y = []
        X = []
        for i in messages:
            Y.append(i.northing)
            X.append(i.easting)

        segment1_x = X[:pts[0]]
        segment1_y = Y[:pts[0]]
        
        segment2_x = X[pts[0]:pts[1]]
        segment2_y = Y[pts[0]:pts[1]]
        
        segment3_x = X[pts[1]:pts[2]]
        segment3_y = Y[pts[1]:pts[2]]

        segment4_x = X[pts[2]:]
        segment4_y = Y[pts[2]:]

        plt.figure(fig_name)
        plt.xlabel("Easting in meters")
        plt.ylabel("Northing in meters")
        plt.scatter(X, Y, color='gray')

        print(fig_name,":")
        
        rms1 = self.get_line_fit(segment1_x, segment1_y, 'r')
        # print("RMS of segment 1:", rms1)

        rms2 = self.get_line_fit(segment2_x, segment2_y, 'b')
        # print("RMS of segment 2: ", rms2)

        rms3 = self.get_line_fit(segment3_x, segment3_y, 'g')
        # print("RMS of segment 3: ", rms3)

        rms4 = self.get_line_fit(segment4_x, segment4_y, 'y')
        # print("RMS of segment 4: ", rms4)

        rms_avg = (rms1 + rms2 + rms3 + rms4)/4
        print("RMS error: ",rms_avg)
        pass

    def get_line_fit(self, segment_easting, segment_northing, color):
        
        # Member function that provides the best fit line using the Line of Best Fit method
        sum_Northing = 0
        sum_Easting = 0
        numerator = 0
        denominator = 0

        y_val = []
        err_sq_sum = 0

        # Getting the sum
        for i in range(len(segment_easting)):
            
            sum_Northing = sum_Northing + segment_northing[i]
            sum_Easting = sum_Easting + segment_easting[i]
            pass
        
        # Calculating the mean
        mean_Northing = sum_Northing / len(segment_northing)
        mean_Easting = sum_Easting / len(segment_easting)

        # Line of Best Fit
        for i in range(len(segment_easting)):

            numerator = numerator + ((segment_easting[i] - mean_Easting) * (segment_northing[i] - mean_Northing))
            denominator = denominator + ((segment_easting[i] - mean_Easting) ** 2)
            pass

        m = numerator / denominator
        b = mean_Northing - (m * mean_Easting)

        for i in range(len(segment_easting)):
            
            y_val.append((m * segment_easting[i]) + b)
            err_sq_sum = err_sq_sum + ((y_val[i] - segment_northing[i]) ** 2)
            pass
        
        plt.plot(segment_easting, y_val, color, linewidth=1.5)
        rms = (err_sq_sum / len(segment_easting)) ** 0.5
        
        return(rms)

    def print_val(self, messages):
        for message in messages:
            print("Header: ", message.header)
            print("Easting: ", message.easting)
            print("Northing: ", message.northing)
            print("sec: ", message.sec, "\n")

if __name__ == '__main__':
    msg_plotter()