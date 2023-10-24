import rosbag
from std_msgs.msg import Header
import matplotlib.pyplot as plt
import time
import utm

# Open Vertices: 185, 380, 545, 730
# Occluded Vertices: 260, 390, 610


class msg:
    def __init__(self, header, easting, northing, alt, sec):
        
        self.header = header
        self.easting = easting
        self.northing = northing
        self.alt = alt
        self.sec = sec
        pass

class msg_plotter():
    def __init__(self):

        self.messages = []
        
        # open_abs = [327969, 4689505]
        # colluded_abs = [328178, 4689481]

        open_abs = utm.from_latlon(42.33862, -71.08575)
        colluded_abs = utm.from_latlon(42.33879, -71.0883)

        path = "/home/bhanu/catkin_ws/src/lab2/data/occluded_stationary.bag"
        occluded_messages = self.read_val(path)

        path2 = "/home/bhanu/catkin_ws/src/lab2/data/open_stationary.bag"
        open_messages = self.read_val(path2)
        # self.print_val()

        self.plot_graph(occluded_messages, colluded_abs, "Occluded")
        self.plot_graph(open_messages, open_abs, "Open")

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
            alt = data.altitude

            msg_list.append(msg(header, easting, northing, alt, sec))
            
        bagread.close()
        return msg_list

    def plot_graph(self, messages, abs, name):

        Y = []
        X = []
        sec = []
        alt = []

        for i in messages:
            Y.append(i.northing)
            X.append(i.easting)
            # print(i.alt, i.sec)
            sec.append(i.sec)
            alt.append(i.alt)

        # print("val:", len(sec), len(alt))

        plt.figure(name + " Altitude graph")
        plt.xlabel("Time")
        plt.ylabel("Altitude")
        plt.scatter(sec, alt)
        
        plt.figure(name)
        plt.xlabel("Easting in meters")
        plt.ylabel("Northing in meters")
        plt.scatter(X, Y)

        # print(name, ":", X[0], Y[0])
        
        plt.scatter(abs[0], abs[1], color='red')
        
        err_val, avg_error = self.get_error(X, Y, abs)
        print("Average", name + " error:", avg_error)

        plt.figure("Histogram of " + name + " data set")
        plt.xlabel("Error in meters")
        plt.ylabel("Frequency")
        plt.hist(err_val, bins=30)
        pass

    def get_error(self, x, y, control):
        
        error_val = []
        error_easting = []
        error_northing = []
        avg_error = 0
        
        # Getting the magnitude of error
        for i in range(len(x)):
            error_easting.append(x[i] - control[0])
            error_northing.append(y[i] - control[1]) 
            error_val.append((((error_easting[i]) ** 2) + ((error_northing[i]) ** 2)) ** 0.5)
            avg_error = avg_error + error_val[i]
            pass

        avg_error = avg_error/len(x)
        
        return (error_val, avg_error)
    pass

    def print_val(self):
        for message in self.messages:
            print("Header: ", message.header)
            print("Easting: ", message.easting)
            print("Northing: ", message.northing)
            print("sec: ", message.sec, "\n")

if __name__ == '__main__':
    msg_plotter()