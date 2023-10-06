#!/usr/bin/python3

import rosbag
from gps_driver.msg import gps_msg
import matplotlib.pyplot as plt

class data_points:
# data_points is a class capable of storing all the relevant data required for the analysis of the various data sets
# The class has several member functions to assist with handling the data

    def __init__(self):
        self.Easting = []
        self.Northing = []
        self.normalized_easting = []
        self.normalized_northing = []
        self.Altitude = []
        self.time = []
        self.size = 0
        pass

    def append_data(self, msg):
        
        # Member function to add data to object
        self.Easting.append(msg.UTM_easting)
        self.Northing.append(msg.UTM_northing)
        self.Altitude.append(msg.Altitude)
        self.time.append(msg.Header.stamp.secs)
        self.size = self.size + 1
        pass

    def normalize(self):
        
        # Member function to normalize the input data by subtracting the first value from all the data points
        # The function returns the slope and the constant of the line function respectively 
        base_easting = self.Easting[0]
        base_northing = self.Northing[0]
        
        for i in range(self.size):
            self.normalized_easting.append(self.Easting[i] - base_easting)
            pass

        for i in range(self.size):
            self.normalized_northing.append(self.Northing[i] - base_northing)
            pass
    
    def get_line_fit(self):
        
        # Member function that provides the best fit line using the Line of Best Fit method
        sum_Northing = 0
        sum_Easting = 0
        numerator = 0
        denominator = 0

        # Getting the sum
        for i in range(self.size):
            
            sum_Northing = sum_Northing + self.Northing[i]
            sum_Easting = sum_Easting + self.Easting[i]
            pass
        
        # Calculating the mean
        mean_Northing = sum_Northing / self.size
        mean_Easting = sum_Easting / self.size

        # Line of Best Fit
        for i in range(self.size):

            numerator = numerator + ((self.Easting[i] - mean_Easting) * (self.Northing[i] - mean_Northing))
            denominator = denominator + ((self.Easting[i] - mean_Easting) ** 2)
            pass

        m = numerator / denominator
        b = mean_Northing - (m * mean_Easting)
        
        return(m, b)




class GPS_plotter:

    def __init__(self):

        # Opening bag files
        occluded_bag = rosbag.Bag("/home/bhanu/catkin_ws/data/occluded.bag")
        open_bag = rosbag.Bag("/home/bhanu/catkin_ws/data/open.bag")
        st_line = rosbag.Bag("/home/bhanu/catkin_ws/data/straight_line.bag")

        # Initializing data_points objects
        self.occluded_data = data_points()
        self.open_data = data_points()
        self.st_line_data = data_points()

        # Ablosute values (easting, northing)
        self.occluded_abs = (327975, 4689574)
        self.open_abs = (324307, 4675039)

        # Reading data from bag files
        for topic, msg, t in occluded_bag.read_messages(topics=['/gps']):
            self.occluded_data.append_data(msg)
            pass

        for topic, msg, t in open_bag.read_messages(topics=['/gps']):
            self.open_data.append_data(msg)
            pass

        for topic, msg, t in st_line.read_messages(topics=['/gps']):
            self.st_line_data.append_data(msg)
            pass

        # Closing the bag objects
        occluded_bag.close()
        open_bag.close()
        st_line.close()

        self.plot_scatter()
        pass
    
    def plot_scatter(self):
        
        # Normalizing the input data
        self.occluded_data.normalize()
        self.open_data.normalize()

        # Limit for x and y axes
        plot_limit = 3.5

        slope, con = self.st_line_data.get_line_fit()
        y_val = []
        err_sq_sum = 0

        # Calculating rms value
        for i in range(self.st_line_data.size):
            y_val.append((slope * self.st_line_data.Easting[i]) + con)
            err_sq_sum = err_sq_sum + ((y_val[i] - self.st_line_data.Northing[i]) ** 2)
            pass

        rms = (err_sq_sum / self.st_line_data.size) ** 0.5


        # Opening the 1st window
        plt.figure("Stationary data-set",figsize=(12,12))

        # Plotting normalized easting vs normalized northing of occluded data set
        plt.subplot(221)
        plt.scatter(self.occluded_data.normalized_easting, self.occluded_data.normalized_northing)
        plt.title("Occluded data set")
        plt.xlabel("Normalized Easting")
        plt.ylabel("Normalized Northing")
        plt.xlim(-plot_limit, plot_limit)
        plt.ylim(-plot_limit, plot_limit)

        # Plotting normalized easting vs normalized northing of open data set
        plt.subplot(222)
        plt.scatter(self.open_data.normalized_easting, self.open_data.normalized_northing)
        plt.title("Open data set")
        plt.xlabel("Normalized Easting")
        plt.ylabel("Normalized Northing")
        plt.xlim(-plot_limit, plot_limit)
        plt.ylim(-plot_limit, plot_limit)
        
        # Plotting the altitude vs time of occluded data set
        plt.subplot(223)
        plt.scatter(self.occluded_data.time, self.occluded_data.Altitude)
        plt.title("Occluded data set")
        plt.xlabel("Time elapsed")
        plt.ylabel("Altitude")
        
        # Plotting the altitude vs time of open data set
        plt.subplot(224)
        plt.scatter(self.open_data.time, self.open_data.Altitude)
        plt.title("Open data set")
        plt.xlabel("Time elapsed")
        plt.ylabel("Altitude")


        # Opening 2nd window
        plt.figure("Error histogram", figsize=(12,6))

        error_val = self.get_error(self.occluded_data, self.occluded_abs)
        plt.subplot(121)
        plt.hist(error_val, bins=30)
        plt.title("Occluded data set")
        plt.xlabel("Error")
        plt.ylabel("Frequency")

        error_val = self.get_error(self.open_data, self.open_abs)
        plt.subplot(122)
        plt.hist(error_val, bins=30)
        plt.title("Open data set")
        plt.xlabel("Error")
        plt.ylabel("Frequency")

        
        # Opening 3rd window
        plt.figure("Walking data-set",figsize=(12,12))

        # Plotting easting vs northing of st_line data set
        plt.subplot(221)
        plt.scatter(self.st_line_data.Easting, self.st_line_data.Northing)
        plt.title("Easting vs Northing")
        plt.xlabel("Easting")
        plt.ylabel("Northing")

        # Plotting the altitude vs time of st_line data set
        plt.subplot(222)
        plt.scatter(self.st_line_data.time, self.st_line_data.Altitude)
        plt.title("Altitude vs Time")
        plt.xlabel("Time Elapsed")
        plt.ylabel("Altitude")
        
        # Plotting the fitted line
        plt.subplot(223)
        plt.scatter(self.st_line_data.Easting, self.st_line_data.Northing)
        plt.plot(self.st_line_data.Easting, y_val, "r", linewidth=2)
        plt.title("Fitted line")
        plt.xlabel("Easting")
        plt.ylabel("Northing")


        print("RMS error of straight line data:", rms)
        plt.show()
        pass
    
    def get_error(self, data, control):
        
        error_val = []
        
        # Getting the magnitude of error
        for i in range(data.size):
            error_val.append((((data.Easting[i] - control[0]) ** 2) + ((data.Northing[i] - control[1]) ** 2)) ** 0.5)
            pass
        
        return error_val
    pass    


if __name__ == '__main__':
    GPS_plotter()