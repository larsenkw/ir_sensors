#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray, Float64
import rospkg
import numpy as np
import pandas as pd


#===============================================================================
# Object for performing calculations
#===============================================================================
class IRSensors(object):
    def __init__(self, window_len=10):
        # Length of averaging window (how many past points to average)
        self.window_len = window_len
        self.store_len = 2*window_len # how many values to store in numpy arrays

        # Create linear interpolation function from inverse distance plot found in the
        # data sheet for GP2Y0A41SK0F (the .csv files is saved as '4-30cm Inverse Plot
        # Data.csv')
        # The function for converting from voltage to distance is:
        # y = m*x + b
        # y = voltage (V)
        # x = inverse distance 1/(d + 0.42), d = distance in cm
        # m,b = linear regression constants
        rospack = rospkg.RosPack()
        filename = '20-150cm Inverse Plot Data.csv'
        try:
            self.df = pd.read_csv(rospack.get_path('ir_sensors') + '/src/' + filename, sep=',', header=None)
        except:
            rospy.logerr('Failed to read file: '+filename)
            rospy.logerr('Make sure it is in the ' + rospack.get_path('ir_sensors') + '/src/ ' + ' folder')
        # Remove the last 3 data points, these contain values outside the linear region
        # with d < 4cm
        self.y =self.df.values[0:-3,0] # inverse cm data
        self.x = self.df.values[0:-3,1] # voltage data
        # Linear fit parameters
        self.p = np.polyfit(self.x,self.y,1)
        self.m = self.p[0]
        self.b = self.p[1]

        # Initialize values and arrays for storing past values
        self.iter_val = 0
        self.values_analog = [np.array([]),np.array([]),np.array([]),np.array([]),np.array([]),np.array([])]
        self.distances = [0, 0, 0 ,0 ,0 ,0]
        self.values_cm = [np.array([]),np.array([]),np.array([]),np.array([]),np.array([]),np.array([])]
        self.values_avg = [0, 0, 0, 0, 0, 0]
        self.distances_avg = [0, 0, 0, 0, 0, 0]
        self.valuesavg_cm = [np.array([]),np.array([]),np.array([]),np.array([]),np.array([]),np.array([])]
        self.times = np.array([])

    def convertData(self, data):
        self.values = list(data)
        self.time_s = rospy.get_time()
        self.times = np.append(self.times[-self.store_len:], self.time_s)
        for i in range(len(self.values)):
            #--- Convert data to int
            self.values[i] = int(self.values[i])
            #--- Store analog value
            self.values_analog[i] = np.append(self.values_analog[i], self.values[i])
            #--- Convert to distance (cm) and store
            self.distances[i] = self.calcDistance(self.values[i])
            self.values_cm[i] = np.append(self.values_cm[i], self.distances[i])
            #--- Calculate average over window
            self.values_avg[i] = np.mean(self.values_analog[i][-self.window_len:])
            self.distances_avg[i] = self.calcDistance(self.values_avg[i])
            self.valuesavg_cm[i] = np.append(self.valuesavg_cm[i], self.distances_avg[i])

        #--- Caluclate angle and average distance
        self.theta, self.avg_distance = self.calcAngleAvgDistance(self.distances_avg)
        return self.theta, self.avg_distance

    def calcDistance(self, analog):
        # Convert analog reading to Voltage
        V = (analog/1024.)*5
        # Convert voltage to inverse distance
        inv_cm = self.m*V + self.b
        # Avoid issues with dicontinuities by keeping inv_cm >= 0.02
        # this causes the distance to max at ~300cm
        if (inv_cm < 0.003):
            inv_cm = 0.003
        # Convert inverse distance to distance
        distance = (1./inv_cm)
        return distance

    def calcAngleAvgDistance(self, distances):
        # Unpack distance values
        distance1 = distances[0]
        distance2 = distances[1]
        distance3 = distances[2]
        distance4 = distances[3]
        distance5 = distances[4]
        distance6 = distances[5]
        # Determine which endpoints are irrelevant (reaching near maximum reading)
        # (if end value is >150cm then check the next end-most sensor value as well)
        ignore1 = False
        ignore2 = False
        ignore3 = False
        ignore4 = False
        ignore5 = False
        ignore6 = False
        if (distance1 > 150):
            ignore1 = True
            if (distance2 > 150):
                ignore2 = True
                if (distance3 > 150):
                    ignore3 = True
        if (distance6 > 150):
            ignore6 = True
            if (distance5 > 150):
                ignore5 = True
                if (distance4 > 150):
                    ignore4 = True
        # If all sensors are maxed out, theta = 90deg
        if not (ignore1 and ignore2 and ignore3 and ignore4 and ignore5 and ignore6):
            # Find line of best fit
            spacing = 2
            y = np.array([distance1,distance2,distance3,distance4,distance5,distance6])
            x = np.array([0,spacing,2*spacing,3*spacing,4*spacing,5*spacing])
            if (ignore1):
                y = np.delete(y,0)
                x = np.delete(x,0)
            if (ignore2):
                y = np.delete(y,0)
                x = np.delete(x,0)
            if (ignore3):
                y = np.delete(y,0)
                x = np.delete(x,0)
            if (ignore6):
                y = np.delete(y,-1)
                x = np.delete(x,-1)
            if (ignore5):
                y = np.delete(y,-1)
                x = np.delete(x,-1)
            if (ignore4):
                y = np.delete(y,-1)
                x = np.delete(x,-1)
            # Solve least squares for linear best fit parameters
            # y = A*p, A = [x, 1], p = [m, b], whwere line is y = m*x + b
            A = np.vstack([x, np.ones(len(x))]).T
            m, b = np.linalg.lstsq(A, y)[0]
            # Use slope of line (m) to determine angle
            theta = np.arctan(m) + (np.pi/2)
            theta = (180./np.pi)*theta
            # Set number of angles to use
            num_angles = 5
            num_divisions = num_angles - 1
            # Create angle divisions from 0 to 180 deg
            delta_angle = 180./num_divisions
            angles = np.array([0])
            for i in range(1,num_divisions):
                angles = np.append(angles, i*delta_angle)
            angles = np.append(angles, 180)
            # Find closest angle and return it
            angle_error = np.abs(angles - theta)
            theta = angles[angle_error.argmin()]
            # Find average distance after removing maxed sensor readings
            avg_distance = np.mean(y)
        else:
            theta = 90
            avg_distance = 150
        return theta, avg_distance


#===============================================================================
# ROS Node Content
#===============================================================================
# Create publisher
pub_angle = rospy.Publisher('ir_angle', Float64)
pub_distance = rospy.Publisher('ir_distance', Float64)

def callback(data, ir_sensors):
    # Receive raw data and publish a value
    raw_data = data.data
    theta, distance = ir_sensors.convertData(raw_data)
    rospy.loginfo('Angle: {:.0f}, Distance: {:.2f}'.format(theta, distance))
    pub_angle.publish(Float64(theta))
    pub_distance.publish(Float64(distance))

def ir_raw_to_dist_ang(ir_sensors):
    rospy.init_node('ir_raw_to_dist_ang')#, anonymous=True)
    rospy.Subscriber('ir_raw_data', Int16MultiArray, callback, ir_sensors)
    rospy.spin()

if __name__ == '__main__':
    window_len = 10
    ir_sensors = IRSensors(window_len)
    try:
        ir_raw_to_dist_ang(ir_sensors)
    except rospy.ROSInterruptException:
        pass
