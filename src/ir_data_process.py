#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int8
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import rospkg
import numpy as np
import pandas as pd


#===============================================================================
# Sensor frame definition
#===============================================================================
# Positive displacement is to the right and up (X,Z)
# Positive angle is clockwise (right-hand rule, because Y-axis is pointing
# downward/into the screen)
#
#  Top View
#                 x   /
#  |    *         |__/ theta
#  |    |    *    | /              Z
#  |    |  --|----*-------z        ^
#  |    |    |    |    *           |              -theta  0  +theta
#  |    |    |    |    |    *      |                   \  |  /
#  |    |    |    |    |    |      |                    \ | /
#  |    |    |    |    |    |      |                     \|/
#  S6---S5---S4---S3---S2---S1     +----------> X         *
#               |
# -x3  -x2  -x1 0 x1   x2   x3


#===============================================================================
# Object for performing calculations
#===============================================================================
class IRSensors(object):
    def __init__(self, window_len=10):
        # Length of averaging window (how many past points to average)
        self.window_len = window_len
        self.store_len = 2*window_len # how many values to store in numpy arrays
        self.max_distance = 140 # cm

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
        self.seq = 0

    def rawDataToPose(self, data, LS_header):
        self.values = list(data)
        self.time_s = rospy.get_time()
        self.times = np.append(self.times[-self.store_len:], self.time_s)
        for i in range(len(self.values)):
            #--- Convert data to int
            self.values[i] = int(self.values[i])
            #--- Store analog value
            self.values_analog[i] = np.append(self.values_analog[i][-self.store_len:], self.values[i])
            #--- Convert to distance (cm) and store
            self.distances[i] = self.analogToDistance(self.values[i])
            self.values_cm[i] = np.append(self.values_cm[i][-self.store_len:], self.distances[i])
            #--- Calculate average over window
            self.values_avg[i] = np.mean(self.values_analog[i][-self.window_len:])
            self.distances_avg[i] = np.mean(self.values_cm[i][-self.window_len:])
            self.valuesavg_cm[i] = np.append(self.valuesavg_cm[i][-self.store_len:], self.distances_avg[i])

        #--- Caluclate angle and average distance
        self.theta_p, self.avg_distance, self.num_sensors, self.rotation_direction, self.pose = self.calcPose(self.distances_avg, LS_header)
        return self.theta_p, self.avg_distance, self.num_sensors, self.rotation_direction, self.pose

    def analogToDistance(self, analog):
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

    def calcPose(self, distances, LS_header):
        # For the header values of the PoseStamped message, the time stamp will
        # be taken from the header sent from the LaserScan message from the
        # Arduino (LS_header)

        # Determine which sensors are maxed out
        maxed_out = []
        rows_to_delete = [] # indices for rows to delete from data to find linear fit
        for i in range(len(distances)):
            if (distances[i] > self.max_distance):
                maxed_out.append(1)
                rows_to_delete.append(i)
            else:
                maxed_out.append(0)
        # If all sensors are maxed out, theta_p = 0deg
        if not (sum(maxed_out) == 6):
            # Removed maxed out data
            spacing = 2
            y = np.array([distances[0],distances[1],distances[2],distances[3],distances[4],distances[5]])
            #x = np.array([5*spacing,4*spacing,3*spacing,2*spacing,1*spacing,0*spacing])
            x = np.array([2.5*spacing, 1.5*spacing, 0.5*spacing, -0.5*spacing, -1.5*spacing, -2.5*spacing])
            y = np.delete(y, rows_to_delete)
            x = np.delete(x, rows_to_delete)
            # Solve least squares for linear best fit parameters
            # y = A*p, A = [x, 1], p = [m, b], whwere line is y = m*x + b
            A = np.vstack([x, np.ones(len(x))]).T
            m, b = np.linalg.lstsq(A, y)[0]
            # Use slope of line (m) to determine angle
            theta_p = np.arctan(m) + (np.pi/2)
            theta_p = (180./np.pi)*theta_p
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
            angle_error = np.abs(angles - theta_p)
            theta_p = angles[angle_error.argmin()]
            # Convert to range 90 <-> -90 deg
            theta_p = -(theta_p - 90)
            # Find average distance after removing maxed sensor readings
            avg_distance = np.mean(y)
            num_sensors = 6 - sum(maxed_out)
            # If sensor 1 is maxed out, it means the person is to the left of
            # robot, so the robot should rotate counter-clockwise (+1 direction)
            if (maxed_out[0]):
                rotation_direction = 1
            elif (maxed_out[5]):
                rotation_direction = -1
            else:
                rotation_direction = 0
        else:
            theta_p = 0
            avg_distance = 150
            num_sensors = 0
            rotation_direction = 0
            x = 0

        # Generate pose message
        pose = PoseStamped()
        pose.header.seq = self.seq
        self.seq += 1
        pose.header.stamp = LS_header.stamp
        pose.header.frame_id = 'IR_Pose'
        pose.pose.position.z = avg_distance/100. # convert [cm] to [m]
        pose.pose.position.x = np.mean(x)/100. # conert [cm] to [m]
        k = [0, 1, 0] # axis of rotation, Y-axis
        # Quaternion scalar value
        q0 = np.cos(theta_p/2.0)
        # Quaternion vector values
        q1 = np.sin(theta_p/2.0)*k[0]
        q2 = np.sin(theta_p/2.0)*k[1]
        q3 = np.sin(theta_p/2.0)*k[2]
        pose.pose.orientation.x = q1
        pose.pose.orientation.y = q2
        pose.pose.orientation.z = q3
        pose.pose.orientation.w = q0
        return theta_p, avg_distance, num_sensors, rotation_direction, pose


#===============================================================================
# ROS Node Content
#===============================================================================
# Create publisher
pub_human_angle = rospy.Publisher('ir_angle', Float64, queue_size = 10)
pub_distance = rospy.Publisher('ir_distance', Float64, queue_size = 10)
pub_num_sensors = rospy.Publisher('num_sensors', Int8, queue_size = 10)
pub_rotation_dir = rospy.Publisher('rotation_direction', Int8, queue_size = 10)
pub_ir_pose = rospy.Publisher('IR_pose', PoseStamped, queue_size = 10)

def callback(msg, ir_sensors):
    # Receive raw data and publish a value
    # The raw_data variable should be an array of numbers
    raw_data = msg.ranges
    theta_p, distance, num_sensors, rotation_direction, pose = ir_sensors.rawDataToPose(raw_data, msg.header)
    rospy.loginfo('Angle: {:.0f}, Distance: {:.2f}, Num Sensors: {:d}, Rot Dir: {:d}'.format(theta_p, distance, num_sensors, rotation_direction))
    rospy.loginfo(pose)
    pub_human_angle.publish(Float64(theta_p))
    pub_distance.publish(Float64(distance))
    pub_num_sensors.publish(Int8(num_sensors))
    pub_rotation_dir.publish(Int8(rotation_direction))
    pub_ir_pose.publish(pose)

def ir_data_process(ir_sensors):
    rospy.init_node('ir_data_process')#, anonymous=True)
    rospy.Subscriber('ir_raw_data', LaserScan, callback, ir_sensors)
    rospy.spin()

if __name__ == '__main__':
    window_len = 10
    ir_sensors = IRSensors(window_len)
    try:
        ir_data_process(ir_sensors)
    except rospy.ROSInterruptException:
        pass
