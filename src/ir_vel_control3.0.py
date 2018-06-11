#!/usr/bin/env python

# Reads messages from:
#   'IR_pose'
#   'following_status'
#   'BT_connection_status'
# And then determines the velocity commands to give to the Turtlebot

import rospy
from std_msgs.msg import Float64, Int8, Bool, String
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np

class Server(object):
    def __init__(self):
        self.theta_p = None # predicted angle of person in front
        self.distance = None # average distance from IR readings
        self.num_sensors = None # number of sensors not maxed out
        self.rotation_direction = None # predicted direction robot needs to rotate (+ counter-clockwise, - clockwise)
        self.following = Bool() # indicates whether the app user has told the robot to follow or not
        self.following.data = False
        self.connection = Bool() # indicates whether the phone is currently connected
        self.connection.data = False
        self.command = String()
        self.command.data = '' # Blank string indicates that there is currently no command
        self.pose = PoseStamped() # The human's pose w.r.t. IR sensors
        self.command_Twist = Twist() # robot velocity command
        self.d_offset = 50/100. # m
        self.d_max = 140/100. # m
        self.delta = 10/100. # m
        self.correcting_distance = 0 # indicates whether the robot is too far (1), too close (-1) or stopped (0)
        self.k_lin_forward = 0.8 # linear velocity proportion parameter (for forward motion)
        self.k_lin_reverse = 1.0 # linear velocity proportion parameter (for reverse motion)
        self.k_ang = (15.0/1.0) # angular velocity proportion parameter
        # Velocity smoothing parameters
        self.frequency = 1 # Hz
        self.max_accel_lin = 1 # m/s^2
        self.max_accel_ang = 1 # rad/s^2
        self.delta_v_lin = self.max_accel_lin/self.frequency # maximum change in velocity since previous command
        self.delta_v_ang = self.max_accel_ang/self.frequency # maximum change in angular velocity
        self.command_Twist.linear.x = 0
        self.command_Twist.angular.z = 0
        self.smoothed_Twist = Twist() # final Twist command sent to turtlebot after being smoothed
        self.smoothed_Twist.linear.x = 0
        self.smoothed_Twist.angular.z = 0
        self.previous_Twist = Twist() # previously seen Twist command from last publish
        self.previous_TWist = self.smoothed_Twist
        # Create Publisher
        self.pub_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 10) # for turtlebot simulation

    def callback_theta_p(self, msg):
        self.theta_p = msg.data
        if ((self.theta_p != None)
            and (self.distance != None)
            and (self.num_sensors != None)
            and (self.rotation_direction != None)):
            self.followCommand()

    def callback_distance(self, msg):
        self.distance = msg.data
        if ((self.theta_p != None)
            and (self.distance != None)
            and (self.num_sensors != None)
            and (self.rotation_direction != None)):
            self.followCommand()

    def callback_num_sensors(self, msg):
        self.num_sensors = msg.data
        # if ((self.theta_p != None)
        #     and (self.distance != None)
        #     and (self.num_sensors != None)
        #     and (self.rotation_direction != None)):
        #     self.followCommand()
        self.followCommand()

    def callback_rotation_direction(self, msg):
        self.rotation_direction = msg.data
        if ((self.theta_p != None)
            and (self.distance != None)
            and (self.num_sensors != None)
            and (self.rotation_direction != None)):
            self.followCommand()

    def callback_app_read(self, msg):
        if (msg.linear.x > 0.5 and self.following.data == False):
            self.following.data = True
        if (msg.linear.x < -0.5 and self.following.data == True):
            self.following.data = False
        if ((self.theta_p != None)
            and (self.distance != None)
            and (self.num_sensors != None)
            and (self.rotation_direction != None)):
            self.followCommand()

    def callback_pose(self, msg):
        self.pose = msg
        self.followCommand()

    def callback_following_status(self, msg):
        self.following.data = msg.data
        self.followCommand()

    def callback_BT_connection_status(self, msg):
        self.connection.data = msg.data
        self.followCommand()

    def callback_other_commands(self, msg):
        self.command.data = msg.data
        self.followCommand()

    def followCommand(self):
        if (self.following.data and self.connection.data):
            self.velocityCommand()
        else:
            self.command_Twist.linear.x = 0
            self.command_Twist.angular.z = 0
            self.pub_vel.publish(self.command_Twist)
            rospy.loginfo(self.command_Twist)

    def velocityCommand(self):
        #----- Linear Velocity -----#
        # Proportional control for velocity
        self.distance = np.sqrt(self.pose.pose.position.x**2 + self.pose.pose.position.z**2)
        if (self.distance <= self.d_max):
            self.d_error = self.distance - self.d_offset
            # If distance is outside d_offset +/- delta then move until you get to d_offset
            if (self.correcting_distance == 0):
                if (self.d_error > self.delta):
                    # need to move forward
                    self.correcting_distance = 1
                elif (self.d_error < -self.delta):
                    # need to move backward
                    self.correcting_distance = -1

            if (self.correcting_distance == 1):
                # currently moving forward
                if (self.d_error <= 0):
                    self.command_Twist.linear.x = 0.0
                    self.correcting_distance = 0
                else:
                    self.command_Twist.linear.x = self.k_lin_forward*self.d_error

            elif (self.correcting_distance == -1):
                # currently moving backward
                if (self.d_error >= 0):
                    self.command_Twist.linear.x = 0.0
                    self.correcting_distance = 0
                else:
                    self.command_Twist.linear.x = self.k_lin_forward*self.d_error

        #----- Angular Velocity -----#
        # Angular velocity is proportional to the number of sensors that are maxed out
        if (self.pose.pose.position.x == 0):
            self.command_Twist.angular.z = 0
        else:
            self.command_Twist.angular.z = self.k_ang*(np.arctan2(self.pose.pose.position.z,self.pose.pose.position.x) - np.pi/2)

        # If readers are out of range, STOP
        if (self.num_sensors == 0):
            self.command_Twist.linear.x = 0
            self.command_Twist.angular.z = 0
            self.correcting_distance = 0
        # Publish velocity command
        #self.pub_vel.publish(self.command_Twist)
        print('Error: %d, Correcting: %d' % (self.d_error, self.correcting_distance))
        rospy.loginfo(self.command_Twist)

    def callback_vel_smoother(self, msg):
        # Linear Smoothing
        self.previous_Twist.linear.x = msg.linear.x
        if (self.command_Twist.linear.x >= self.previous_Twist.linear.x - self.delta_v_lin):
            if (self.command_Twist.linear.x > self.previous_Twist.linear.x + self.delta_v_lin):
                self.smoothed_Twist.linear.x = self.previous_Twist.linear.x + self.delta_v_lin
            else:
                self.smoothed_Twist.linear.x = self.command_Twist.linear.x
        else:
            self.smoothed_Twist.linear.x = self.previous_Twist.linear.x - self.delta_v_lin
        # Angular Smoothing
        self.previous_Twist.angular.z = msg.angular.z
        if (self.command_Twist.angular.z >= self.previous_Twist.angular.z - self.delta_v_ang):
            if (self.command_Twist.angular.z > self.previous_Twist.angular.z + self.delta_v_ang):
                self.smoothed_Twist.angular.z = self.previous_Twist.angular.z + self.delta_v_ang
            else:
                self.smoothed_Twist.angular.z = self.command_Twist.angular.z
        else:
            self.smoothed_Twist.angular.z = self.previous_Twist.angular.z - self.delta_v_ang


def ir_vel_control(server):
    # Read IR sensor data
    rospy.init_node('ir_vel_control')
    rospy.Subscriber('IR_pose', PoseStamped, server.callback_pose)
    rospy.Subscriber('num_sensors', Int8, server.callback_num_sensors)

    # Read app commands to control robot
    rospy.Subscriber('/following_status', Bool, server.callback_following_status)
    rospy.Subscriber('/BT_connection_status', Bool, server.callback_BT_connection_status)
    rospy.Subscriber('/other_commands', String, server.callback_other_commands)

    # Velocity Smoothing
    pub_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=100)
    rospy.Subscriber('cmd_vel_mux/input/teleop', Twist, server.callback_vel_smoother)
    #rospy.spin()

    rate = rospy.Rate(server.frequency)
    while not rospy.is_shutdown():
        pub_vel.publish(server.smoothed_Twist)
        rate.sleep()


server = Server()
if __name__ == '__main__':
    try:
        ir_vel_control(server)
    except rospy.ROSInterruptException:
        pass
