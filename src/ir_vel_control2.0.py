#!/usr/bin/env python

# Reads messages from:
#   'IR_pose'
#   'following_status'
#   'BT_connection_status'
# And then determines the velocity commands to give to the Turtlebot

import rospy
from std_msgs.msg import Float64, Int8, Bool, String
from geometry_msgs.msg import Twist, PoseStamped

class Server(object):
    def __init__(self):
        self.theta_p = None # predicted angle of person in front
        self.distance = None # average distance from IR readings
        self.num_sensors = None # number of sensors not maxed out
        self.rotation_direction = None # predicted direction robot needs to rotate (+ counter-clockwise, - clockwise)
        self.follow = Bool() # indicates whether the app user has told the robot to follow or not
        self.follow.data = False
        self.connection = Bool() # indicates whether the phone is currently connected
        self.connection.data = False
        self.command = String()
        self.command.data = '' # Blank string indicates that there is currently no command
        self.pose = PoseStamped() # The human's pose w.r.t. IR sensors
        self.msg_Twist = Twist() # robot velocity command
        self.d_offset = 40 # cm
        self.d_max = 150 # cm
        self.delta = 10 # cm
        self.correcting_distance = 0 # indicates whether the robot is too far (1), too close (-1) or stopped (0)
        self.k_lin = 0.02 # linear velocity proportion parameter
        self.k_ang = (1.0/1.0) # angular velocity proportion parameter
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
        if (msg.linear.x > 0.5 and self.follow.data == False):
            self.follow.data = True
        if (msg.linear.x < -0.5 and self.follow.data == True):
            self.follow.data = False
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
        if (self.follow.data and self.connection.data):
            self.velocityCommand()
        else:
            self.msg_Twist.linear.x = 0
            self.msg_Twist.angular.z = 0
            self.pub_vel.publish(self.msg_Twist)
            rospy.loginfo(self.msg_Twist)

    def velocityCommand(self):
        #----- Linear Velocity -----#
        # Proportional control for velocity
        self.distance = np.sqrt(pose.pose.position.x**2 + pose.pose.position.z**2)
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
                self.msg_Twist.linear.x = 0.0
                self.correcting_distance = 0
            else:
                self.msg_Twist.linear.x = self.k_lin*self.d_error

        elif (self.correcting_distance == -1):
            # currently moving backward
            if (self.d_error >= 0):
                self.msg_Twist.linear.x = 0.0
                self.correcting_distance = 0
            else:
                self.msg_Twist.linear.x = self.k_lin*self.d_error

        #----- Angular Velocity -----#
        # Angular velocity is proportional to the number of sensors that are maxed out
        self.msg_Twist.angular.z = self.k_ang*(np.atan(self.pose.pose.position.z/self.pose.pose.position.x) - np.pi/2)

        # If readers are out of range, STOP
        if (self.num_sensors == 0):
            self.msg_Twist.linear.x = 0
            self.msg_Twist.angular.z = 0
            self.correcting_distance = 0
        # Publish velocity command
        self.pub_vel.publish(self.msg_Twist)
        print('Error: %d, Correcting: %d' % (self.d_error, self.correcting_distance))
        rospy.loginfo(self.msg_Twist)

def ir_vel_control(server):
    # Read IR sensor data
    rospy.init_node('ir_vel_control')
    rospy.Subscriber('IR_pose', PoseStamped, server.callback_pose)
    rospy.Subscriber('num_sensors', Int8, server.callback_num_sensors)

    # Read app commands to control robot
    rospy.Subscriber('/following_status', Bool, server.callback_following_status)
    rospy.Subscriber('/BT_connection_status', Bool, server.callback_BT_connection_status)
    rospy.Subscriber('/other_commands', String, server.callback_other_commands)
    rospy.spin()


server = Server()
if __name__ == '__main__':
    try:
        ir_vel_control(server)
    except rospy.ROSInterruptException:
        pass
