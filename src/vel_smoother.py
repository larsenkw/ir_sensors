#!/usr/bin/env python

from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist

class Server(object):
    def __init__(self, smoothing=True):
        # Velocity smoothing parameters (be sure to make these values floats by adding a decimal point)
        self.smoothing = smoothing
        self.frequency = 60. # Hz
        self.max_accel_lin = 1. # m/s^2
        self.max_accel_ang = 1. # rad/s^2
        self.delta_v_lin = self.max_accel_lin/self.frequency # maximum change in velocity since previous command
        self.delta_v_ang = self.max_accel_ang/self.frequency # maximum change in angular velocity
        self.command_Twist = Twist()
        self.command_Twist.linear.x = 0.
        self.command_Twist.angular.z = 0.
        self.smoothed_Twist = Twist() # final Twist command sent to turtlebot after being smoothed
        self.smoothed_Twist.linear.x = 0.
        self.smoothed_Twist.angular.z = 0.
        self.previous_Twist = Twist() # previously seen Twist command from last publish
        self.previous_Twist = self.smoothed_Twist

    def callback_update_command(self, msg):
        # Update the current command
        self.command_Twist = msg

    def callback_vel_smoother(self, msg):
        # Linear Smoothing
        self.previous_Twist.linear.x = msg.linear.x
        # Command is within lower limit
        if (self.command_Twist.linear.x >= self.previous_Twist.linear.x - self.delta_v_lin):
            # Command is above upper limit
            if (self.command_Twist.linear.x > self.previous_Twist.linear.x + self.delta_v_lin):
                self.smoothed_Twist.linear.x = self.previous_Twist.linear.x + self.delta_v_lin
            else:
                # Command is within range
                self.smoothed_Twist.linear.x = self.command_Twist.linear.x
        # Command is below lower limit
        else:
            self.smoothed_Twist.linear.x = self.previous_Twist.linear.x - self.delta_v_lin
        # Angular Smoothing (same logic as above)
        self.previous_Twist.angular.z = msg.angular.z
        if (self.command_Twist.angular.z >= self.previous_Twist.angular.z - self.delta_v_ang):
            if (self.command_Twist.angular.z > self.previous_Twist.angular.z + self.delta_v_ang):
                self.smoothed_Twist.angular.z = self.previous_Twist.angular.z + self.delta_v_ang
            else:
                self.smoothed_Twist.angular.z = self.command_Twist.angular.z
        else:
            self.smoothed_Twist.angular.z = self.previous_Twist.angular.z - self.delta_v_ang

server = Server()
def vel_smoother(server):
    rospy.init_node('vel_smoother')

    # Subscriber to velocity controller to update commands
    #rospy.Subscriber('ir_cmd_vel', Twist, server.callback_update_command) # for ir sensors
    rospy.Subscriber("vel_cmd_obstacle", Twist, server.callback_update_command) # for combined
    # Subscribe to its own publisher so it updates the smoothed velocity with each publish
    rospy.Subscriber('cmd_vel_mux/input/teleop', Twist, server.callback_vel_smoother)

    # Create Publisher
    pub_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 10)

    # Continue publishing at a specific frequency to help with smoothing
    rate = rospy.Rate(server.frequency)
    while not rospy.is_shutdown():
        pub_vel.publish(server.smoothed_Twist)
        rospy.loginfo(server.smoothed_Twist)
        rate.sleep()

if __name__ == "__main__":
    try:
        vel_smoother(server)
    except rospy.ROSInterruptException:
        pass
