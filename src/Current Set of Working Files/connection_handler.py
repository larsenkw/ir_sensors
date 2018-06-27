#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool


class Server(object):
    def __init__(self):
        self.arduino_following = Bool()
        self.arduino_connection = Bool()
        self.joy_following = Bool()
        self.joy_connection = Bool()
        self.following = Bool()
        self.connection = Bool()
        self.prev_following = Bool()
        self.prev_connection = Bool()

        # Set defauls
        self.arduino_following.data = None
        self.arduino_connection.data = None
        self.following.data = False
        self.connection.data = False
        self.prev_following.data = False
        self.prev_connection.data = False

        self.pub_following = rospy.Publisher("following_status", Bool, queue_size=10)
        self.pub_connection = rospy.Publisher("connection_status", Bool, queue_size=10)
        # Send initial signals
        self.pub_following.publish(self.following)
        self.pub_connection.publish(self.connection)

    def callback_arduino_connection(self, msg):
        self.arduino_connection = msg
        self.connection_check()

    def callback_arduino_following(self, msg):
        self.arduino_following = msg

        if (self.arduino_connection.data):
            self.pub_following.publish(self.arduino_following)

    def callback_joy_connection(self, msg):
        self.joy_connection = msg
        self.connection_check()

    def callback_joy_following(self, msg):
        self.joy_following = msg

        if (self.joy_connection.data):
            self.pub_following.publish(self.joy_following)

    def connection_check(self):
        if (self.arduino_connection.data):
            # App is connected, use it for Follow/Stop commands
            self.connection.data = self.arduino_connection.data

            if (self.connection.data != self.prev_connection.data):
                self.prev_connection.data = self.connection.data
                self.pub_connection.publish(self.connection)

        elif (self.joy_connection):
            # App is not connected but joystick is, use it for Follow/Stop commands
            self.connection.data = self.joy_connection.data

            if (self.connection.data != self.prev_connection.data):
                self.prev_connection.data = self.connection.data
                self.pub_connection.publish(self.connection)

        else:
            # No control device connected
            self.connection.data = False

            if (self.connection.data != self.prev_connection.data):
                self.prev_connection.data = self.connection.data
                self.pub_connection.publish(self.connection)

def connection_handler(server):

    rospy.Subscriber("arduino_connection_status", Bool, server.callback_arduino_connection)
    rospy.Subscriber("joy_connection_status", Bool, server.callback_joy_connection)
    rospy.Subscriber("arduino_following_status", Bool, server.callback_arduino_following)
    rospy.Subscriber("joy_following_status", Bool, server.callback_joy_following)

    # Keep ROS running
    rospy.spin()


if __name__ == "__main__":
    try:
        # Start ROS
        rospy.init_node("connection_handler")
        server = Server()
        connection_handler(server)
    except rospy.ROSInterruptException:
        pass
