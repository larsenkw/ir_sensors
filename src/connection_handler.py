#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool


class Server(object):
    def __init__(self):
        self.following = Bool()
        self.connection = Bool()

        self.pub_following = rospy.Publisher("following_status", Bool, queue_size=10)
        self.pub_connection = rospy.Publisher("connection_status", Bool, queue_size=10)

    def callback_arduino_connection(self, msg):
        self.pub_connection.publish(msg)

    def callback_arduino_following(self, msg):
        self.pub_following.publish(msg)

    def callback_joy_connection(self, msg):
        self.pub_connection.publish(msg)

    def callback_joy_following(self, msg):
        self.pub_following.publish(msg)

def connection_handler(server):
    # Start ROS
    rospy.init_node("connection_handler")
    rospy.Subscriber("arduino_connection_status", Bool, server.callback_arduino_connection)
    rospy.Subscriber("joy_connection_status", Bool, server.callback_joy_connection)
    rospy.Subscriber("arduino_following_status", Bool, server.callback_arduino_following)
    rospy.Subscriber("joy_following_status", Bool, server.callback_joy_following)

    # Keep ROS running
    rospy.spin()

server = Server()
if __name__ == "__main__":
    try:
        connection_handler(server)
    except rospy.ROSInterruptException:
        pass
