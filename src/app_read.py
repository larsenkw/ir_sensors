#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def callback(msg):
    if (msg.linear.x > 0.5 and follow.data == False):
        follow.data = True
        rospy.loginfo('Now following')
    if (msg.linear.x < -0.5 and follow.data == True):
        follow.data = False
        rospy.loginfo('Stop following')
    #rospy.loginfo(follow.data)

def app_read():
    rospy.init_node('app_read')
    rospy.Subscriber('/joy_teleop/cmd_vel', Twist, callback)
    rospy.spin()

pub = rospy.Publisher('follow', Bool, queue_size=10)
follow = Bool()
follow.data = False

if __name__ == '__main__':
    try:
        app_read()
    except rospy.ROSInterruptException:
        pass
