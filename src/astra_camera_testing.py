#! /usr/bin/env python

import rospy
from numpy import arctan2
from skeleton_markers.msg import Skeleton


class CameraTest():
    def __init__(self):
        rospy.init_node("camera_test")
        rospy.Subscriber("/skeleton", Skeleton, self.skeleton_callback)
        rospy.spin()

    def skeleton_callback(self, msg):
        index = 2
        user_id = msg.user_id
        name = msg.name[index]
        x = msg.position[index].x
        y = msg.position[index].y
        z = msg.position[index].z
        theta = arctan2(z,x)
        print "%d,%s:(%f,%f,%f), theta: %f" % (user_id, name, x, y, z, theta)

if __name__ == "__main__":
    try:
        CameraTest()
    except rospy.ROSInterruptException:
        pass
