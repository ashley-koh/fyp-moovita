#!/usr/bin/env python

import numpy
from matplotlib import pyplot as plt
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def plot(msg):
    global counter
    if counter % 10 == 0:
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(msg.pose.pose.position.x, msg.pose.pose.position.y, 'bo')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

if __name__ == "__main__":
    counter = 0
    rospy.init_node("plotter")
    # rospy.Subscriber("odometry/filtered", Odometry, plot)
    rospy.Subscriber("encoder0/Odometry", Odometry, plot)
    # rospy.Subscriber("matcher0/PoseWithCovarianceStamped", PoseWithCovarianceStamped, plot)
    plt.ion()
    plt.show()
    rospy.spin()