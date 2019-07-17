#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped

class Reframer():

    def __init__(self):
        self.header = None

        self.imu_frame_id = "odom"
        self.gps_frame_id = "map"
        self.matcher_frame_id = "odom"

        self.imu_pub = rospy.Publisher("imu/data", Imu, queue_size=5)
        self.gps_pub = rospy.Publisher("gps/fix", NavSatFix, queue_size=5)
        self.matcher_pub = rospy.Publisher("matcher/PoseWithCovarianceStamped", PoseWithCovarianceStamped, queue_size=5)

        rospy.Subscriber("imu_node_dual/Imu", Imu, callback=self.imu_handler)
        rospy.Subscriber("imu_node_dual/NavSatFix", NavSatFix, callback=self.gps_handler)
        rospy.Subscriber("matcher_result", PoseWithCovarianceStamped, callback=self.matcher_handler)

    def imu_handler(self, msg):
        self.header = msg.header

        imu_msg = msg
        imu_msg.header.frame_id = self.imu_frame_id

        self.imu_pub.publish(imu_msg)

    def gps_handler(self, msg):
        gps_msg = msg
        gps_msg.header.frame_id = self.gps_frame_id
        self.gps_pub.publish(gps_msg)

    def matcher_handler(self, msg):
        matcher_msg = msg
        matcher_msg.header.frame_id = self.matcher_frame_id
        matcher_msg.pose.covariance = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.matcher_pub.publish(matcher_msg)


def main():
    rospy.init_node("msg_reframer")
    Reframer()
    rospy.spin()

if __name__ == "__main__":
    main()