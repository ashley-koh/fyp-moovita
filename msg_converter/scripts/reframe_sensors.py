#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, PoseStamped, PoseWithCovarianceStamped

class Reframer():

    def __init__(self):
        self.header = None

        self.imu_frame_id = "base_link"
        self.twist_frame_id = "base_link"
        self.gps_frame_id = "map"
        self.matcher_frame_id = "odom"

        self.imu_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=5)
        self.twist_pub = rospy.Publisher("twist/TwistWithCovarianceStamped", TwistWithCovarianceStamped, queue_size=5)
        self.gps_pub = rospy.Publisher("gps/fix", NavSatFix, queue_size=5)
        self.matcher_pub = rospy.Publisher("matcher/PoseWithCovarianceStamped", PoseWithCovarianceStamped, queue_size=5)

        rospy.Subscriber("an_device/Imu", Imu, callback=self.imu_handler)
        rospy.Subscriber("an_device/Twist", Twist, callback=self.twist_handler)
        rospy.Subscriber("an_device/NavSatFix", NavSatFix, callback=self.gps_handler)
        rospy.Subscriber("matcher_result", PoseStamped, callback=self.matcher_handler)

    def imu_handler(self, msg):
        self.header = msg.header

        imu_msg = msg
        imu_msg.header.frame_id = self.imu_frame_id

        # Convert from NED to ENU
        # x -> y
        # y -> x
        # z -> -z

        # imu_msg.linear_acceleration.x = msg.linear_acceleration.y
        # imu_msg.linear_acceleration.y = msg.linear_acceleration.x
        # imu_msg.linear_acceleration.x = -1 * msg.linear_acceleration.x
        # imu_msg.linear_acceleration.y = -1 * msg.linear_acceleration.y
        # imu_msg.linear_acceleration.z = -1 * msg.linear_acceleration.z
        # imu_msg.angular_velocity.z = -1 * msg.angular_velocity.z


        self.imu_pub.publish(imu_msg)

    def gps_handler(self, msg):
        gps_msg = msg
        gps_msg.header.frame_id = self.gps_frame_id
        self.gps_pub.publish(gps_msg)

    def twist_handler(self, msg):
        twist_msg = TwistWithCovarianceStamped()

        twist_msg.header = self.header
        twist_msg.twist.twist = msg

        twist_msg.twist.twist.angular.z = -1 * msg.angular.z
        self.twist_pub.publish(twist_msg)

    def matcher_handler(self, msg):
        matcher_msg = PoseWithCovarianceStamped()
        matcher_msg.header = msg.header
        matcher_msg.header.frame_id = self.matcher_frame_id
        matcher_msg.pose.pose = msg.pose
        # matcher_msg.pose.pose.position.x = msg.pose.position.y
        # matcher_msg.pose.pose.position.y = msg.pose.position.x

        self.matcher_pub.publish(matcher_msg)


def main():
    rospy.init_node("msg_reframer")
    Reframer()
    rospy.spin()

if __name__ == "__main__":
    main()