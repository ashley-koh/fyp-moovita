#!/usr/bin/env python

import rospy
import math
import numpy as np

from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from msg_converter.msg import accuracy

class AccuracyCalculator():

    def __init__(self):

        self.initializing = True
        self.gps_active = False

        self.imu_rot = 0.0
        self.matcher_rot = 0.0
        self.matcher_dist = 0.0
        self.encoder_rot = 0.0
        self.encoder_dist = 0.0
        self.gps_dist = 0.0

        self.header = None
        self.x = None
        self.y = None
        self.z = None
        self.orientation_x = None
        self.orientation_y = None
        self.orientation_z = None
        self.orientation_w = None
        self.linear_x = None
        self.linear_y = None
        self.linear_z = None
        self.angular_x = None
        self.angular_y = None
        self.angular_z = None

        self.map_x = None
        self.map_y = None
        self.map_z = None
        self.map_orientation_x = None
        self.map_orientation_y = None
        self.map_orientation_z = None
        self.map_orientation_w = None

        rospy.Subscriber("imu/data", Imu, callback=self.imu_handler)
        rospy.Subscriber("matcher/PoseWithCovarianceStamped", PoseWithCovarianceStamped, callback=self.matcher_handler)
        rospy.Subscriber("encoder/Odometry", Odometry, callback=self.encoder_handler)
        rospy.Subscriber("odom", Odometry, callback=self.encoder_handler)
        rospy.Subscriber("odometry/gps", Odometry, callback=self.gps_handler)
        rospy.Subscriber("odometry/filtered", Odometry, callback=self.odom_handler)
        rospy.Subscriber("odometry/filtered_map", Odometry, callback=self.map_handler)

        self.acc_pub = rospy.Publisher("sensors/accuracy", accuracy, queue_size=5)

    def imu_handler(self, msg):
        if self.header == None:
            return

        rot_x = abs(self.orientation_x - msg.orientation.x)
        rot_y = abs(self.orientation_y - msg.orientation.y)
        rot_z = abs(self.orientation_z - msg.orientation.z)

        self.imu_rot = math.sqrt(rot_x**2 + rot_y**2 + rot_z**2)

        self.cal_acc(self.imu_rot, self.matcher_rot, self.matcher_dist, self.encoder_rot, self.encoder_dist, self.gps_dist)
        

    def matcher_handler(self, msg):
        if self.header == None:
            return

        dist_x = abs(self.x - msg.pose.pose.position.x)
        dist_y = abs(self.y - msg.pose.pose.position.y)
        dist_z = abs(self.z - msg.pose.pose.position.z)
        self.matcher_dist = math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)

        rot_x = abs(self.orientation_x - msg.pose.pose.orientation.x)
        rot_y = abs(self.orientation_y - msg.pose.pose.orientation.y)
        rot_z = abs(self.orientation_z - msg.pose.pose.orientation.z)
        self.matcher_rot = math.sqrt(rot_x**2 + rot_y**2 + rot_z**2)

        self.cal_acc(self.imu_rot, self.matcher_rot, self.matcher_dist, self.encoder_rot, self.encoder_dist, self.gps_dist)


    def encoder_handler(self, msg):
        if self.header == None:
            return

        dist_x = abs(self.x - msg.pose.pose.position.x)
        dist_y = abs(self.y - msg.pose.pose.position.y)
        dist_z = abs(self.z - msg.pose.pose.position.z)
        self.encoder_dist = math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)

        rot_x = abs(self.orientation_x - msg.pose.pose.orientation.x)
        rot_y = abs(self.orientation_y - msg.pose.pose.orientation.y)
        rot_z = abs(self.orientation_z - msg.pose.pose.orientation.z)
        self.encoder_rot = math.sqrt(rot_x**2 + rot_y**2 + rot_z**2)

        self.cal_acc(self.imu_rot, self.matcher_rot, self.matcher_dist, self.encoder_rot, self.encoder_dist, self.gps_dist)

    def gps_handler(self, msg):
        if self.header == None:
            return

        self.gps_active = True
        dist_x = abs(self.map_x - msg.pose.pose.position.x)
        dist_y = abs(self.map_y - msg.pose.pose.position.y)
        dist_z = abs(self.map_z - msg.pose.pose.position.z)
        self.gps_dist = math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)

        self.cal_acc(self.imu_rot, self.matcher_rot, self.matcher_dist, self.encoder_rot, self.encoder_dist, self.gps_dist)

    def odom_handler(self, msg):
        self.header = msg.header
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.orientation_x = msg.pose.pose.orientation.x
        self.orientation_y = msg.pose.pose.orientation.y
        self.orientation_z = msg.pose.pose.orientation.z
        self.orientation_w = msg.pose.pose.orientation.w
        self.linear_x = msg.twist.twist.linear.x
        self.linear_y = msg.twist.twist.linear.y
        self.linear_z = msg.twist.twist.linear.z
        self.angular_x = msg.twist.twist.angular.x
        self.angular_y = msg.twist.twist.angular.y
        self.angular_z = msg.twist.twist.angular.z

        self.cal_acc(self.imu_rot, self.matcher_rot, self.matcher_dist, self.encoder_rot, self.encoder_dist, self.gps_dist)

    def map_handler(self, msg):
        self.map_x = msg.pose.pose.position.x
        self.map_y = msg.pose.pose.position.y
        self.map_z = msg.pose.pose.position.z
        self.map_orientation_x = msg.pose.pose.orientation.x
        self.map_orientation_y = msg.pose.pose.orientation.y
        self.map_orientation_z = msg.pose.pose.orientation.z
        self.map_orientation_w = msg.pose.pose.orientation.w

    def cal_acc(self, imu_rot, matcher_rot, matcher_dist, encoder_rot, encoder_dist, gps_dist):
        if matcher_dist == 0.0 and encoder_dist == 0.0 and gps_dist == 0:
            return
        
        acc_msg = accuracy()
        acc_msg.header = self.header
        acc_msg.header.frame_id = "accuracy"

        if self.gps_active:
            total_dist = matcher_dist + encoder_dist + gps_dist
            total_rot = imu_rot + matcher_rot + encoder_rot
            acc_msg.imu_acc = (1.0 - (imu_rot / total_rot)) * 100
            acc_msg.mat_acc = (1.0 - (matcher_dist / total_dist)) * 100
            acc_msg.enc_acc = (1.0 - (encoder_dist / total_dist)) * 100
            acc_msg.gps_acc = (1.0 - (gps_dist / total_dist)) * 100

        else:
            total_dist = matcher_dist + encoder_dist
            total_rot = imu_rot + matcher_rot + encoder_rot
            acc_msg.imu_acc = (1.0 - (imu_rot / total_rot)) * 100
            acc_msg.mat_acc = (1.0 - (matcher_dist / total_dist)) * 100
            acc_msg.enc_acc = (1.0 - (encoder_dist / total_dist)) * 100
            acc_msg.gps_acc = -1.0

        self.acc_pub.publish(acc_msg)

def main():
    rospy.init_node("sensor_accuracy")
    AccuracyCalculator()
    rospy.spin()

if __name__ == "__main__":
    main()