#!/usr/bin/env python

import time
import rospy, tf
import math
import numpy as np

from tf.transformations import quaternion_matrix, quaternion_from_matrix
from podzero_message_definitions.msg import pod_to_acs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

class EncoderConverter():

    def __init__(self):

        self.x = 0
        self.y = 0
        self.z = 0
        self.rot = None
        self.init_rot = None
        self.initializing = True
        self.count_init = 0
        self.max_count_init = 10
        self.sum_init = [0, 0, 0, 0]

        self.imu_rot = None

        self.isFirstData = True
        self.wheel_left_count = -1
        self.wheel_right_count = -1
        self.wheel_radius = 0.24
        self.DistPerCount = math.pi * 2 * 0.26 / 4000.0
        self.WheelRadius = 0.25
        self.NumTicksWheelRev = 48./4.

        self.tf_broadcast = tf.TransformBroadcaster()

        self.odom_publisher = rospy.Publisher("encoder/Odometry", Odometry, queue_size=5)
        rospy.Subscriber("pod_to_acs", pod_to_acs, self.parse_serial)
        rospy.Subscriber("imu_node_dual/Imu", Imu, self.imu_handler)
        
    def imu_handler(self, msg):
        if self.initializing:
            if self.count_init == self.max_count_init:
                aver = [x / self.max_count_init for x in self.sum_init]
                print aver
                self.init_rot = np.linalg.inv(tf.transformations.quaternion_matrix(aver))
                self.imu_rot = tf.transformations.quaternion_matrix([1, 0, 0, 0])
                self.initializing = False
            else:
                self.count_init += 1
                rot = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
                self.sum_init = [sum(x) for x in zip(self.sum_init, rot)]
            return

        rot = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        current_rot = np.dot(self.init_rot, tf.transformations.quaternion_matrix(rot))
        rot = quaternion_from_matrix(current_rot)
        #self.rot = self.perturb([rot[0], -rot[1], -rot[2], rot[3]])
        self.rot = [rot[0], -rot[1], -rot[2], rot[3]]
        return

    def parse_serial(self, msg):
        
        current_time = msg.header.stamp
        if self.isFirstData:
            self.last_time = current_time
            self.wheel_left_count = msg.RearLeftWheelCnt
            self.wheel_right_count = msg.RearRightWheelCnt

            if self.wheel_left_count > 0 or self.wheel_right_count > 0:
                self.isFirstData = False
        
        else:
            dt = (msg.header.stamp - self.last_time).to_sec()
            left_counts = msg.RearLeftWheelCnt
            left_nticks = left_counts - self.wheel_left_count
            if left_nticks < -180:
                left_nticks += 255
            if left_nticks > 200:
                left_nticks = 255 - left_nticks

            ld = left_nticks / self.NumTicksWheelRev * 2 * math.pi * self.WheelRadius
            
            right_counts = msg.RearRightWheelCnt
            right_nticks = right_counts - self.wheel_right_count

            if right_nticks < -180:
                right_nticks += 255

            if right_nticks > 200:
                right_nticks = 255 - right_nticks

            rd = right_nticks / self.NumTicksWheelRev * 2 * math.pi * self.WheelRadius

            d = (ld + rd) / 2.0
            d = d / 4.0

            if self.rot == None:
                return
            self.calculate_odom(d, dt, current_time)

            self.wheel_left_count = left_counts
            self.wheel_right_count = right_counts
            self.last_time = current_time

    def calculate_odom(self, d, dt, current_time):
        if self.rot == None:
            return
            
        rot_mat = quaternion_matrix(self.rot)
        dist = np.asarray([d, 0, 0, 1])
        displace = np.dot(rot_mat, dist)
        
        self.x += displace[0]
        self.y += displace[1]
        self.z += displace[2]
        #self.z = self.current_height
        odom_msg = Odometry()

        odom_msg.header.stamp = current_time #+ rospy.Duration(0.015)
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        orientation = tf.transformations.quaternion_from_matrix(rot_mat)
        if dt != 0:
            odom_msg.twist.twist.linear.x = d / dt
        else:
            odom_msg.twist.twist.linear.x = 0
        odom_msg.pose.pose.orientation = Quaternion(*orientation)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.z

        self.tf_broadcast.sendTransform(
            (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
             odom_msg.pose.pose.position.z),
            orientation,
            odom_msg.header.stamp, odom_msg.child_frame_id, odom_msg.header.frame_id)

        self.odom_publisher.publish(odom_msg)

def main():
    rospy.init_node("encoder_converter")
    EncoderConverter()
    rospy.spin()

if __name__ == '__main__':
    main()