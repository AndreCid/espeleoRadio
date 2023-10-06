#!/usr/bin/env python3
"""
Scrip to read the radio signals and plot then in a array of markers in RVIZ

"""

import rospy

import sys, select, termios, tty, math, message_filters

from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from radio_lib import RadioLib


class RadioSignal:
    def __init__(self):

        # Starting variables

        # Class variables
        self.radio_RSSI = 0  # Instant radio signal intensity
        self.x = 0  # Instant robot position (x)
        self.y = 0
        self.x_map, self.y_map = 0, 0
        self.first_time = True
        self.dist = 0
        self.prev_point = [0, 0]
        self.total_distance = 0
        self.total_rssi = []

        self.points = [[0, 0]]  # Array containing robot positions
        self.RSSI = []  # Array containing intensities off radio signal
        self.rate = rospy.Duration(1, 0)

        # Starting ROS
        rospy.init_node('radio_LARS', anonymous=True)
        rospy.loginfo("Gathering data")

        self.subscriber_radio = rospy.Subscriber("/radio/status", Float32MultiArray, self.subscriber_radio_callback)
        self.subscriber_odom = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)
        print("ue")
        self.init_node()

    def init_node(self):
        while not rospy.is_shutdown():
            if self.total_rssi:
                rospy.loginfo("\n PATH DATA: \n Total distance(meters): %s", self.total_distance)
                rospy.loginfo("\n RADIO DATA: \n "
                              "Maximum radio signal: %s \n "
                              "Minimum radio signal: %s \n "
                              "Total radio signal: %s \n "
                              "Average radio signal: %s\n\n",
                              max(self.total_rssi),
                              min(self.total_rssi),
                              sum(self.total_rssi),
                              (sum(self.total_rssi) / len(self.total_rssi)))
            rospy.sleep(self.rate)

    def subscriber_radio_callback(self, message):
        self.radio_RSSI = message.data[1]
        self.total_rssi.append(message.data[1])

    def odom_callback(self, message):
        if self.first_time:
            self.prev_point = [message.pose.pose.position.x, message.pose.pose.position.y]
            self.first_time = False
        actual_point = [message.pose.pose.position.x, message.pose.pose.position.y]
        self.dist = math.sqrt(((actual_point[0] - self.prev_point[0])**2) + ((actual_point[1] - self.prev_point[1])**2))
        self.total_distance = self.total_distance + self.dist
        self.prev_point = [message.pose.pose.position.x, message.pose.pose.position.y]

if __name__ == '__main__':
    radio_obj = RadioLib()
    real_time_radio_signal = RadioSignal()