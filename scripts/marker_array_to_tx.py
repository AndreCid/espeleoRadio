#!/usr/bin/env python3
"""
Scrip to read the radio signals from markers in RVIZ and data from the map and create a interpolation map

"""

import rospy

import matplotlib.pyplot as plt
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from scipy.interpolate import griddata
from radio_lib import RadioLib


class MapInterpolation:
    def __init__(self):

        # Starting variables

        self.map_topic_name = '/map'
        self.radio_RSSI_signals_topic_name = '/radio/collected_data'

        # Radio variables
        self.radio_RSSI_signals = []
        self.x = []
        self.y = []
        self.RSSI = []
        self.radio_map = []
        self.first_time = True

        self.x_low = 0
        self.y_low = 0
        self.x_max = 0
        self.y_max = 0

        self.type = True
        # Map variables

        self.map_origin = []
        self.map_resolution = 0
        self.map_points = []
        self.map_size = []
        self.explored_map = []
        self.explored_map_x = []
        self.explored_map_y = []

        self.radio_data_size = 0

        # Starting ROS
        self.radio_markers_publisher = None
        self.time_interval = 5
        rospy.init_node('radio_to_txt', anonymous=True)

        self.rate = rospy.Duration(self.time_interval, 0)

        # Waiting for radio signal marker
        self.txt_filename = "radio_markers" + str(rospy.Time.now()) + ".txt"
        # rospy.loginfo("File name: ", self.txt_filename)
        self.file = open(self.txt_filename, 'w')
        self.subscriber_map = rospy.Subscriber(self.map_topic_name,
                                               OccupancyGrid,
                                               self.map_callback)

        while not self.map_points:
            rospy.logwarn("Waiting map topic... ")
            rospy.sleep(self.rate)

        rospy.loginfo("Received map topic! Waiting for radio markers topic...")
        self.subscriber_radio = rospy.Subscriber(self.radio_RSSI_signals_topic_name,
                                                 MarkerArray,
                                                 self.subscriber_radio_callback)
        while not self.RSSI:
            rospy.logwarn("Waiting for radio markers... ")
            rospy.sleep(self.rate)
        rospy.loginfo("Received radio markers!")
        rospy.loginfo("Writing in file")
        self.init_ros()

    # Initializing main loop
    def init_ros(self):

        rospy.sleep(self.rate)
        rospy.loginfo("Marker scripted")

    def subscriber_radio_callback(self, msg):
        if self.first_time:
            for i in range(len(msg.markers)):
                x, y = radio_obj.map_to_coordinates(msg.markers[i].pose.position.x,
                                                    msg.markers[i].pose.position.y,
                                                    self.map_origin,
                                                    self.map_resolution)
                self.x.append(x)
                self.y.append(y)
                self.radio_map.append([self.x, self.y])
                self.RSSI.append(radio_obj.get_rssi_from_rgb(msg.markers[i].color.r, self.type))
                self.file.write(str(radio_obj.get_rssi_from_rgb(msg.markers[i].color.r, self.type)) +
                                "," +
                                str(msg.markers[i].pose.position.x) +
                                "," +
                                str(msg.markers[i].pose.position.y) +
                                "\n")

            self.first_time = False
        else:
            x, y = radio_obj.map_to_coordinates(msg.markers[-1].pose.position.x,
                                                msg.markers[-1].pose.position.y,
                                                self.map_origin,
                                                self.map_resolution)

            self.x.append(x)
            self.y.append(y)
            self.radio_map.append([self.x, self.y])
            self.RSSI.append(radio_obj.get_rssi_from_rgb(msg.markers[-1].color.r, self.type))

        # if len(self.RSSI) < 4:
        #     rospy.logwarn("Waiting for more thant 4 radio signal points...")

    def map_callback(self, msg):
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_resolution = msg.info.resolution
        self.map_size = [msg.info.width, msg.info.height]
        self.map_points = msg.data


"""
Initialize the radio and map objects
"""
if __name__ == '__main__':
    radio_obj = RadioLib()
    map_obj = MapInterpolation()
